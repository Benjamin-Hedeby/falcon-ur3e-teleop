#!/usr/bin/env python3
"""Falcon -> UR3e (URSim) teleop bridge.

Hold any falcon grip button (clutch) to engage. Two control modes (parameter `mode`):
  - rate (default): joystick-like. The grip position when the clutch is pressed is the
    stick's center; the further the grip is pushed from it, the faster the TCP moves
    that way (RATE_*). A spring pulls the grip back to center while the clutch is held.
    The commanded speed is scaled by the pendant speed slider so the target never runs
    ahead of the arm, and falcon_node's position-mode drag (speed_damping, clutch hold
    friction) is switched off (RATE_FALCON_PARAMS).
  - position: the robot's TCP translates from wherever it was when the clutch was
    pressed, following scaled Falcon displacement.
  - twist (real robot only): same joystick feel as rate, but the velocity is streamed at
    TWIST_RATE to the UR driver's twist_controller (speedl on the robot) instead of going
    through IK and trajectory goals. Tap the minus button (INSERT_BUTTON) to toggle Insert,
    which works one of two ways (insert_method):
      - admittance (default): the zeroed wrist force drives a virtual mass-damper-spring per
        translation axis, M a + D v + K x = F (adm_mass/adm_damping/adm_stiffness, RK4 every
        twist tick). Its velocity is added to the Falcon's: twist = v_falcon + v_admittance.
        Orientation stays fixed. Anything exceeding adm_max_displacement/adm_max_speed ends Insert.
      - force_mode: UR force mode makes the insert_axes compliant (0 N target, task frame = the
        TCP at that moment, so set the pendant TCP to the peg tip), while the Falcon drives
        only Z.
    twist_controller stops the arm if commands stop for 0.1 s.
In both modes releasing the clutch stops the robot where it is (the active goal is cancelled).
Hold the plus button (button 0) for larger/faster, boosted movements. Orientation is held fixed.

Force feedback (real robot): the UR's wrist force/torque sensor (FT_TOPIC) is zeroed when the
clutch is pressed, rotated into base_link, low-pass filtered and rendered on the Falcon, in all
modes. force_feedback:=false turns contact feedback off (wrist sensor and Gazebo contacts; the
joystick centring spring stays), live too: ros2 param set /falcon_ur_teleop force_feedback false

Contact limit (safety, all modes): the contact wrench (wrist sensor minus one zero kept for the
whole trial, ~50 Hz low-pass) is checked in the force loop against contact_force_limit and
contact_torque_limit. The zero is taken by ~/zero_contact_limit (std_srvs/Trigger; record_trial.sh
calls it at the start of a trial, arm still and out of contact), or else at the first clutch
press. It is not retaken on later presses, so a force left over from a contact still counts.
A trip stops the arm (cancels the trajectory goal / commands zero twist), optionally backs off
contact_backoff along the contact force, and ignores the Falcon until the clutch is released and
pressed again. If the contact is still over the limit on that press, only motion away from the
contact is allowed until it drops below the limit. State on ~/limit_tripped and in the CSV.

Data collection: ~/target (PoseStamped, base_link, flange = tool0, every command tick while the
clutch is held, rate/position modes), ~/clutch and ~/limit_tripped (Bool, every command tick).
Preset: ros2 param load /falcon_ur_teleop config/data_collection.yaml; trials: record_trial.sh.

Live-tunable ROS parameters (ros2 param set /falcon_ur_teleop <name> <value>): speed
(rate_max_speed, rate_boost_max_speed, rate_exponent, rate_spring, max_tcp_speed, max_lead),
force feedback (ft_gain, ft_deadband, ft_sign, ft_filter_hz, ff_max_force) and the contact limit
(contact_force_limit, contact_torque_limit, contact_backoff). Defaults are the constants below.

Logging: set log_file to a .csv path (or a directory, for a timestamped file in it) to record
the force feedback at FF_RATE; set it to '' to stop. See LOG_COLUMNS for the columns. Uses move_group's
/compute_ik service (seeded with current joint state) and streams goals to
scaled_joint_trajectory_controller's FollowJointTrajectory action (each new
goal preempts the last -- this is the controller that's proven to actually
move URSim; forward_position_controller silently accepted commands but
never applied them).

Force feedback (Gazebo): while the clutch is held and the arm touches an
obstacle (contacts on /obstacle_contacts, bridged from Gazebo contact sensors),
the gap between the commanded and actual TCP is rendered on falcon/force as a
spring. The gap is divided by the motion scale, so the stiffness is felt in
hand-space and is the same with or without boost. Gating on contact matters:
normal tracking lag during fast free motion makes a gap just as large.
With no contact publisher (e.g. mock hardware) the force stays zero.
"""
import csv
import math
import os
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped, Vector3, WrenchStamped
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import Joy, JointState
from std_srvs.srv import Trigger
from ur_msgs.srv import SetForceMode
from std_msgs.msg import Bool, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from admittance_insert import AdmittanceInsert, AdmittanceParams, AdmittanceStop
from contact_limit import ContactLimit, away_only
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from tf2_ros import Buffer, TransformListener

JOINT_ORDER = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

SCALE = 2.5          # robot meters per Falcon meter
MAX_RADIUS = 0.15    # m, clamp offset from clutch-engage pose
BOOST_BUTTON = 0     # index in falcon/buttons; 0 = PLUS_BUTTON on the 4-button grip
BOOST_SCALE = 7.5    # robot meters per Falcon meter while the boost button is held
BOOST_MAX_RADIUS = 0.35  # m, clamp offset while boosted
AXIS_SIGN = (-1.0, 1.0, 1.0)  # per-axis Falcon -> base_link direction; -1 mirrors that axis
                              # (also applied to force feedback so forces stay consistent)
MAX_TCP_SPEED = 0.25   # m/s, hard cap on how fast the commanded TCP may move; the robot
                       # lags and catches up if the hand moves faster (falcon_node's
                       # speed_damping makes that felt before the cap is reached)
MAX_LEAD = 0.05       # max_lead: m, the command may run at most this far ahead of the actual
                       # TCP (rate/position modes), so a slow (pendant speed slider) or stopped
                       # robot doesn't make the target run away and trip MAX_JOINT_JUMP
STALL_WARN_TIME = 1.0  # s at max_lead before warning that the robot isn't following

# Contact limit (safety); live parameters with these defaults
CONTACT_FORCE_LIMIT = 20.0   # contact_force_limit: N, 0.0 disables
CONTACT_TORQUE_LIMIT = 2.0   # contact_torque_limit: Nm, 0.0 disables
CONTACT_BACKOFF = 0.0        # contact_backoff: m, retreat along the contact force after a trip
CONTACT_FILTER_HZ = 50.0     # light low-pass so single noise spikes don't trip it
BACKOFF_SPEED = 0.01         # m/s, how fast the backoff moves
MAX_JOINT_JUMP = 0.3  # rad, reject an IK solution that jumps further than this on any joint
                       # from the current state -- guards against the solver picking a
                       # different-but-valid solution branch and winding the joints

# Rate (joystick) mode
RATE_DEADZONE = 0.005    # m of grip deflection that commands no motion
RATE_FULL = 0.04         # m of grip deflection that commands full speed
RATE_MAX_SPEED = 0.08    # m/s TCP speed at full deflection
RATE_BOOST_MAX_SPEED = 0.2   # m/s at full deflection while boosted (keep below MAX_TCP_SPEED)
RATE_EXPONENT = 2.0      # speed curve; >1 gives finer control near the center
RATE_SPRING = 50.0       # N/m, pulls the grip back to center while the clutch is held
RATE_FALCON_PARAMS = {   # falcon_node haptics meant for position mode, overridden in rate mode
    'speed_damping': 0.0,      # drag on fast hand motion; hand speed isn't robot speed here
    'hold_force_active': 0.0,  # friction while the clutch is held; the spring centers instead
}
RATE_CONTACT_STIFFNESS = 80.0  # N/m per m the robot lags the target while in contact (Gazebo)

SINGULAR_MARGIN = 0.15  # rad, how close to a singular joint angle counts as "near" for the IK hint

# moveit_msgs/MoveItErrorCodes values that /compute_ik can return
IK_ERRORS = {-31: 'NO_IK_SOLUTION', -6: 'TIMED_OUT', -10: 'START_STATE_IN_COLLISION',
             -12: 'GOAL_IN_COLLISION', -15: 'INVALID_GROUP_NAME', -21: 'FRAME_TRANSFORM_FAILURE'}

FF_STIFFNESS = 250.0   # N/m, spring felt at the Falcon per meter of hand-space gap
FF_DEADBAND = 0.005    # m, robot-space gap ignored (steady tracking error)
FF_MAX_FORCE = 5.0     # N, total force sent to the Falcon (falcon_node clamps to its own max_force)
FF_FILTER = 0.3        # 0..1, low-pass factor per force tick; smooths contact on/off edges
FF_RATE = 200.0        # Hz, force update rate
# Twist mode (real robot)
TWIST_RATE = 100.0       # Hz; twist_controller commands zero after 0.1 s without a command
TWIST_TOPIC = '/twist_controller/twist'
INSERT_BUTTON = 3        # minus button: tap to toggle Insert; not a clutch button in twist mode
INSERT_SERVICE_TIMEOUT = 5.0  # s to wait for zero_ftsensor/start/stop_force_mode before giving up
TWIST_CONTROLLERS = ['twist_controller', 'force_mode_controller']
MOTION_CONTROLLERS = [   # mutually exclusive with twist_controller; deactivated in twist mode
    'scaled_joint_trajectory_controller', 'joint_trajectory_controller',
    'passthrough_trajectory_controller', 'forward_velocity_controller',
    'forward_position_controller', 'forward_effort_controller', 'freedrive_mode_controller',
]
# Insert (UR force mode) defaults, read when Insert is switched on (ros2 param set to change):
INSERT_AXES = 'x,y,rz'          # insert_axes: compliant axes of the TCP frame (x,y,z,rx,ry,rz)
INSERT_SPEED_LIMIT = 0.02       # insert_speed_limit: m/s, max speed along compliant axes
INSERT_ROT_SPEED_LIMIT = 0.1    # insert_rot_speed_limit: rad/s, about compliant axes
INSERT_DAMPING = 0.1            # insert_damping: UR force mode damping factor, 0..1
INSERT_GAIN_SCALING = 0.5       # insert_gain_scaling: UR force mode gain scaling, 0..2 (>1 unstable)

# Admittance Insert defaults (live: ros2 param set /falcon_ur_teleop <name> <value>). Per axis
# X, Y, Z in base_link. Z has no spring, so pressing on a hard stop at Falcon speed v settles at
# F = D_z * v: D_z sets the insertion force per unit commanded speed.
INSERT_METHOD = 'admittance'          # insert_method: 'admittance' or 'force_mode'
ADM_MASS = [40.0, 40.0, 60.0]          # adm_mass: kg (virtual); M/D is the response time (40 ms sideways)
ADM_DAMPING = [1000.0, 1000.0, 1000.0]  # adm_damping: N s/m; steady speed = F / D (sideways: 2026-11-24 sweep)
ADM_STIFFNESS = [400.0, 400.0, 0.0]     # adm_stiffness: N/m; steady deflection = F / K
ADM_FILTER_HZ = 10.0         # adm_filter_hz: low-pass on the force input (0 = off)
ADM_DEADBAND = 1.0           # adm_deadband: N, force magnitude ignored (sensor noise -> drift)
ADM_MAX_DISPLACEMENT = 0.03  # adm_max_displacement: m of admittance motion before Insert stops
ADM_MAX_SPEED = 0.05         # adm_max_speed: m/s of admittance speed before Insert stops

# CSV log, one row per force tick (FF_RATE). Vectors are in base_link unless noted.
LOG_COLUMNS = [
    'time',                                    # s, wall clock (Unix time)
    'clutch', 'boost',                         # 0/1
    'falcon_x', 'falcon_y', 'falcon_z',        # m, grip position (Falcon axes, as published)
    'tcp_x', 'tcp_y', 'tcp_z',                 # m, actual TCP
    'target_x', 'target_y', 'target_z',        # m, last commanded TCP
    'sensor_x', 'sensor_y', 'sensor_z',        # N, wrist sensor force, raw
    'contact_x', 'contact_y', 'contact_z',     # N, sensor minus clutch-press zero, low-passed
                                               #    (updated only while the clutch is held)
    'ft_force_x', 'ft_force_y', 'ft_force_z',  # N, contact part of the Falcon force
    'force_x', 'force_y', 'force_z',           # N, total Falcon force sent (spring + contact)
    'speed_scaling',                           # %, pendant speed slider
    'insert',                                  # 0/1, twist-mode Insert active
    'adm_force_x', 'adm_force_y', 'adm_force_z',  # N, admittance input (zeroed, filtered)
    'adm_x', 'adm_y', 'adm_z',                 # m, admittance displacement
    'adm_vx', 'adm_vy', 'adm_vz',              # m/s, admittance velocity (added to the twist)
    'torque_x', 'torque_y', 'torque_z',        # Nm, wrist sensor torque, raw, rotated into
                                               #    base_link like sensor_*: torque about the
                                               #    sensor's reference point (the TCP on the UR)
    'limit_tripped',                           # 0/1, contact limit latched
    'ros_time',                                # s, node clock (matches bag timestamps)
    'limit_recovering',                        # 0/1, tripped and re-pressed while still over the
                                               #    limit: only motion away from the contact
]

CONTACT_TIMEOUT = 0.05  # s, how long a reported contact counts as ongoing

# Real robot: wrist force/torque sensor -> Falcon
FT_TOPIC = '/force_torque_sensor_broadcaster/wrench'
# Defaults for the live-tunable parameters (ros2 param set /falcon_ur_teleop <name> <value>):
FT_GAIN = 0.2        # ft_gain: N at the Falcon per N measured at the tool
FT_DEADBAND = 4.0    # ft_deadband: N, ignored (sensor noise, inertia of the tool while moving)
FT_SIGN = 1.0        # ft_sign: +1 = Falcon is pushed the way the environment pushes the tool
FT_FILTER_HZ = 5.0   # ft_filter_hz: low-pass cutoff; contacts are slow, motion jitter is fast and
                     # would otherwise feed back through the hand into the joystick command
FT_TIMEOUT = 0.1     # s, stop rendering if the sensor goes quiet


def rotate(q, v):
    """Rotate vector v by quaternion q (geometry_msgs Quaternion)."""
    x, y, z, w = q.x, q.y, q.z, q.w
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v); v' = v + w*t + cross(q.xyz, t)
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


class FalconURTeleop(Node):
    def __init__(self):
        super().__init__('falcon_ur_teleop')
        self.mode = self.declare_parameter('mode', 'rate').value
        self.ft_gain = self.declare_parameter('ft_gain', FT_GAIN).value
        self.force_feedback = self.declare_parameter('force_feedback', True).value
        self.ft_deadband = self.declare_parameter('ft_deadband', FT_DEADBAND).value
        self.ft_sign = self.declare_parameter('ft_sign', FT_SIGN).value
        self.ft_filter_hz = self.declare_parameter('ft_filter_hz', FT_FILTER_HZ).value
        self.ff_max_force = self.declare_parameter('ff_max_force', FF_MAX_FORCE).value
        self.rate_max_speed = self.declare_parameter('rate_max_speed', RATE_MAX_SPEED).value
        self.rate_boost_max_speed = self.declare_parameter(
            'rate_boost_max_speed', RATE_BOOST_MAX_SPEED).value
        self.rate_exponent = self.declare_parameter('rate_exponent', RATE_EXPONENT).value
        self.rate_spring = self.declare_parameter('rate_spring', RATE_SPRING).value
        self.max_tcp_speed = self.declare_parameter('max_tcp_speed', MAX_TCP_SPEED).value
        self.max_lead = self.declare_parameter('max_lead', MAX_LEAD, ParameterDescriptor(
            description='m, max distance the commanded target may run ahead of the actual TCP '
                        '(rate/position modes); > 0')).value
        self.contact_force_limit = self.declare_parameter(
            'contact_force_limit', CONTACT_FORCE_LIMIT, ParameterDescriptor(
                description='N, contact force that stops the arm until the clutch is re-pressed; '
                            '0.0 disables')).value
        self.contact_torque_limit = self.declare_parameter(
            'contact_torque_limit', CONTACT_TORQUE_LIMIT, ParameterDescriptor(
                description='Nm, contact torque that stops the arm until the clutch is re-pressed; '
                            '0.0 disables')).value
        self.contact_backoff = self.declare_parameter(
            'contact_backoff', CONTACT_BACKOFF, ParameterDescriptor(
                description='m, retreat along the contact force after a contact-limit trip; '
                            '0.0 = just stop')).value
        self.add_on_set_parameters_callback(self.on_set_params)
        self.ft_filtered = (0.0, 0.0, 0.0)
        self.log = None              # (file, csv.writer) while logging
        self.log_rows = 0
        self.declare_parameter('log_file', '')  # declaring runs on_set_params, which opens it
        if self.mode not in ('rate', 'position', 'twist'):
            raise ValueError(f"mode must be 'rate', 'position' or 'twist', got '{self.mode}'")
        self.joystick = self.mode in ('rate', 'twist')  # grip offset sets a velocity
        self.declare_parameter('insert_axes', INSERT_AXES)
        self.declare_parameter('insert_speed_limit', INSERT_SPEED_LIMIT)
        self.declare_parameter('insert_rot_speed_limit', INSERT_ROT_SPEED_LIMIT)
        self.declare_parameter('insert_damping', INSERT_DAMPING)
        self.declare_parameter('insert_gain_scaling', INSERT_GAIN_SCALING)
        self.declare_parameter('insert_method', INSERT_METHOD)
        self.adm_mass = list(self.declare_parameter('adm_mass', ADM_MASS).value)
        self.adm_damping = list(self.declare_parameter('adm_damping', ADM_DAMPING).value)
        self.adm_stiffness = list(self.declare_parameter('adm_stiffness', ADM_STIFFNESS).value)
        self.adm_filter_hz = self.declare_parameter('adm_filter_hz', ADM_FILTER_HZ).value
        self.adm_deadband = self.declare_parameter('adm_deadband', ADM_DEADBAND).value
        self.adm_max_displacement = self.declare_parameter(
            'adm_max_displacement', ADM_MAX_DISPLACEMENT).value
        self.adm_max_speed = self.declare_parameter('adm_max_speed', ADM_MAX_SPEED).value

        self.falcon_pos = None
        self.clutch = False
        self.boost = False
        self.prev_boost = False
        self.last_target = None  # (x, y, z) of the last commanded TCP position
        self.prev_clutch = False
        self.center_falcon = None
        self.center_tcp = None  # (x, y, z, qx, qy, qz, qw)
        self.joint_state = None
        self.last_contact = None  # time of the last reported obstacle contact
        self.force = (0.0, 0.0, 0.0)
        self.stall_since = None
        self.speed_scaling = None    # percent, from the real/mock UR driver
        self.program_running = None  # External Control program state (real robot only)
        self.goal_handle = None      # last accepted trajectory goal, cancelled on clutch release
        self.ft_msg = None           # latest WrenchStamped from the wrist sensor
        self.ft_time = None
        self.ft_bias = None          # sensor force (base_link) just after the clutch was pressed
        self.contact = ContactLimit()
        self.cl_bias = None          # (force, torque) zero for the contact limit, kept for the trial
        self.cl_zero_request = False  # ~/zero_contact_limit called: zero on the next reading
        self.cl_restart_filter = True  # start the contact filter from the current value
        self.cl_engaged = False      # engaged at the previous force tick (to catch the press)
        self.cl_wrench = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))  # filtered contact (force, torque)
        self.backoff_left = 0.0      # m of backoff still to move (twist mode)
        self.backoff_dir = (0.0, 0.0, 0.0)

        self.create_subscription(PointStamped, 'falcon/position', self.on_falcon_pos, 10)
        self.create_subscription(Joy, 'falcon/buttons', self.on_buttons, 10)
        # Depth 1: only the latest sample matters, and a busy executor then skips stale 500 Hz
        # messages instead of working through a backlog (which delays the command timer)
        if self.mode != 'twist':  # only the IK modes need joint states (500 Hz of messages)
            self.create_subscription(JointState, 'joint_states', self.on_joint_state, 1)
        self.create_subscription(Contacts, '/obstacle_contacts', self.on_contacts, 10)
        self.create_subscription(WrenchStamped, FT_TOPIC, self.on_wrench, 1)
        self.force_pub = self.create_publisher(Vector3, 'falcon/force', 10)
        self.target_pub = self.create_publisher(PoseStamped, '~/target', 10)
        self.clutch_pub = self.create_publisher(Bool, '~/clutch', 10)
        self.tripped_pub = self.create_publisher(Bool, '~/limit_tripped', 10)
        self.create_service(Trigger, '~/zero_contact_limit', self.on_zero_contact_limit)
        self.create_subscription(
            Float64, '/speed_scaling_state_broadcaster/speed_scaling',
            lambda m: setattr(self, 'speed_scaling', m.data), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            Bool, '/io_and_status_controller/robot_program_running',
            lambda m: setattr(self, 'program_running', m.data), latched)

        if self.mode != 'twist':
            self.traj_client = ActionClient(
                self, FollowJointTrajectory,
                '/scaled_joint_trajectory_controller/follow_joint_trajectory')
            self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        self.tf_buffer = Buffer()
        # Own node and thread: on the real robot /tf carries tool0_controller at 500 Hz, which
        # would otherwise compete with the command and force timers in this node's executor.
        # node=None matters: given this node, tf2_ros would spin ALL of its callbacks in that
        # thread too, concurrently with rclpy.spin() in main().
        self.tf_listener = TransformListener(self.tf_buffer, None, spin_thread=True)

        self.timer = self.create_timer(0.05, self.on_timer)  # 20 Hz
        self.force_timer = self.create_timer(1.0 / FF_RATE, self.on_force_timer)
        if self.mode == 'twist':
            self.twist_pub = self.create_publisher(TwistStamped, TWIST_TOPIC, 10)
            self.cm_list = self.create_client(ListControllers, '/controller_manager/list_controllers')
            self.cm_switch = self.create_client(SwitchController, '/controller_manager/switch_controller')
            self.zero_ft_client = self.create_client(Trigger, '/io_and_status_controller/zero_ftsensor')
            self.fm_start = self.create_client(SetForceMode, '/force_mode_controller/start_force_mode')
            self.fm_stop = self.create_client(Trigger, '/force_mode_controller/stop_force_mode')
            self.controllers_ready = False
            self.cm_busy = False
            self.insert = False             # force mode on
            self.insert_busy = False        # a force mode service call is in flight
            self.insert_busy_since = None
            self.insert_token = 0           # bumped per toggle; late answers to old toggles are ignored
            self.insert_requested = False   # minus button tapped
            self.prev_insert_button = False
            self.base_rot = None            # rotation base_link -> base (the twist frame)
            self.insert_method = None       # method of the active Insert
            self.adm = AdmittanceInsert()   # shared with the MuJoCo peg-in-hole simulation
            self.motion_lockout = False     # after an admittance stop: no motion until clutch release
            self.last_twist_time = None
            self.create_timer(2.0, self.ensure_twist_controllers)
            self.twist_timer = self.create_timer(1.0 / TWIST_RATE, self.on_twist_timer)
        if self.joystick:
            # Re-applied whenever falcon_node (re)appears, e.g. when it runs in its own terminal
            self.falcon_params = AsyncParameterClient(self, 'falcon_node')
            self.falcon_params_set = False
            self.create_timer(1.0, self.apply_falcon_params)
        if self.mode == 'rate':
            self.get_logger().info('Falcon->UR teleop ready (rate mode). Hold any falcon button and push '
                                   'the grip like a joystick; plus button for faster movements.')
        elif self.mode == 'twist':
            self.get_logger().info('Falcon->UR teleop ready (twist mode). Hold any falcon button except '
                                   'minus and push the grip like a joystick; plus button for faster '
                                   f"movements; tap minus to toggle Insert ({self.get_parameter('insert_method').value}).")
        else:
            self.get_logger().info('Falcon->UR teleop ready (position mode). Hold any falcon button to '
                                   'move the robot, plus button for larger movements.')

    def lookup_tcp_in_base_link(self):
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'tool0', Time())
        except Exception:
            return None
        p, q = t.transform.translation, t.transform.rotation
        return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def on_falcon_pos(self, msg: PointStamped):
        self.falcon_pos = (msg.point.x, msg.point.y, msg.point.z)

    def on_buttons(self, msg: Joy):
        buttons = list(msg.buttons)
        if self.mode == 'twist':
            pressed = len(buttons) > INSERT_BUTTON and bool(buttons[INSERT_BUTTON])
            if pressed and not self.prev_insert_button:
                self.insert_requested = True
            self.prev_insert_button = pressed
            buttons = [b for i, b in enumerate(buttons) if i != INSERT_BUTTON]
        self.clutch = any(buttons)
        self.boost = len(msg.buttons) > BOOST_BUTTON and bool(msg.buttons[BOOST_BUTTON])

    def on_joint_state(self, msg: JointState):
        self.joint_state = msg

    TUNABLE = ('adm_filter_hz', 'adm_deadband', 'adm_max_displacement', 'adm_max_speed',
               'ft_gain', 'ft_deadband', 'ft_sign', 'ft_filter_hz', 'ff_max_force',
               'rate_max_speed', 'rate_boost_max_speed', 'rate_exponent', 'rate_spring',
               'max_tcp_speed', 'max_lead', 'contact_force_limit', 'contact_torque_limit',
               'contact_backoff')
    ADM_ARRAYS = ('adm_mass', 'adm_damping', 'adm_stiffness')
    NON_NEGATIVE = ('adm_filter_hz', 'adm_deadband', 'adm_max_displacement', 'adm_max_speed',
                    'ft_gain', 'ft_deadband', 'ff_max_force', 'rate_max_speed',
                    'rate_boost_max_speed', 'rate_spring', 'max_tcp_speed',
                    'contact_force_limit', 'contact_torque_limit', 'contact_backoff')

    def open_log(self, path):
        """Start logging to path (a .csv file, or a directory for a timestamped file)."""
        path = os.path.expanduser(path)
        if os.path.isdir(path) or not path.endswith('.csv'):
            os.makedirs(path, exist_ok=True)
            path = os.path.join(path, time.strftime('falcon_ff_%Y%m%d-%H%M%S.csv'))
        else:
            os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        f = open(path, 'w', newline='')
        self.close_log()
        self.log = (f, csv.writer(f))
        self.log[1].writerow(LOG_COLUMNS)
        self.log_rows = 0
        self.get_logger().info(f'Logging force feedback to {path}')

    def close_log(self):
        if self.log is not None:
            self.log[0].close()
            self.get_logger().info(f'Log closed ({self.log_rows} rows)')
            self.log = None

    def write_log_row(self, sensor, ft, force, torque=None):
        tcp = self.lookup_tcp_in_base_link()
        nan3 = (math.nan,) * 3
        row = [f'{time.time():.4f}', int(self.clutch), int(self.boost)]
        for v in (self.falcon_pos or nan3, tcp[:3] if tcp else nan3, self.last_target or nan3,
                  sensor or nan3, self.ft_filtered, ft, force):
            row += [f'{c:.5f}' for c in v]
        row.append(f'{self.speed_scaling:.1f}' if self.speed_scaling is not None else 'nan')
        insert = self.mode == 'twist' and getattr(self, 'insert', False)  # not set up yet at start
        adm_on = insert and self.insert_method == 'admittance'
        row.append(int(insert))
        for v in ((self.adm.force, self.adm.model.x, self.adm.model.v) if adm_on else (nan3, nan3, nan3)):
            row += [f'{c:.5f}' for c in v]
        row += [f'{c:.5f}' for c in (torque or nan3)]
        row.append(int(self.contact.tripped))
        row.append(f'{self.get_clock().now().nanoseconds * 1e-9:.4f}')
        row.append(int(self.contact.recovering))
        self.log[1].writerow(row)
        self.log_rows += 1
        if self.log_rows % int(FF_RATE) == 0:  # flush about once a second
            self.log[0].flush()

    def on_set_params(self, params):
        for p in params:
            if p.name == 'log_file':
                if p.type_ != Parameter.Type.STRING:
                    return SetParametersResult(successful=False, reason='log_file must be a string')
                try:
                    if p.value:
                        self.open_log(p.value)
                    else:
                        self.close_log()
                except OSError as e:
                    return SetParametersResult(successful=False, reason=f'Cannot open log: {e}')
            if p.name == 'force_feedback':
                if p.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(successful=False,
                                               reason='force_feedback must be true or false')
                self.force_feedback = p.value
                self.get_logger().info('Contact force feedback on the Falcon: '
                                       + ('ON' if p.value else 'OFF'))
            if p.name == 'insert_method' and p.value not in ('admittance', 'force_mode'):
                return SetParametersResult(successful=False,
                                           reason="insert_method must be 'admittance' or 'force_mode'")
            if p.name in self.ADM_ARRAYS:
                vals = list(p.value) if p.type_ == Parameter.Type.DOUBLE_ARRAY else []
                if len(vals) != 3 or not all(math.isfinite(v) for v in vals):
                    return SetParametersResult(successful=False,
                                               reason=f'{p.name} must be 3 decimals [x, y, z]')
                if p.name == 'adm_mass' and min(vals) <= 0.0:
                    return SetParametersResult(successful=False, reason='adm_mass must be > 0')
                if min(vals) < 0.0:
                    return SetParametersResult(successful=False, reason=f'{p.name} must be >= 0')
                continue
            if p.name not in self.TUNABLE:
                continue
            if p.type_ not in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                return SetParametersResult(successful=False, reason=f'{p.name} must be a number')
            v = float(p.value)
            if p.name in self.NON_NEGATIVE and v < 0.0:
                return SetParametersResult(successful=False, reason=f'{p.name} must be >= 0')
            if p.name in ('rate_exponent', 'max_lead') and v <= 0.0:
                return SetParametersResult(successful=False, reason=f'{p.name} must be > 0')
        for p in params:
            if p.name in self.TUNABLE:
                setattr(self, p.name, float(p.value))
                self.get_logger().info(f'{p.name} = {float(p.value)}')
            elif p.name in self.ADM_ARRAYS:
                setattr(self, p.name, [float(v) for v in p.value])
                self.get_logger().info(f'{p.name} = {list(p.value)}')
        return SetParametersResult(successful=True)

    def on_wrench(self, msg: WrenchStamped):
        if self.ft_msg is None:
            self.get_logger().info(f'Receiving {FT_TOPIC}: force feedback from the wrist sensor is on')
        self.ft_msg = msg
        self.ft_time = self.get_clock().now()

    def ft_force_in_base_link(self):
        """Latest wrist-sensor force rotated into base_link, or None if stale/unavailable."""
        w = self.ft_wrench_in_base_link()
        return w[0] if w is not None else None

    def ft_wrench_in_base_link(self):
        """Latest wrist-sensor (force, torque) rotated into base_link, or None if stale/unavailable.
        The torque stays about the sensor's reference point (the TCP on the UR)."""
        if self.ft_msg is None or (self.get_clock().now() - self.ft_time).nanoseconds * 1e-9 > FT_TIMEOUT:
            return None
        # tool0_controller (the pendant's TCP) is only valid on a real robot (mock hardware publishes
        # it as NaN); tool0 (the flange) has the same orientation unless the pendant TCP is rotated
        for frame in (self.ft_msg.header.frame_id, 'tool0'):
            try:
                q = self.tf_buffer.lookup_transform('base_link', frame, Time()).transform.rotation
            except Exception:
                continue
            if all(math.isfinite(c) for c in (q.x, q.y, q.z, q.w)):
                break
        else:
            self.get_logger().warn(f'No TF base_link->{self.ft_msg.header.frame_id}, no force feedback',
                                   throttle_duration_sec=5.0)
            return None
        f, t = self.ft_msg.wrench.force, self.ft_msg.wrench.torque
        f, t = rotate(q, (f.x, f.y, f.z)), rotate(q, (t.x, t.y, t.z))
        return (f, t) if all(math.isfinite(c) for c in f + t) else None

    def ft_feedback(self, f):
        """Contact force to render on the Falcon (base_link) from sensor force f (base_link),
        relative to the reading just after the clutch was pressed."""
        if f is None:
            return (0.0, 0.0, 0.0)
        if self.ft_bias is None:
            self.ft_bias = f
            self.ft_filtered = (0.0, 0.0, 0.0)
            return (0.0, 0.0, 0.0)
        # First-order low-pass at ft_filter_hz, run at FF_RATE
        dt = 1.0 / FF_RATE
        a = dt / (dt + 1.0 / (2.0 * math.pi * max(self.ft_filter_hz, 0.1)))
        self.ft_filtered = tuple(p + a * ((f[i] - self.ft_bias[i]) - p)
                                 for i, p in enumerate(self.ft_filtered))
        ext = self.ft_filtered
        mag = math.sqrt(sum(c * c for c in ext))
        # With force_feedback off the filter above still runs, so the log keeps its contact column
        if not self.force_feedback or mag <= self.ft_deadband:
            return (0.0, 0.0, 0.0)
        k = self.ft_sign * self.ft_gain * (mag - self.ft_deadband) / mag
        return tuple(k * c for c in ext)

    def on_contacts(self, msg: Contacts):
        if msg.contacts:
            self.last_contact = self.get_clock().now()

    def compute_target(self):
        """TCP position (base_link) that the current Falcon position maps to."""
        # prev_boost is the scale the current anchor was set with; self.boost may
        # already have flipped before on_timer re-anchors.
        scale = BOOST_SCALE if self.prev_boost else SCALE
        max_radius = BOOST_MAX_RADIUS if self.prev_boost else MAX_RADIUS
        dx = (self.falcon_pos[0] - self.center_falcon[0]) * scale * AXIS_SIGN[0]
        dy = (self.falcon_pos[1] - self.center_falcon[1]) * scale * AXIS_SIGN[1]
        dz = (self.falcon_pos[2] - self.center_falcon[2]) * scale * AXIS_SIGN[2]
        r = math.sqrt(dx * dx + dy * dy + dz * dz)
        if r > max_radius:
            s = max_radius / r
            dx, dy, dz = dx * s, dy * s, dz * s
        cx, cy, cz = self.center_tcp[:3]
        return (cx + dx, cy + dy, cz + dz)

    def apply_falcon_params(self):
        if not self.falcon_params.services_are_ready():
            self.falcon_params_set = False
            return
        if self.falcon_params_set:
            return
        self.falcon_params_set = True
        params = [Parameter(k, Parameter.Type.DOUBLE, v) for k, v in RATE_FALCON_PARAMS.items()]
        self.falcon_params.set_parameters(params).add_done_callback(
            lambda f: self.get_logger().info(f'Set falcon_node {RATE_FALCON_PARAMS} for rate mode'))

    def stop_robot(self):
        """Cancel the active trajectory goal; the controller holds the current position."""
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def rate_velocity(self):
        """TCP velocity (base_link, m/s) for the current grip deflection in rate mode."""
        d = [(self.falcon_pos[i] - self.center_falcon[i]) * AXIS_SIGN[i] for i in range(3)]
        r = math.sqrt(sum(c * c for c in d))
        if r <= RATE_DEADZONE:
            return (0.0, 0.0, 0.0)
        u = min((r - RATE_DEADZONE) / (RATE_FULL - RATE_DEADZONE), 1.0) ** self.rate_exponent
        speed = u * (self.rate_boost_max_speed if self.boost else self.rate_max_speed)
        if self.speed_scaling is not None:
            # The pendant slider slows the arm; slow the target too so it doesn't run ahead
            speed *= max(0.0, min(self.speed_scaling, 100.0)) / 100.0
        return tuple(c / r * speed for c in d)

    def rate_force(self, in_contact):
        """Centering spring (+ contact resistance) in base_link for rate mode."""
        f = [-self.rate_spring * (self.falcon_pos[i] - self.center_falcon[i]) * AXIS_SIGN[i]
             for i in range(3)]
        if in_contact and self.last_target is not None:
            tcp = self.lookup_tcp_in_base_link()
            if tcp is not None:
                # The target runs ahead of a blocked arm (up to max_lead); push the hand back
                gap = [tcp[i] - self.last_target[i] for i in range(3)]
                mag = math.sqrt(sum(g * g for g in gap))
                if mag > FF_DEADBAND:
                    k = RATE_CONTACT_STIFFNESS * (mag - FF_DEADBAND) / mag
                    f = [f[i] + k * gap[i] for i in range(3)]
        fmag = math.sqrt(sum(c * c for c in f))
        if fmag > FF_MAX_FORCE:
            f = [c * FF_MAX_FORCE / fmag for c in f]
        return tuple(f)

    def on_force_timer(self):
        f = (0.0, 0.0, 0.0)
        in_contact = (self.force_feedback and self.last_contact is not None and
                      (self.get_clock().now() - self.last_contact).nanoseconds * 1e-9 < CONTACT_TIMEOUT)
        if self.joystick:
            if self.clutch and self.center_falcon is not None:
                f = self.rate_force(in_contact)
        elif in_contact and self.clutch and self.center_falcon is not None:
            tcp = self.lookup_tcp_in_base_link()
            if tcp is not None:
                # Computed from the live Falcon position (not the 20 Hz IK goal)
                # so the spring reacts to the hand without extra delay.
                target = self.compute_target()
                gap = [tcp[i] - target[i] for i in range(3)]  # points from target back to the arm
                mag = math.sqrt(sum(g * g for g in gap))
                if mag > FF_DEADBAND:
                    scale = BOOST_SCALE if self.prev_boost else SCALE
                    k = FF_STIFFNESS * (mag - FF_DEADBAND) / mag / scale
                    f = tuple(k * g for g in gap)
                    fmag = math.sqrt(sum(c * c for c in f))
                    if fmag > FF_MAX_FORCE:
                        f = tuple(c * FF_MAX_FORCE / fmag for c in f)
        ft = (0.0, 0.0, 0.0)
        engaged = self.clutch and self.center_falcon is not None
        wrench = self.ft_wrench_in_base_link() if engaged or self.log or self.cl_zero_request \
            else None
        sensor = wrench[0] if wrench is not None else None
        self.check_contact_limit(wrench, engaged)
        if engaged:
            ft = self.ft_feedback(sensor)
            f = tuple(f[i] + ft[i] for i in range(3))
        fmag = math.sqrt(sum(c * c for c in f))
        if fmag > self.ff_max_force:
            f = tuple(c * self.ff_max_force / fmag for c in f)
        if not all(math.isfinite(c) for c in f):
            f = (0.0, 0.0, 0.0)
        self.force = tuple(p + FF_FILTER * (c - p) for p, c in zip(self.force, f))
        # force is in base_link; map it back into Falcon axes
        fx, fy, fz = (c * s for c, s in zip(self.force, AXIS_SIGN))
        self.force_pub.publish(Vector3(x=fx, y=fy, z=fz))
        if self.log is not None:
            self.write_log_row(sensor, ft, self.force, wrench[1] if wrench is not None else None)

    # --- Contact limit ------------------------------------------------------------------------

    def on_zero_contact_limit(self, req, res):
        """Retake the contact-limit zero on the next sensor reading (arm still, out of contact)."""
        self.cl_zero_request = True
        res.success = True
        res.message = 'contact-limit zero will be taken from the next wrist-sensor reading'
        return res

    def check_contact_limit(self, wrench, engaged):
        """Every force tick: filter the contact wrench and update the latching contact limit."""
        was_tripped, was_recovering = self.contact.tripped, self.contact.recovering
        if wrench is not None and (self.cl_zero_request or (engaged and self.cl_bias is None)):
            self.cl_bias = wrench
            self.cl_restart_filter = True
            self.get_logger().info(
                'Contact-limit zero taken ' + ('(~/zero_contact_limit)' if self.cl_zero_request else
                                               'at the first clutch press; kept for the trial'))
            self.cl_zero_request = False
        if engaged and not self.cl_engaged:
            self.cl_restart_filter = True  # a re-press under load must not ramp up from zero
        self.cl_engaged = engaged
        contact = None
        if engaged and wrench is not None and self.cl_bias is not None:
            raw = tuple(tuple(wrench[k][i] - self.cl_bias[k][i] for i in range(3)) for k in range(2))
            if self.cl_restart_filter:
                self.cl_wrench = raw
                self.cl_restart_filter = False
            else:
                dt = 1.0 / FF_RATE
                a = dt / (dt + 1.0 / (2.0 * math.pi * CONTACT_FILTER_HZ))
                self.cl_wrench = tuple(tuple(p + a * (raw[k][i] - p)
                                             for i, p in enumerate(self.cl_wrench[k]))
                                       for k in range(2))
            contact = self.cl_wrench
        tripped_now = self.contact.update(
            contact[0] if contact else None, contact[1] if contact else None, engaged,
            self.contact_force_limit, self.contact_torque_limit)
        if tripped_now:
            self.on_contact_trip(contact[0])
        if self.contact.recovering and not was_recovering:
            self.get_logger().warn(
                f'Contact limit: {self.contact.reason} on re-press. Only motion away from the '
                'contact is allowed until it drops below the limit.', throttle_duration_sec=1.0)
        if self.contact.tripped != was_tripped:
            self.tripped_pub.publish(Bool(data=self.contact.tripped))
            if not self.contact.tripped:
                self.get_logger().info('Contact limit cleared' + (
                    ' (moved away from the contact)' if was_recovering else ' (clutch re-pressed)'))

    def contact_away(self):
        """The filtered contact force on the tool (base_link): points away from the contact."""
        return tuple(self.ft_sign * c for c in self.cl_wrench[0])

    def on_contact_trip(self, force):
        """Stop the arm, optionally back off along the contact force, and log."""
        self.get_logger().warn(
            f'CONTACT LIMIT: {self.contact.reason}. Arm stopped; release and re-press the clutch '
            f'to continue. (force {tuple(round(c, 1) for c in self.cl_wrench[0])} N, '
            f'torque {tuple(round(c, 2) for c in self.cl_wrench[1])} Nm, base_link)',
            throttle_duration_sec=1.0)
        fmag = math.sqrt(sum(c * c for c in force))
        away = tuple(self.ft_sign * c / fmag for c in force) if fmag > 1e-6 else (0.0, 0.0, 0.0)
        backoff = self.contact_backoff if fmag > 1e-6 else 0.0
        if self.mode == 'twist':
            self.adm.model.reset()  # don't carry admittance motion through the stop
            self.backoff_left, self.backoff_dir = backoff, away
            if self.controllers_ready:
                self.publish_twist((0.0, 0.0, 0.0))
            return
        self.stop_robot()
        tcp = self.lookup_tcp_in_base_link()
        if tcp is None:
            return
        self.last_target = tuple(tcp[i] + away[i] * backoff for i in range(3))
        if backoff > 0.0 and self.joint_state is not None and self.ik_client.service_is_ready():
            target = self.make_target_pose(self.last_target, tcp[3:])
            self.request_ik(target, retreat=max(0.15, backoff / BACKOFF_SPEED))

    # --- Twist mode ---------------------------------------------------------------------------

    def ensure_twist_controllers(self):
        """Keep twist_controller and force_mode_controller active (the driver loads them inactive)."""
        if self.cm_busy or not self.cm_list.service_is_ready():
            return
        self.cm_busy = True
        self.cm_list.call_async(ListControllers.Request()).add_done_callback(self.on_controller_list)

    def on_controller_list(self, future):
        try:
            states = {c.name: c.state for c in future.result().controller}
        except Exception:
            self.cm_busy = False
            return
        if any(n not in states for n in TWIST_CONTROLLERS):  # driver's spawner hasn't loaded them yet
            self.cm_busy = False
            return
        activate = [n for n in TWIST_CONTROLLERS if states[n] != 'active']
        if not activate:
            if not self.controllers_ready:
                self.get_logger().info('twist_controller and force_mode_controller active')
            self.controllers_ready = True
            self.cm_busy = False
            return
        self.controllers_ready = False
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = [n for n in MOTION_CONTROLLERS if states.get(n) == 'active']
        req.strictness = SwitchController.Request.STRICT
        self.cm_switch.call_async(req).add_done_callback(self.on_controller_switch)

    def on_controller_switch(self, future):
        self.cm_busy = False
        try:
            ok = future.result().ok
        except Exception:
            ok = False
        if not ok:
            self.get_logger().warn('Could not activate twist_controller/force_mode_controller, retrying',
                                   throttle_duration_sec=5.0)

    def publish_twist(self, v):
        """Publish linear velocity v (base_link, m/s) to twist_controller, which wants frame 'base'."""
        if self.base_rot is None:
            try:
                self.base_rot = self.tf_buffer.lookup_transform('base', 'base_link', Time()).transform.rotation
            except Exception:
                v = (0.0, 0.0, 0.0)
        vb = rotate(self.base_rot, v) if self.base_rot is not None else (0.0, 0.0, 0.0)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vb
        self.twist_pub.publish(msg)

    def on_twist_timer(self):
        if self.insert_busy and (self.get_clock().now() - self.insert_busy_since).nanoseconds * 1e-9 \
                > INSERT_SERVICE_TIMEOUT:
            # Give up on the call and make sure force mode is off, in case a late start went through
            self.insert_token += 1
            self.insert_busy = False
            self.insert = False
            self.fm_stop.call_async(Trigger.Request())
            self.get_logger().warn(f'Insert: no answer from the robot within {INSERT_SERVICE_TIMEOUT:.0f} s, '
                                   'force mode stopped')
        if self.insert_requested:
            self.insert_requested = False
            self.toggle_insert()
        if not self.controllers_ready:
            return  # twist_controller would reject (and log) every message until it is active
        if self.falcon_pos is None:
            self.publish_twist((0.0, 0.0, 0.0))
            return
        if self.clutch and not self.prev_clutch:
            self.center_falcon = self.falcon_pos
            self.ft_bias = None  # re-zero the wrist sensor on the next reading
            note = ''
            if self.insert:
                note = (' (Insert, force mode: only Z follows the grip)' if self.insert_method == 'force_mode'
                        else ' (Insert, admittance on)')
            self.get_logger().info('Clutch engaged' + note)
        self.prev_clutch = self.clutch
        if not self.clutch:
            self.motion_lockout = False

        now = self.get_clock().now()
        dt = 1.0 / TWIST_RATE
        if self.last_twist_time is not None:
            # Measured tick, bounded so a stalled executor can't make one huge RK4 step
            dt = min(max((now - self.last_twist_time).nanoseconds * 1e-9, 0.2 / TWIST_RATE),
                     3.0 / TWIST_RATE)
        self.last_twist_time = now
        v_adm = self.admittance_tick(dt) if self.insert and self.insert_method == 'admittance' \
            else (0.0, 0.0, 0.0)
        self.clutch_pub.publish(Bool(data=self.clutch))
        self.tripped_pub.publish(Bool(data=self.contact.tripped))

        v = (0.0, 0.0, 0.0)
        if self.contact.tripped and not self.contact.recovering:
            # Contact limit: the Falcon is ignored; only the optional backoff moves the arm
            if self.backoff_left > 0.0:
                step = min(BACKOFF_SPEED * dt, self.backoff_left)
                self.backoff_left -= step
                v = tuple(c * step / dt for c in self.backoff_dir)
        elif self.clutch and self.center_falcon is not None and not self.motion_lockout:
            v = self.rate_velocity()
            if self.insert and self.insert_method == 'force_mode':
                v = (0.0, 0.0, v[2])  # X/Y belong to force mode while inserting
            v = tuple(v[i] + v_adm[i] for i in range(3))
            if self.contact.recovering:
                v = away_only(v, self.contact_away())  # still over the limit: only away
            speed = math.sqrt(sum(c * c for c in v))
            if speed > self.max_tcp_speed:
                v = tuple(c * self.max_tcp_speed / speed for c in v)
            if self.program_running is False:
                self.get_logger().warn('The External Control program is not running on the pendant '
                                       '(press Play)', throttle_duration_sec=3.0)
        self.publish_twist(v)

    def admittance_params(self):
        return AdmittanceParams(
            mass=self.adm_mass, damping=self.adm_damping, stiffness=self.adm_stiffness,
            filter_hz=self.adm_filter_hz, deadband=self.adm_deadband,
            max_displacement=self.adm_max_displacement, max_speed=self.adm_max_speed,
            sign=self.ft_sign)

    def admittance_tick(self, dt):
        """Step the admittance from the wrist force; return its velocity (base_link, m/s)."""
        f = self.ft_force_in_base_link()
        if f is None:
            self.stop_admittance('the wrist force sensor stopped reporting')
            return (0.0, 0.0, 0.0)
        was_zeroed = self.adm.zeroed
        try:
            v = self.adm.update(f, self.clutch and not self.contact.tripped, dt,
                                self.admittance_params())
        except AdmittanceStop as e:
            self.stop_admittance(str(e))
            return (0.0, 0.0, 0.0)
        if self.adm.zeroed and not was_zeroed:
            self.get_logger().info(
                f'Insert ON (admittance): M={self.adm_mass} D={self.adm_damping} '
                f'K={self.adm_stiffness}. Tap minus again to leave.')
        return v

    def stop_admittance(self, reason):
        self.insert = False
        self.insert_method = None
        self.adm.reset()
        # Stop the arm too: without the admittance, continuing to push would press stiffly
        # into whatever caused the stop
        self.motion_lockout = True
        self.get_logger().error(f'Insert stopped: {reason}. The arm is stopped: release the '
                                'clutch to continue, tap minus to start Insert again.')

    def start_admittance(self):
        if self.clutch:
            self.get_logger().warn('Insert: release the clutch first (the sensor is zeroed with the '
                                   'arm still and out of contact)')
            return
        self.adm.reset()
        self.insert_method = 'admittance'
        self.insert = True
        self.get_logger().info('Insert: zeroing the wrist sensor, keep still...')

    def toggle_insert(self):
        if self.insert_busy:
            return
        if self.insert and self.insert_method == 'admittance':
            self.insert = False
            self.insert_method = None
            self.adm.reset()
            self.get_logger().info('Insert OFF: normal joystick control')
            return
        if not self.insert and self.get_parameter('insert_method').value == 'admittance':
            self.start_admittance()
            return
        if not self.controllers_ready:
            self.get_logger().warn('Insert: force_mode_controller is not active yet')
            return
        if self.insert:
            if not self.fm_stop.service_is_ready():
                self.get_logger().warn('Insert: stop_force_mode service not available')
                return
            token = self.start_insert_call()
            self.fm_stop.call_async(Trigger.Request()).add_done_callback(
                lambda f: self.on_insert_result(f, False, token))
            return
        if not (self.zero_ft_client.service_is_ready() and self.fm_start.service_is_ready()):
            self.get_logger().warn('Insert: zero_ftsensor/start_force_mode services not available')
            return
        req = self.force_mode_request()
        if req is None:
            return
        token = self.start_insert_call()
        # Zero the sensor first: with a 0 N target, any offset would make the arm drift
        self.zero_ft_client.call_async(Trigger.Request()).add_done_callback(
            lambda f: self.on_ft_zeroed(f, req, token))

    def start_insert_call(self):
        self.insert_token += 1
        self.insert_busy = True
        self.insert_busy_since = self.get_clock().now()
        return self.insert_token

    def on_ft_zeroed(self, future, req, token):
        if token != self.insert_token:
            return  # timed out meanwhile
        try:
            ok = future.result().success
        except Exception:
            ok = False
        if not ok:
            self.insert_busy = False
            self.get_logger().warn('Insert: zeroing the force sensor failed, force mode not started')
            return
        self.ft_bias = None
        self.fm_start.call_async(req).add_done_callback(lambda f: self.on_insert_result(f, True, token))

    def on_insert_result(self, future, on, token):
        if token != self.insert_token:
            return  # timed out meanwhile
        self.insert_busy = False
        try:
            ok = future.result().success
        except Exception:
            ok = False
        if not ok:
            self.get_logger().warn('Insert: could not ' + ('start' if on else 'stop') + ' force mode')
            return
        self.insert = on
        self.insert_method = 'force_mode' if on else None
        if on:
            self.get_logger().info(f"Insert ON: compliant in {self.get_parameter('insert_axes').value}, "
                                   'grip drives Z only. Tap minus again to leave.')
        else:
            self.get_logger().info('Insert OFF: normal joystick control')

    def force_mode_request(self):
        """SetForceMode request with the current TCP as task frame, or None if not possible."""
        valid = ('x', 'y', 'z', 'rx', 'ry', 'rz')
        axes = {a.strip() for a in self.get_parameter('insert_axes').value.split(',') if a.strip()}
        if not axes or not axes <= set(valid):
            self.get_logger().warn(f"Insert: insert_axes must be a comma list of {', '.join(valid)}")
            return None
        t = None
        for frame in ('tool0_controller', 'tool0'):  # tool0_controller = the pendant's TCP
            try:
                t = self.tf_buffer.lookup_transform('base', frame, Time()).transform
            except Exception:
                continue
            if frame == 'tool0':
                self.get_logger().warn('Insert: no tool0_controller TF, using the flange (tool0) as '
                                       'task frame instead of the pendant TCP')
            break
        if t is None:
            self.get_logger().warn('Insert: no TF base->TCP, force mode not started')
            return None
        req = SetForceMode.Request()
        req.task_frame.header.frame_id = 'base'
        req.task_frame.pose.position.x = t.translation.x
        req.task_frame.pose.position.y = t.translation.y
        req.task_frame.pose.position.z = t.translation.z
        req.task_frame.pose.orientation = t.rotation
        for a in valid:
            setattr(req, f'selection_vector_{a}', a in axes)
        req.type = SetForceMode.Request.NO_TRANSFORM
        lin = float(self.get_parameter('insert_speed_limit').value)
        rot = float(self.get_parameter('insert_rot_speed_limit').value)
        req.speed_limits.linear.x = req.speed_limits.linear.y = req.speed_limits.linear.z = lin
        req.speed_limits.angular.x = req.speed_limits.angular.y = req.speed_limits.angular.z = rot
        req.damping_factor = float(self.get_parameter('insert_damping').value)
        req.gain_scaling = float(self.get_parameter('insert_gain_scaling').value)
        return req

    # --- Rate / position modes (IK + trajectory goals) ------------------------------------------

    def on_timer(self):
        if self.mode == 'twist':
            return  # on_twist_timer handles twist mode
        if self.falcon_pos is None or self.joint_state is None:
            return

        if self.clutch and not self.prev_clutch:
            tcp = self.lookup_tcp_in_base_link()
            if tcp is None:
                # Leave prev_clutch False so engaging is retried next tick
                self.get_logger().warn('No TF base_link->tool0 yet, cannot engage clutch.',
                                       throttle_duration_sec=1.0)
                return
            self.center_falcon = self.falcon_pos
            self.center_tcp = tcp
            self.last_target = tcp[:3]
            self.prev_boost = self.boost
            # Re-zero the wrist sensor on the next valid reading: removes its offset and the tool's
            # weight (orientation stays fixed while engaged). Don't press the clutch while touching
            # something.
            self.ft_bias = None
            self.get_logger().info(f'Clutch engaged at TCP (base_link) {self.center_tcp[:3]}')
        elif (self.mode == 'position' and self.clutch and self.boost != self.prev_boost
              and self.last_target is not None):
            # Scale changed mid-move: re-anchor at the current hand and target
            # positions so the robot doesn't jump when the scale switches.
            self.center_falcon = self.falcon_pos
            self.center_tcp = self.last_target + self.center_tcp[3:]
            self.prev_boost = self.boost
            self.get_logger().info('Boost ' + ('on' if self.boost else 'off'))
        if self.prev_clutch and not self.clutch:
            self.stop_robot()
        self.prev_clutch = self.clutch
        self.clutch_pub.publish(Bool(data=self.clutch))
        self.tripped_pub.publish(Bool(data=self.contact.tripped))

        if (not self.clutch or self.center_falcon is None
                or (self.contact.tripped and not self.contact.recovering)):
            return
        if not self.ik_client.service_is_ready() or not self.traj_client.server_is_ready():
            return

        if self.mode == 'rate':
            v = self.rate_velocity()
            dt = self.timer.timer_period_ns * 1e-9
            tx, ty, tz = (self.last_target[i] + v[i] * dt for i in range(3))
        else:
            tx, ty, tz = self.compute_target()
        if self.contact.recovering:
            # Still over the contact limit: only the part of the move away from the contact
            d = away_only((tx - self.last_target[0], ty - self.last_target[1],
                           tz - self.last_target[2]), self.contact_away())
            tx, ty, tz = (self.last_target[i] + d[i] for i in range(3))
        # Rate-limit the commanded TCP toward the hand's target
        lx, ly, lz = self.last_target
        step = math.sqrt((tx - lx) ** 2 + (ty - ly) ** 2 + (tz - lz) ** 2)
        max_step = self.max_tcp_speed * self.timer.timer_period_ns * 1e-9
        if step > max_step:
            s = max_step / step
            tx, ty, tz = lx + (tx - lx) * s, ly + (ty - ly) * s, lz + (tz - lz) * s
            self.get_logger().info('TCP speed capped', throttle_duration_sec=2.0)
        tcp = self.lookup_tcp_in_base_link()
        if tcp is not None:
            ax, ay, az = tcp[:3]
            lead = math.sqrt((tx - ax) ** 2 + (ty - ay) ** 2 + (tz - az) ** 2)
            if lead > self.max_lead:
                s = self.max_lead / lead
                tx, ty, tz = ax + (tx - ax) * s, ay + (ty - ay) * s, az + (tz - az) * s
                self.warn_if_stalled()
            else:
                self.stall_since = None
        target = self.make_target_pose((tx, ty, tz), self.center_tcp[3:])
        self.last_target = (tx, ty, tz)
        self.target_pub.publish(target)
        self.request_ik(target)

    def make_target_pose(self, p, q):
        target = PoseStamped()
        target.header.frame_id = 'base_link'
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = p
        (target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z,
         target.pose.orientation.w) = q
        return target

    def request_ik(self, target, retreat=None):
        """IK for target, then a trajectory goal. retreat: the contact-limit backoff, sent
        despite the latch, taking this many seconds."""
        req = GetPositionIK.Request()
        req.ik_request = PositionIKRequest()
        req.ik_request.group_name = 'ur_manipulator'
        req.ik_request.robot_state.joint_state = self.joint_state
        req.ik_request.pose_stamped = target
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = 30_000_000
        req.ik_request.avoid_collisions = False

        future = self.ik_client.call_async(req)
        future.add_done_callback(lambda f: self.on_ik_response(f, retreat))

    def warn_if_stalled(self):
        now = self.get_clock().now()
        if self.stall_since is None:
            self.stall_since = now
            return
        if (now - self.stall_since).nanoseconds * 1e-9 < STALL_WARN_TIME:
            return
        reasons = []
        if self.program_running is False:
            reasons.append('the External Control program is not running on the pendant (press Play; '
                           'if it stops with an error, check its Host IP and that the firewall allows '
                           'TCP 50001-50004 from the robot)')
        if self.speed_scaling is not None and self.speed_scaling < 1.0:
            reasons.append(f'speed scaling is {self.speed_scaling:.0f}% '
                           '(pendant speed slider at 0, or the program is paused)')
        if not reasons:
            reasons.append('check the ur_robot_driver terminal for errors')
        self.get_logger().warn('Robot is not following commands: ' + '; '.join(reasons),
                               throttle_duration_sec=3.0)

    def ik_hint(self):
        """Likely reason IK fails from the current joint state (real arm has no move_to_ready)."""
        if self.joint_state is None:
            return ''
        q = dict(zip(self.joint_state.name, self.joint_state.position))
        deg = {j: math.degrees(v) for j, v in q.items()}
        hints = []
        if abs(q.get('elbow_joint', 1.0)) > math.pi:
            hints.append(f"elbow_joint is {deg['elbow_joint']:.0f} deg, outside MoveIt's +-180 deg limit")
        elif abs(q.get('elbow_joint', 1.0)) < SINGULAR_MARGIN:
            hints.append('the arm is almost straight (elbow singularity): bend the elbow')
        if abs(math.sin(q.get('wrist_2_joint', 1.0))) < math.sin(SINGULAR_MARGIN):
            hints.append(f"wrist_2_joint is {deg['wrist_2_joint']:.0f} deg (wrist singularity): "
                         'rotate it towards +-90 deg')
        for j in JOINT_ORDER[:5]:
            if j != 'elbow_joint' and abs(q.get(j, 0.0)) > 2 * math.pi:
                hints.append(f'{j} is {deg[j]:.0f} deg, outside MoveIt\'s +-360 deg limit')
        tcp = self.lookup_tcp_in_base_link()
        if tcp is not None and math.hypot(tcp[0], tcp[1]) < 0.1:
            hints.append('the tool is right above/below the base (shoulder singularity): move it outwards')
        if not hints:
            return 'The target may be out of reach; release the clutch and re-engage closer in.'
        return 'Jog the arm on the pendant first: ' + '; '.join(hints) + '.'

    def on_goal_response(self, future, retreat=None):
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().warn(f'Trajectory goal failed: {e}', throttle_duration_sec=2.0)
            return
        if not handle.accepted:
            self.get_logger().warn('Trajectory goal rejected by scaled_joint_trajectory_controller',
                                   throttle_duration_sec=2.0)
            return
        self.goal_handle = handle
        # Released, or the contact limit tripped, while this goal was in flight
        if retreat is None and (not self.clutch or
                                (self.contact.tripped and not self.contact.recovering)):
            self.stop_robot()

    def on_ik_response(self, future, retreat=None):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().warn(f'IK call failed: {e}')
            return
        # Released, or the contact limit tripped, while IK was running: don't move
        if retreat is None and (not self.clutch or
                                (self.contact.tripped and not self.contact.recovering)):
            return
        if res.error_code.val != 1:  # moveit_msgs/MoveItErrorCodes.SUCCESS
            code = res.error_code.val
            self.get_logger().warn(f'IK failed: {IK_ERRORS.get(code, code)}. {self.ik_hint()}',
                                   throttle_duration_sec=1.0)
            return
        name_to_pos = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        try:
            ordered = [name_to_pos[j] for j in JOINT_ORDER]
        except KeyError:
            return

        if self.joint_state is not None:
            current = dict(zip(self.joint_state.name, self.joint_state.position))
            try:
                jump = max(abs(ordered[i] - current[JOINT_ORDER[i]]) for i in range(6))
            except KeyError:
                jump = 0.0
            if jump > MAX_JOINT_JUMP:
                self.get_logger().warn(
                    f'Rejected IK solution: joint jump {jump:.3f} rad > {MAX_JOINT_JUMP}',
                    throttle_duration_sec=1.0)
                return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = JOINT_ORDER
        point = JointTrajectoryPoint()
        point.positions = ordered
        # 0.15 s overlaps the next 20 Hz tick; a backoff takes longer to stay slow
        dur = 0.15 if retreat is None else retreat
        point.time_from_start = Duration(sec=int(dur), nanosec=int((dur % 1.0) * 1e9))
        goal.trajectory.points = [point]
        # New goals preempt in-flight ones on this controller; only acceptance is checked.
        self.traj_client.send_goal_async(goal).add_done_callback(
            lambda f: self.on_goal_response(f, retreat))


def main():
    rclpy.init()
    node = FalconURTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Ctrl+C can invalidate the context while spin() is waiting (more likely with the TF
        # listener's own thread), which surfaces as an RCLError; only real errors propagate
        if rclpy.ok():
            raise
    finally:
        # Best effort only: the context can be shut down between the ok() check and the publish
        # (a signal during shutdown). Nothing depends on these: falcon_node zeroes stale forces by
        # itself, and twist_controller stops the arm 0.1 s after the last twist.
        if rclpy.ok():
            try:
                node.force_pub.publish(Vector3())
                if node.mode == 'twist':
                    node.publish_twist((0.0, 0.0, 0.0))
                    if node.insert:
                        node.fm_stop.call_async(Trigger.Request())
            except Exception:
                pass
        node.close_log()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
