"""Peg-in-hole insertion on a simulated UR3e, using the bridge's own admittance code.

Mirrors the real twist-mode chain:
  robot   speedl-like velocity input -> joint reference (integrated, like the UR controller)
          -> stiff joint servo (so a blocked peg builds force, as on the real arm)
  bridge  every CONTROL_DT (twist rate): wrist force -> AdmittanceInsert (shared with
          falcon_ur_teleop.py) -> v = v_operator + v_admittance -> speed cap -> robot

The "operator" is a scripted Falcon: clutch held, pushing straight down at --speed. The hole is
offset sideways by --offset, so the peg meets the taper off-centre and has to be guided in.

    ../.venv/bin/python sim.py                       # admittance on, defaults
    ../.venv/bin/python sim.py --no-admittance       # same run, stiff (for comparison)
    ../.venv/bin/python sim.py --viewer              # watch it live in the MuJoCo viewer
    ../.venv/bin/python sim.py --video               # also save an MP4 to the output folder
    ../.venv/bin/python sim.py --damping 1500 1500 1000 --stiffness 400 400 0 --mass 60 60 60
"""
import argparse
from datetime import datetime
from pathlib import Path
import sys
import time

import matplotlib
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

import scene

# Camera on the socket's long side, looking through its side window (MuJoCo free camera)
CAMERA = dict(distance=0.80, azimuth=105.0, elevation=-12.0, lookat_up=0.12)
VIDEO_FPS = 30
VIDEO_SIZE = (960, 544)  # multiple of 8 for the video encoder

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src' / 'falcon_ros2' / 'scripts'))
from admittance_insert import AdmittanceInsert, AdmittanceParams, AdmittanceStop  # noqa: E402

CONTROL_DT = 0.01          # s, bridge twist rate (100 Hz)
MAX_TCP_SPEED = 0.25       # m/s, bridge speed cap
KP, KD = 900.0, 60.0       # joint servo (computed torque): ~30 rad/s, critically damped
SETTLE_TIME = 1.0          # s before the clutch is pressed (the admittance zeroes here)
START_GAP = 0.02           # m, tip above the socket's top rim at the start
HOLE_TOP = 0.1517          # m, socket height
SEAT_HEIGHT = 0.0209       # m, tip above the socket bottom when seated (seat_depth.py --tiles)
JOINT_NAMES = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',  # qpos order
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
R_TOOL_DOWN = np.diag([1.0, -1.0, -1.0])  # tool X = world X (slot aligned), tool Z = down


def site_pose(m, d, site):
    return d.site(site).xpos.copy(), d.site(site).xmat.reshape(3, 3).copy()


def jac6(m, d, site_id):
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    mujoco.mj_jacSite(m, d, jp, jr, site_id)
    return np.vstack([jp, jr])


def rot_error(R_des, R):
    """Rotation vector taking R to R_des (world frame), via MuJoCo's fast quaternion helpers."""
    q_des, q, q_err = np.zeros(4), np.zeros(4), np.zeros(4)
    mujoco.mju_mat2Quat(q_des, R_des.flatten())
    mujoco.mju_mat2Quat(q, R.flatten())
    q_inv = np.zeros(4)
    mujoco.mju_negQuat(q_inv, q)
    mujoco.mju_mulQuat(q_err, q_des, q_inv)
    w = np.zeros(3)
    mujoco.mju_quat2Vel(w, q_err, 1.0)
    return w


def dls(J, twist, lam=0.01):
    return J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), twist)


def ik(m, p_des, R_des, q0):
    d = mujoco.MjData(m)
    d.qpos[:] = q0
    sid = m.site('peg_tip').id
    for _ in range(500):
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        p, R = site_pose(m, d, 'peg_tip')
        err = np.r_[p_des - p, Rotation.from_matrix(R_des @ R.T).as_rotvec()]
        if np.linalg.norm(err) < 1e-7:
            break
        d.qpos[:] += dls(jac6(m, d, sid), err, 0.05)
    return d.qpos.copy(), np.linalg.norm(err)


class Sim:
    def __init__(self, hole_offset, viewer=False, see_through=False, fast=False):
        self.hole_center = np.array([0.30, 0.0, -0.15])
        hole_pos = self.hole_center + [hole_offset[0], hole_offset[1], 0.0]
        self.m = scene.load(hole_pos=tuple(hole_pos), hole_alpha=0.45 if see_through else 1.0,
                            peg_slabs=fast, timestep=0.002 if fast else None, coarse_tiles=fast)
        self.d = mujoco.MjData(self.m)
        self.ref = mujoco.MjData(self.m)  # kinematics of the integrated joint reference
        self.hole_bottom = hole_pos
        self.tip_id = self.m.site('peg_tip').id
        start = self.hole_center + [0, 0, HOLE_TOP + START_GAP]
        for guess in ([0, -1.57, 1.57, -1.57, -1.57, 0], [0, -1.2, 1.8, -2.2, -1.57, 0],
                      [np.pi, -1.57, 1.57, -1.57, -1.57, 0]):
            q, err = ik(self.m, start, R_TOOL_DOWN, guess)
            if err < 1e-5:
                break
        else:
            raise RuntimeError(f'No IK solution for the start pose (error {err:.2e})')
        self.d.qpos[:] = q
        self.q_ref = q.copy()
        self.qd_ref = np.zeros(self.m.nv)
        mujoco.mj_forward(self.m, self.d)
        self.M = np.zeros((self.m.nv, self.m.nv))
        nv = self.m.nv
        self._buf = {'q': np.zeros(4), 'q_inv': np.zeros(4), 'q_err': np.zeros(4), 'w': np.zeros(3),
                     'twist': np.zeros(6), 'jac': np.zeros((6, nv)), 'tau': np.zeros(nv)}
        self._q_down = np.zeros(4)
        mujoco.mju_mat2Quat(self._q_down, R_TOOL_DOWN.flatten())
        self._lam2 = 0.01 ** 2 * np.eye(6)
        self.lo, self.hi = self.m.actuator_ctrlrange[:, 0], self.m.actuator_ctrlrange[:, 1]
        self.sensor_sign = None
        self.viewer = None
        if viewer:
            self.viewer = mujoco.viewer.launch_passive(self.m, self.d)
            with self.viewer.lock():
                set_camera(self.viewer.cam, hole_pos)
                self.viewer.opt.geomgroup[3] = 0  # hide the collision shapes

    def step(self, v_lin):
        """One physics step with the linear TCP velocity command v_lin (world = base_link).

        Runs every physics step, so it reuses preallocated buffers instead of building small
        numpy arrays (that overhead decided whether the live simulation kept up with real time).
        """
        m, d, ref, b = self.m, self.d, self.ref, self._buf
        ref.qpos[:] = self.q_ref
        mujoco.mj_kinematics(m, ref)
        mujoco.mj_comPos(m, ref)
        # Orientation hold: rotation vector from the reference's tool rotation to R_TOOL_DOWN
        mujoco.mju_mat2Quat(b['q'], ref.site_xmat[self.tip_id])
        mujoco.mju_negQuat(b['q_inv'], b['q'])
        mujoco.mju_mulQuat(b['q_err'], self._q_down, b['q_inv'])
        mujoco.mju_quat2Vel(b['w'], b['q_err'], 1.0)
        b['twist'][:3] = v_lin
        b['twist'][3:] = 2.0 * b['w']
        J = b['jac']
        mujoco.mj_jacSite(m, ref, J[:3], J[3:], self.tip_id)
        self.qd_ref = J.T @ np.linalg.solve(J @ J.T + self._lam2, b['twist'])  # damped LSQ
        self.q_ref = self.q_ref + self.qd_ref * m.opt.timestep
        acc = KP * (self.q_ref - d.qpos) + KD * (self.qd_ref - d.qvel)
        mujoco.mj_mulM(m, d, b['tau'], acc)  # M(q) @ acc without building M
        np.clip(b['tau'] + d.qfrc_bias, self.lo, self.hi, out=d.ctrl)
        mujoco.mj_step(m, d)

    def wrist_force(self):
        """Force the environment applies on the tool, in world/base_link, plus a constant offset
        (the peg's weight) that the admittance zeroes away, like the UR's TCP force."""
        _, R = site_pose(self.m, self.d, 'ft_sensor')
        f = R @ self.d.sensor('ft_force').data
        return self.sensor_sign * f

    def calibrate_sign(self):
        """At rest the sensor carries the peg's weight; the environment-on-tool reading must point
        down. Then confirm with a known sideways push on the peg."""
        for _ in range(500):
            self.step(np.zeros(3))
        _, R = site_pose(self.m, self.d, 'ft_sensor')
        raw = R @ self.d.sensor('ft_force').data
        self.sensor_sign = -1.0 if raw[2] > 0 else 1.0
        rest = self.wrist_force()
        peg = self.m.body('peg').id
        self.d.xfrc_applied[peg, :3] = [5.0, 0.0, 0.0]
        for _ in range(300):
            self.step(np.zeros(3))
        pushed = self.wrist_force()
        self.d.xfrc_applied[peg, :3] = 0.0
        for _ in range(300):
            self.step(np.zeros(3))
        delta = pushed - rest
        ok = abs(rest[2] + scene.PEG_MASS * 9.81) < 0.3 and abs(delta[0] - 5.0) < 0.5
        print(f'sensor check: at rest {np.round(rest, 2)} N (weight {scene.PEG_MASS * 9.81:.2f} N), '
              f'+5 N push along X reads {np.round(delta, 2)} N -> {"OK" if ok else "WRONG"}')
        if not ok:
            raise RuntimeError('Force sensor sign/frame check failed')

    def tip_rel_hole(self):
        return self.d.site('peg_tip').xpos - self.hole_bottom


def set_camera(cam, hole_pos):
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [hole_pos[0], hole_pos[1], hole_pos[2] + CAMERA['lookat_up']]
    cam.distance, cam.azimuth, cam.elevation = (CAMERA['distance'], CAMERA['azimuth'],
                                                CAMERA['elevation'])


class Recorder:
    """Offscreen frames -> MP4, with a text overlay of the run's state."""

    def __init__(self, sim, path, label):
        import imageio
        from PIL import ImageFont
        w, h = VIDEO_SIZE
        self.renderer = mujoco.Renderer(sim.m, h, w)
        self.cam = mujoco.MjvCamera()
        set_camera(self.cam, sim.hole_bottom)
        self.opt = mujoco.MjvOption()
        self.opt.geomgroup[3] = 0
        self.writer = imageio.get_writer(path, fps=VIDEO_FPS, codec='libx264', quality=7,
                                         macro_block_size=8)
        self.font = ImageFont.load_default(size=17)
        self.label = label
        self.path = path

    def frame(self, sim, t, contact, insert_on, stopped):
        from PIL import Image, ImageDraw
        self.renderer.update_scene(sim.d, camera=self.cam, scene_option=self.opt)
        img = Image.fromarray(self.renderer.render())
        tip = sim.tip_rel_hole()
        fmag = float(np.linalg.norm(np.nan_to_num(contact)))
        lines = [self.label,
                 f't = {t:5.2f} s',
                 f'tip above hole bottom: {tip[2] * 1000:6.1f} mm (seated {SEAT_HEIGHT * 1000:.1f})',
                 f'off-centre: {np.hypot(tip[0], tip[1]) * 1000:5.1f} mm',
                 f'contact force: {fmag:6.1f} N',
                 ('STOPPED at ' + stopped.replace(' (', '\n(') if stopped else
                  ('admittance ON' if insert_on else ''))]
        lines = [x for line in lines for x in line.split('\n')]
        draw = ImageDraw.Draw(img)
        x0 = VIDEO_SIZE[0] - 470  # top-right corner (empty floor in this view)
        draw.rectangle([x0 - 8, 8, VIDEO_SIZE[0] - 8, 16 + 22 * len(lines)], fill=(0, 0, 0))
        for i, line in enumerate(lines):
            draw.text((x0, 12 + 22 * i), line, font=self.font,
                      fill=(255, 120, 90) if line.startswith('STOPPED') else (235, 235, 235))
        self.writer.append_data(np.asarray(img))

    def close(self):
        self.writer.close()
        self.renderer.close()


def run(args, out):
    sim = Sim(args.offset, viewer=args.viewer, see_through=args.viewer or args.video)
    label = 'no admittance (stiff robot)' if args.no_admittance else 'admittance'
    label += f', hole {args.offset[0]*1000:.0f}/{args.offset[1]*1000:.0f} mm off-centre'
    rec = Recorder(sim, str(out / 'insertion.mp4'), label) if args.video else None
    next_frame = 0.0
    sim.calibrate_sign()
    params = AdmittanceParams(mass=args.mass, damping=args.damping, stiffness=args.stiffness,
                              filter_hz=args.filter_hz, deadband=args.deadband,
                              max_displacement=args.max_displacement, max_speed=args.max_speed)
    adm = AdmittanceInsert()
    use_adm = not args.no_admittance
    n_sub = round(CONTROL_DT / sim.m.opt.timestep)
    rows, v_cmd, stopped = [], np.zeros(3), None
    t, wall0 = 0.0, time.time()
    while t < args.duration:
        f = sim.wrist_force()
        clutch = t >= SETTLE_TIME
        v_adm = (0.0, 0.0, 0.0)
        if use_adm and stopped is None:
            try:
                v_adm = adm.update(f, clutch, CONTROL_DT, params)
            except AdmittanceStop as e:
                stopped = f'{t:.2f} s: {e}'
                print('Insert stopped:', stopped)
        # Like the bridge: after an admittance stop the arm stops until the clutch is released
        # (the scripted operator never releases it)
        v_op = np.array([0.0, 0.0, -args.speed]) if clutch and stopped is None else np.zeros(3)
        v_cmd = v_op + np.array(v_adm)
        sp = np.linalg.norm(v_cmd)
        if sp > MAX_TCP_SPEED:
            v_cmd *= MAX_TCP_SPEED / sp
        contact = f - adm.bias if adm.bias is not None else np.full(3, np.nan)
        if not use_adm:  # still report the zeroed contact force for comparison
            if t < SETTLE_TIME:
                rest = f
            contact = f - rest
        rows.append([t, *sim.tip_rel_hole(), *contact, *v_adm, *v_cmd, *adm.model.x])
        if rec is not None and t >= next_frame - 1e-9:
            rec.frame(sim, t, contact, use_adm and adm.zeroed and stopped is None, stopped)
            next_frame += 1.0 / VIDEO_FPS
        for _ in range(n_sub):
            sim.step(v_cmd)
        t += CONTROL_DT
        if sim.viewer is not None:
            sim.viewer.sync()
            time.sleep(max(0.0, t - (time.time() - wall0)))
            if not sim.viewer.is_running():
                break
    if rec is not None:
        rec.close()
        print('video:', rec.path)
    return np.array(rows), stopped, time.time() - wall0


def report(rows, stopped, args, wall, out):
    cols = ('t tip_x tip_y tip_z f_x f_y f_z vadm_x vadm_y vadm_z vcmd_x vcmd_y vcmd_z '
            'adm_x adm_y adm_z').split()
    label = 'no_admittance' if args.no_admittance else 'admittance'
    np.savetxt(out / 'data.csv', rows, delimiter=',', header=','.join(cols), comments='')
    t, tip, f = rows[:, 0], rows[:, 1:4], rows[:, 4:7]
    fmag = np.linalg.norm(np.nan_to_num(f), axis=1)
    final = tip[-1]
    lateral = np.hypot(final[0], final[1])
    seated = final[2] < SEAT_HEIGHT + 0.003 and lateral < 0.002
    summary = (f'{label}: offset {args.offset[0]*1000:.0f}/{args.offset[1]*1000:.0f} mm, '
               f'push {args.speed*100:.1f} cm/s\n'
               f'  final tip: {lateral*1000:.1f} mm off-centre, {final[2]*1000:.1f} mm above the '
               f'bottom (seated = {SEAT_HEIGHT*1000:.1f}) -> {"SEATED" if seated else "NOT seated"}\n'
               f'  peak contact force {fmag.max():.1f} N, final {fmag[-1]:.1f} N'
               + (f'\n  admittance stopped at {stopped}' if stopped else '')
               + f'\n  {t[-1]:.1f} s simulated in {wall:.0f} s')
    print(summary)
    (out / 'summary.txt').write_text(summary + '\n')

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for i, n in enumerate('xyz'):
        ax[0].plot(t, tip[:, i] * 1000, label=f'tip {n}')
    ax[0].axhline(SEAT_HEIGHT * 1000, color='k', ls=':', lw=1, label='seated height')
    ax[0].set_ylabel('tip rel. hole bottom [mm]')
    for i, n in enumerate('xyz'):
        ax[1].plot(t, f[:, i], label=f'F{n}')
    ax[1].set_ylabel('contact force on tool [N]')
    for i, n in enumerate('xyz'):
        ax[2].plot(t, rows[:, 7 + i] * 1000, label=f'v_adm {n}')
    ax[2].set_ylabel('admittance velocity [mm/s]')
    ax[2].set_xlabel('time [s]')
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(loc='upper right', fontsize=8)
    fig.suptitle(summary.splitlines()[0])
    fig.tight_layout()
    fig.savefig(out / 'insertion.png', dpi=90)
    print('saved', out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--offset', type=float, nargs=2, default=[0.010, 0.006], metavar=('DX', 'DY'),
                    help='hole offset from the peg start, m')
    ap.add_argument('--speed', type=float, default=0.015, help='operator push down, m/s')
    ap.add_argument('--duration', type=float, default=14.0)
    ap.add_argument('--no-admittance', action='store_true')
    ap.add_argument('--viewer', action='store_true', help='watch it live in the MuJoCo viewer')
    ap.add_argument('--video', action='store_true', help='save insertion.mp4 to the output folder')
    d = AdmittanceParams()
    ap.add_argument('--mass', type=float, nargs=3, default=d.mass)
    ap.add_argument('--damping', type=float, nargs=3, default=d.damping)
    ap.add_argument('--stiffness', type=float, nargs=3, default=d.stiffness)
    ap.add_argument('--filter-hz', type=float, default=d.filter_hz)
    ap.add_argument('--deadband', type=float, default=d.deadband)
    ap.add_argument('--max-displacement', type=float, default=d.max_displacement)
    ap.add_argument('--max-speed', type=float, default=d.max_speed)
    args = ap.parse_args()
    if args.viewer:
        import mujoco.viewer  # noqa: F401
    label = 'no_admittance' if args.no_admittance else 'admittance'
    out = Path(__file__).parent / 'outputs' / f'{datetime.now():%Y%m%d-%H%M%S}_{label}'
    out.mkdir(parents=True)
    rows, stopped, wall = run(args, out)
    report(rows, stopped, args, wall, out)


if __name__ == '__main__':
    main()
