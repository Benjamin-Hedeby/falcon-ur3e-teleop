"""Admittance for twist-mode Insert, without ROS: shared by falcon_ur_teleop.py and the MuJoCo
peg-in-hole simulation (mujoco/peg_in_hole), so the simulation tests the code the robot runs.

Per translation axis (base_link X, Y, Z): m a + c v + k x = f, stepped with RK4. The input is the
wrist force, zeroed against the average of the first samples, low-pass filtered and passed
through a magnitude deadband. Exceeding the displacement/speed limits raises AdmittanceStop.
"""
from dataclasses import dataclass, field
import math

BIAS_SAMPLES = 20  # updates averaged to zero the sensor


class AdmittanceStop(Exception):
    """The admittance left its safe envelope; the caller must stop commanding it."""


@dataclass
class AdmittanceParams:
    mass: list = field(default_factory=lambda: [40.0, 40.0, 60.0])         # kg (virtual)
    damping: list = field(default_factory=lambda: [1000.0, 1000.0, 1000.0])  # N s/m; sideways from the 2026-11-24 sweep
    stiffness: list = field(default_factory=lambda: [400.0, 400.0, 0.0])     # N/m
    filter_hz: float = 10.0         # low-pass on the force input (0 = off)
    deadband: float = 1.0           # N, force magnitude ignored
    max_displacement: float = 0.03  # m
    max_speed: float = 0.05         # m/s
    sign: float = 1.0               # +1: the arm yields the way the environment pushes the tool


class Admittance:
    """Independent per-axis virtual mass-damper-spring, m a + c v + k x = f, stepped with RK4.

    x and v are the virtual displacement/velocity relative to the motion the Falcon commands;
    they are not measured robot states.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.x = [0.0, 0.0, 0.0]
        self.v = [0.0, 0.0, 0.0]

    def step(self, f, m, c, k, dt):
        for i in range(3):
            def acc(x, v):
                return (f[i] - c[i] * v - k[i] * x) / m[i]
            x, v = self.x[i], self.v[i]
            # RK4 on the state (x, v); force and parameters are held over the step
            a1 = acc(x, v)
            x2, v2 = x + 0.5 * dt * v, v + 0.5 * dt * a1
            a2 = acc(x2, v2)
            x3, v3 = x + 0.5 * dt * v2, v + 0.5 * dt * a2
            a3 = acc(x3, v3)
            x4, v4 = x + dt * v3, v + dt * a3
            a4 = acc(x4, v4)
            self.x[i] = x + dt * (v + 2 * v2 + 2 * v3 + v4) / 6.0
            self.v[i] = v + dt * (a1 + 2 * a2 + 2 * a3 + a4) / 6.0


class AdmittanceInsert:
    """Zeroing, filtering, deadband, the admittance model and its safety limits."""

    def __init__(self):
        self.model = Admittance()
        self.reset()

    def reset(self):
        """Start over: the next BIAS_SAMPLES updates re-zero the sensor."""
        self.model.reset()
        self.bias = None
        self.bias_buf = []
        self.force = [0.0, 0.0, 0.0]  # zeroed, filtered input (before the deadband)

    @property
    def zeroed(self):
        return self.bias is not None

    def update(self, f, clutch, dt, p):
        """Advance by dt with sensor force f (base_link, N); return the admittance velocity (m/s).

        While zeroing, and while the clutch is released (dead-man; the model re-anchors), the
        velocity is zero. Raises AdmittanceStop when a limit is exceeded.
        """
        if self.bias is None:
            # Zero against the average of the first samples (arm still, no contact)
            self.bias_buf.append(list(f))
            if len(self.bias_buf) >= BIAS_SAMPLES:
                n = len(self.bias_buf)
                self.bias = [sum(s[i] for s in self.bias_buf) / n for i in range(3)]
                self.force = [0.0, 0.0, 0.0]
            return (0.0, 0.0, 0.0)
        ext = [p.sign * (f[i] - self.bias[i]) for i in range(3)]
        a = 1.0 - math.exp(-2.0 * math.pi * p.filter_hz * dt) if p.filter_hz > 0 else 1.0
        self.force = [q + a * (e - q) for q, e in zip(self.force, ext)]
        if not clutch:
            self.model.reset()
            return (0.0, 0.0, 0.0)
        fin = self.force
        mag = math.sqrt(sum(c * c for c in fin))
        fin = [c * (mag - p.deadband) / mag for c in fin] if mag > p.deadband else [0.0, 0.0, 0.0]
        self.model.step(fin, p.mass, p.damping, p.stiffness, dt)
        disp = math.sqrt(sum(c * c for c in self.model.x))
        speed = math.sqrt(sum(c * c for c in self.model.v))
        if not (math.isfinite(disp) and math.isfinite(speed)):
            raise AdmittanceStop('the admittance state became non-finite')
        if disp > p.max_displacement:
            raise AdmittanceStop(f'admittance moved {disp * 100:.1f} cm '
                                 f'(adm_max_displacement {p.max_displacement * 100:.1f} cm)')
        if speed > p.max_speed:
            raise AdmittanceStop(f'admittance speed {speed * 100:.1f} cm/s '
                                 f'(adm_max_speed {p.max_speed * 100:.1f} cm/s)')
        return tuple(self.model.v)
