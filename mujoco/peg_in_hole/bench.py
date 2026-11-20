"""Real-time factor while the peg slides in the socket, for collision/timestep options."""
import time
import mujoco
import numpy as np
import scene
import sim

for slabs, ts, coarse in ((False, None, False), (False, 0.002, False), (False, 0.002, True), (True, 0.002, True)):
    s = sim.Sim((0.010, 0.006))
    if slabs or ts or coarse:
        s.m = scene.load(hole_pos=tuple(s.hole_bottom), peg_slabs=slabs, timestep=ts, coarse_tiles=coarse)
        s.d, s.ref = mujoco.MjData(s.m), mujoco.MjData(s.m)
        s.M = np.zeros((s.m.nv, s.m.nv))
    p = s.hole_center + [0.0, 0.0, 0.060]     # tip 60 mm up, already on the taper
    q, _ = sim.ik(s.m, p, sim.R_TOOL_DOWN, s.q_ref)
    s.d.qpos[:] = s.q_ref = q
    mujoco.mj_forward(s.m, s.d)
    n = round(2.0 / s.m.opt.timestep)
    t0 = time.time()
    for _ in range(n):
        s.step(np.array([0, 0, -0.015]))
    wall = time.time() - t0
    tip = s.tip_rel_hole()
    print(f'coarse={coarse!s:5} slabs={slabs!s:5} timestep={s.m.opt.timestep*1000:.0f} ms: 2 s in contact took {wall:5.2f} s '
          f'-> {2.0 / wall:4.2f}x real time; ncon={s.d.ncon}, tip {tip[2]*1000:.1f} mm up, '
          f'{np.hypot(tip[0], tip[1])*1000:.1f} mm off-centre')
