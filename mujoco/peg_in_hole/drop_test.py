"""Drop the free peg into the hole for several CoACD decompositions and compare where it rests.

The finest decomposition is the reference; a coarser one is good enough if the peg comes to rest
at about the same height and position.
"""
from pathlib import Path
import shutil
import warnings

import coacd
import mujoco
import numpy as np
import trimesh

warnings.filterwarnings('ignore')
HERE = Path(__file__).parent
MESHES = HERE / 'meshes'
VARIANTS = [(0.02, 2000), (0.04, 64), (0.05, 40), (0.08, 30)]  # (threshold, max hulls)


def decompose(threshold, max_hulls):
    d = HERE / f'_variants/t{threshold}_m{max_hulls}'
    if d.exists():
        return sorted(d.glob('*.obj'))
    d.mkdir(parents=True)
    hole = trimesh.load(MESHES / 'hole_visual.obj', force='mesh')
    parts = coacd.run_coacd(coacd.Mesh(hole.vertices, hole.faces), threshold=threshold,
                            max_convex_hull=max_hulls if max_hulls < 2000 else -1)
    for i, (v, f) in enumerate(parts):
        trimesh.Trimesh(v, f).export(d / f'p{i:03d}.obj')
    return sorted(d.glob('*.obj'))


def drop(pieces, start_offset=(0.0, 0.0)):
    assets = ''.join(f'<mesh name="h{i}" file="{p}"/>' for i, p in enumerate(pieces))
    geoms = ''.join(f'<geom type="mesh" mesh="h{i}"/>' for i in range(len(pieces)))
    xml = f"""
    <mujoco>
      <option timestep="0.001" gravity="0 0 -9.81"/>
      <asset>{assets}<mesh name="peg" file="{MESHES / 'peg_collision.obj'}"/></asset>
      <default><geom friction="0.3 0.005 0.0001" solref="0.004 1"/></default>
      <worldbody>
        {geoms}
        <body name="peg" pos="{start_offset[0]} {start_offset[1]} 0.40" quat="0 1 0 0">
          <freejoint/>
          <geom type="mesh" mesh="peg" mass="0.733"/>
        </body>
      </worldbody>
    </mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    for _ in range(3000):
        mujoco.mj_step(m, d)
    pos = d.body('peg').xpos.copy()
    tip = pos + d.body('peg').xmat.reshape(3, 3) @ np.array([0, 0, 0.2257])
    return tip, np.linalg.norm(d.qvel[:3])


if __name__ == '__main__':
    for thr, mh in VARIANTS:
        pieces = decompose(thr, mh)
        tip, speed = drop(pieces)
        tip_off, _ = drop(pieces, (0.015, 0.010))
        print(f'threshold {thr:4}, {len(pieces):3d} pieces: centred drop rests with tip at '
              f'({tip[0]*1000:5.1f}, {tip[1]*1000:5.1f}, {tip[2]*1000:6.1f}) mm, speed {speed:.3f} m/s; '
              f'dropped 15/10 mm off-centre: ({tip_off[0]*1000:5.1f}, {tip_off[1]*1000:5.1f}, '
              f'{tip_off[2]*1000:6.1f}) mm')
