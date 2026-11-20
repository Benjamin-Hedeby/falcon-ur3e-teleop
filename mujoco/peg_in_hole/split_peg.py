"""Split the peg's convex collision hull into horizontal slabs (each still convex).

When the peg is inside the socket, the whole hull's bounding box overlaps nearly every socket tile,
so MuJoCo runs a narrow-phase check against all of them each step. Slabs only overlap the tiles at
their own height, which cuts the number of checks. Writes meshes/peg_slab_*.obj.

    ../.venv/bin/python split_peg.py [n_slabs]
"""
from pathlib import Path
import sys
import warnings

import numpy as np
import trimesh

warnings.filterwarnings('ignore')
MESHES = Path(__file__).parent / 'meshes'


def split(n):
    hull = trimesh.load(MESHES / 'peg_collision.obj', force='mesh')
    z0, z1 = hull.bounds[0][2], hull.bounds[1][2]
    for old in MESHES.glob('peg_slab_*.obj'):
        old.unlink()
    edges = np.linspace(z0, z1, n + 1)
    vol = 0.0
    for i in range(n):
        # Hull of a convex body cut by two planes = hull of its vertices between them plus its
        # outlines on both planes
        lo, hi = edges[i], edges[i + 1]
        pts = [hull.vertices[(hull.vertices[:, 2] >= lo) & (hull.vertices[:, 2] <= hi)]]
        for z in (lo + 1e-6, hi - 1e-6):
            sec = hull.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
            if sec is not None:
                pts.append(sec.vertices)
        piece = trimesh.Trimesh(np.vstack(pts)).convex_hull
        piece.export(MESHES / f'peg_slab_{i:02d}.obj')
        vol += piece.volume
    print(f'{n} slabs, volume {vol * 1e6:.0f} cm3 (hull {hull.volume * 1e6:.0f} cm3)')


if __name__ == '__main__':
    split(int(sys.argv[1]) if len(sys.argv) > 1 else 8)
