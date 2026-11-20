"""Geometric seating depth: lowest tip height at which the (convex) peg envelope, lowered
straight down the hole's centre line, touches no part of the hole's actual surface.

Independent of the convex decomposition, so it checks how faithful the MuJoCo pieces are.
"""
from pathlib import Path
import warnings

import numpy as np
import trimesh

warnings.filterwarnings('ignore')
MESHES = Path(__file__).parent / 'meshes'
TOL = 0.0002   # m, a point must be this far inside the peg to count as touching
CHUNK = 20000  # points per batch (keeps the points x planes matrix small)

hole = trimesh.load(MESHES / 'hole_visual.obj', force='mesh')
peg = trimesh.load(MESHES / 'peg_collision.obj', force='mesh')
# Peg hanging tip-down, as in the drop test: rotate 180 deg about X, tip at z = 0
peg.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))
peg.apply_translation([0, 0, -peg.bounds[0][2]])
# One half-space per face: n . p + d <= 0 inside
planes = np.c_[peg.face_normals, -np.einsum('ij,ij->i', peg.face_normals, peg.triangles[:, 0])]
peg_lo, peg_hi = peg.bounds

import sys
if '--tiles' in sys.argv:
    # Test against the collision tiles from build_hole_tiles.py instead of the CAD surface
    pattern = 'hole_tilec_*.obj' if '--coarse' in sys.argv else 'hole_tile_*.obj'
    tiles = [trimesh.load(f, force='mesh') for f in sorted(MESHES.glob(pattern))]
    pts = np.vstack([trimesh.sample.sample_surface_even(t, 600)[0] for t in tiles] +
                    [t.vertices for t in tiles])
    print(f'testing against {len(tiles)} collision tiles')
else:
    pts, _ = trimesh.sample.sample_surface_even(hole, 400000)
    pts = np.vstack([pts, hole.vertices])


def inside_points(tip_z):
    p = pts - [0, 0, tip_z]
    box = np.all((p > peg_lo - TOL) & (p < peg_hi + TOL), axis=1)  # cheap pre-filter
    cand = np.nonzero(box)[0]
    hits = []
    for i in range(0, len(cand), CHUNK):
        idx = cand[i:i + CHUNK]
        q = p[idx]
        inside = np.all(q @ planes[:, :3].T + planes[:, 3] < -TOL, axis=1)
        hits.append(idx[inside])
    return np.concatenate(hits) if hits else np.array([], dtype=int)


lo, hi = -0.05, 0.3
assert len(inside_points(hi)) == 0
for _ in range(30):  # bisection to ~0.3 um
    mid = (lo + hi) / 2
    if len(inside_points(mid)):
        lo = mid
    else:
        hi = mid
print(f'peg seats with its tip {hi * 1000:.1f} mm above the hole bottom ({len(pts)} surface samples)')
first = pts[inside_points(lo - 0.0005)]
print('first contact points (mm, hole frame):')
print(np.round(first[np.argsort(first[:, 2])][:: max(1, len(first) // 8)][:8] * 1000, 1))
