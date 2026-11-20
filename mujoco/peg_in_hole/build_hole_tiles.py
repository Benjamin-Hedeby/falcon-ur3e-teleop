"""Collision geometry for the hole that follows its inner surface: thin convex tiles.

CoACD pieces cut across the concave inner faces of the socket's round ends and bulge into the
cavity (the peg then seats 13+ mm too high). Contact only ever happens on the inner skin, so this
builds the collision shape from that skin directly:

  - split the hole's surface points into height bands and angular wedges around the centre line,
  - in each cell keep the innermost layer (closest to the centre line),
  - add a copy pushed outward, and take the convex hull: a thin slab hugging the inner surface.

A tile bulges inward by at most the sagitta of its wedge (< 1 mm here). Writes meshes/hole_tile_*.obj;
check them with seat_depth.py --tiles.
"""
from pathlib import Path
import math
import warnings

import numpy as np
import trimesh

warnings.filterwarnings('ignore')
MESHES = Path(__file__).parent / 'meshes'
BAND = 0.008        # m, tile height
WEDGE_DEG = 10.0    # tile width around the centre line
SKIN = 0.004        # m, how deep the kept inner layer is
THICKNESS = 0.005   # m, how far the slab extends outward behind the skin
MIN_POINTS = 12


def build_tiles(hole):
    pts, _ = trimesh.sample.sample_surface_even(hole, 600000)
    pts = np.vstack([pts, hole.vertices])
    r = np.hypot(pts[:, 0], pts[:, 1])
    ang = np.degrees(np.arctan2(pts[:, 1], pts[:, 0])) % 360.0
    tiles = []
    z_max = pts[:, 2].max()
    for z0 in np.arange(0.0, z_max, BAND):
        in_band = (pts[:, 2] >= z0 - 0.0005) & (pts[:, 2] <= z0 + BAND + 0.0005)
        for a0 in np.arange(0.0, 360.0, WEDGE_DEG):
            cell = in_band & (ang >= a0 - 0.5) & (ang < a0 + WEDGE_DEG + 0.5)  # small overlap
            if cell.sum() < MIN_POINTS:
                continue
            p, rc = pts[cell], r[cell]
            skin = p[rc <= rc.min() + SKIN]
            if len(skin) < 4:
                continue
            radial = skin[:, :2] / np.linalg.norm(skin[:, :2], axis=1, keepdims=True)
            back = skin.copy()
            back[:, :2] += radial * THICKNESS
            try:
                tile = trimesh.Trimesh(np.vstack([skin, back])).convex_hull
            except Exception:
                continue
            if tile.volume > 1e-9:
                tiles.append(tile)
    return tiles


if __name__ == '__main__':
    import sys
    # --coarse: fewer, larger tiles (meshes/hole_tilec_*.obj) for a faster, live simulation
    prefix = 'hole_tile_'
    if '--coarse' in sys.argv:
        BAND, WEDGE_DEG, prefix = 0.016, 15.0, 'hole_tilec_'
    hole = trimesh.load(MESHES / 'hole_visual.obj', force='mesh')
    tiles = build_tiles(hole)
    for old in MESHES.glob(prefix + '*.obj'):
        old.unlink()
    for i, t in enumerate(tiles):
        t.export(MESHES / f'{prefix}{i:03d}.obj')
    print(f'{len(tiles)} tiles, {sum(len(t.vertices) for t in tiles)} vertices in total')

    print('check the seating depth with: seat_depth.py --tiles')
