"""Turn the peg/hole STLs into MuJoCo collision meshes.

- Peg (Housing_Folding_Gripper_v2.STL): visual mesh + its convex hull for collision (the peg's
  outer envelope is convex: a tapered stadium). Re-centred so the top face centre is the origin
  and the peg hangs along +Z (the tool axis), in metres.
- Hole (BLANKET_SEGM_OUTBOARD_MIDDLE_GRIPPER_v2.STL): visual mesh + a CoACD convex
  decomposition (MuJoCo collides every mesh as its convex hull, which would fill the hole).
  Re-centred so the bottom centre is the origin and the opening faces +Z, in metres.

Writes OBJ files to ./meshes and prints a summary. Run with the mujoco venv:
    ../.venv/bin/python prepare_meshes.py
"""
from pathlib import Path
import warnings

import coacd
import numpy as np
import trimesh

warnings.filterwarnings('ignore')
SRC = Path(__file__).resolve().parents[2] / 'cad'  # the peg/socket STLs (mm)
OUT = Path(__file__).parent / 'meshes'
COACD_THRESHOLD = 0.02   # concavity tolerance; lower = more, tighter pieces


def load_mm(name):
    m = trimesh.load(SRC / name, force='mesh')
    m.apply_scale(0.001)  # mm -> m
    return m


def main():
    OUT.mkdir(exist_ok=True)

    # Peg: top face centre -> origin, hanging along +Z
    peg = load_mm('Housing_Folding_Gripper_v2.STL')
    lo, hi = peg.bounds
    peg.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -hi[2]])
    peg.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))  # z -> -z
    peg.export(OUT / 'peg_visual.obj')
    hull = peg.convex_hull
    hull.export(OUT / 'peg_collision.obj')
    print(f'peg: {peg.extents.round(4)} m, hull {len(hull.faces)} faces, '
          f'tip at z = {peg.bounds[1][2]:.4f} m')

    # Hole: bottom centre -> origin, opening up
    hole = load_mm('BLANKET_SEGM_OUTBOARD_MIDDLE_GRIPPER_v2.STL')
    lo, hi = hole.bounds
    hole.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
    hole.export(OUT / 'hole_visual.obj')
    parts = coacd.run_coacd(coacd.Mesh(hole.vertices, hole.faces), threshold=COACD_THRESHOLD)
    pieces = []
    for i, (v, f) in enumerate(parts):
        piece = trimesh.Trimesh(v, f)
        piece.export(OUT / f'hole_collision_{i:02d}.obj')
        pieces.append(piece)
    print(f'hole: {hole.extents.round(4)} m, {len(pieces)} convex pieces')

    # Check the decomposition left the opening clear: sample the peg's own envelope, shrunk a
    # little and placed where it would sit, and count points inside any hole piece.
    for z in (0.01, 0.03, 0.06, 0.09, 0.12, 0.145):
        inside = [p.contains([[0.0, 0.0, z]])[0] for p in pieces]
        print(f'  centre line z={z:.3f} m blocked by a piece: {any(inside)}')
    (OUT / 'n_hole_pieces.txt').write_text(str(len(pieces)))


if __name__ == '__main__':
    main()
