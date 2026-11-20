"""Build the MuJoCo peg-in-hole model: the demo's UR3e + the peg on the flange + the socket.

- Peg: welded to wrist_3_link (the flange), hanging along the tool Z axis. Its 6-axis
  force/torque sensor sits between flange and peg, like the UR3e's wrist sensor.
  Collision: the peg's convex hull. Mass 0.733 kg.
- Socket: fixed in the world, opening up. Collision: the inner-surface tiles from
  build_hole_tiles.py (convex pieces that follow the concave inner faces).
- Only peg <-> socket contacts are enabled; the arm itself doesn't collide.

The demo's model places the robot so that the MuJoCo world frame is the UR's base_link frame,
the frame falcon_ur_teleop.py's admittance works in.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco

HERE = Path(__file__).resolve().parent
# UR3e model: Universal Robots' official meshes/parameters (BSD-3, see third_party/ur3e/LICENSE)
DEMO_SCENE = HERE.parent / 'third_party' / 'ur3e' / 'scene.xml'
MESHES = HERE / 'meshes'
PEG_LENGTH = 0.2257   # m, flange to tip
PEG_MASS = 0.733      # kg
FRICTION = 0.3        # sliding friction between peg and socket (plastic on plastic, roughly)


def build_xml(hole_pos=(0.30, 0.0, -0.15), peg_maxhullvert=None, hole_alpha=1.0, peg_slabs=False,
              timestep=None, coarse_tiles=False):
    root = ET.parse(DEMO_SCENE).getroot()
    root.set('model', 'UR3e peg-in-hole')
    root.find('compiler').set('meshdir', str(DEMO_SCENE.parent / 'visual'))
    if timestep:
        root.find('option').set('timestep', str(timestep))

    asset = root.find('asset')
    peg_coll = {'name': 'peg_collision', 'file': str(MESHES / 'peg_collision.obj')}
    if peg_maxhullvert:
        peg_coll['maxhullvert'] = str(peg_maxhullvert)
    ET.SubElement(asset, 'mesh', name='peg_visual', file=str(MESHES / 'peg_visual.obj'))
    ET.SubElement(asset, 'mesh', **peg_coll)
    slabs = sorted(MESHES.glob('peg_slab_*.obj')) if peg_slabs else []
    if peg_slabs and not slabs:
        raise FileNotFoundError('No peg slabs: run split_peg.py')
    for i, sl in enumerate(slabs):
        ET.SubElement(asset, 'mesh', name=f'peg_slab_{i}', file=str(sl))
    ET.SubElement(asset, 'mesh', name='hole_visual', file=str(MESHES / 'hole_visual.obj'))
    # coarse: build_hole_tiles.py --coarse (fewer tiles, faster, ~1 mm less accurate)
    tiles = sorted(MESHES.glob('hole_tilec_*.obj' if coarse_tiles else 'hole_tile_*.obj'))
    if not tiles:
        raise FileNotFoundError('No hole tiles: run prepare_meshes.py and build_hole_tiles.py')
    for i, t in enumerate(tiles):
        ET.SubElement(asset, 'mesh', name=f'hole_tile_{i}', file=str(t))

    # Contacts: peg (contype 1) only against socket tiles (conaffinity 1)
    default = root.find('default')
    peg_cls = ET.SubElement(default, 'default', {'class': 'peg_contact'})
    ET.SubElement(peg_cls, 'geom', contype='1', conaffinity='0', group='3',
                  friction=f'{FRICTION} 0.005 0.0001', solref='0.004 1', rgba='0.3 0.5 0.9 0.3')
    hole_cls = ET.SubElement(default, 'default', {'class': 'hole_contact'})
    ET.SubElement(hole_cls, 'geom', contype='0', conaffinity='1', group='3',
                  friction=f'{FRICTION} 0.005 0.0001', solref='0.004 1', rgba='0.9 0.5 0.3 0.3')

    wrist3 = next(b for b in root.iter('body') if b.get('name') == 'wrist_3_link')
    peg = ET.SubElement(wrist3, 'body', name='peg')
    ET.SubElement(peg, 'inertial', pos=f'0 0 {PEG_LENGTH / 2:.4f}', mass=str(PEG_MASS),
                  diaginertia='0.0045 0.0060 0.0040')
    ET.SubElement(peg, 'site', name='ft_sensor', size='0.006', rgba='0.9 0.9 0.1 1')
    ET.SubElement(peg, 'site', name='peg_tip', pos=f'0 0 {PEG_LENGTH}', size='0.006',
                  rgba='0.1 0.9 0.2 1')
    ET.SubElement(peg, 'geom', type='mesh', mesh='peg_visual', group='1', contype='0',
                  conaffinity='0', rgba='0.25 0.45 0.8 1', mass='0')
    # Collision: the whole hull, or horizontal slabs of it (much faster inside the socket: each
    # slab only meets the tiles at its own height)
    for mesh in ([f'peg_slab_{i}' for i in range(len(slabs))] if slabs else ['peg_collision']):
        ET.SubElement(peg, 'geom', {'class': 'peg_contact', 'type': 'mesh', 'mesh': mesh,
                                    'mass': '0'})

    world = root.find('worldbody')
    # The socket stands on the floor; the robot gets a pedestal up to its base (z = 0)
    floor = next(g for g in world.iter('geom') if g.get('name') == 'floor')
    floor.set('pos', f'0 0 {hole_pos[2]:.4f}')
    ET.SubElement(world, 'geom', name='pedestal', type='cylinder', size=f'0.075 {-hole_pos[2] / 2:.4f}',
                  pos=f'0 0 {hole_pos[2] / 2:.4f}', rgba='0.35 0.37 0.4 1', group='1')
    hole = ET.SubElement(world, 'body', name='hole', pos=' '.join(f'{c:.4f}' for c in hole_pos))
    ET.SubElement(hole, 'site', name='hole_bottom', size='0.006', rgba='0.9 0.2 0.2 1')
    ET.SubElement(hole, 'geom', type='mesh', mesh='hole_visual', group='1', contype='0',
                  conaffinity='0', rgba=f'0.8 0.55 0.3 {hole_alpha}')
    for i in range(len(tiles)):
        ET.SubElement(hole, 'geom', {'class': 'hole_contact', 'type': 'mesh',
                                     'mesh': f'hole_tile_{i}'})

    sensor = ET.SubElement(root, 'sensor')
    ET.SubElement(sensor, 'force', name='ft_force', site='ft_sensor')
    ET.SubElement(sensor, 'torque', name='ft_torque', site='ft_sensor')
    return ET.tostring(root, encoding='unicode')


def load(**kwargs):
    return mujoco.MjModel.from_xml_string(build_xml(**kwargs))


if __name__ == '__main__':
    m = load()
    print(f'model ok: {m.nbody} bodies, {m.ngeom} geoms, {m.nmesh} meshes, {m.nu} actuators, '
          f'{m.nsensor} sensors')
