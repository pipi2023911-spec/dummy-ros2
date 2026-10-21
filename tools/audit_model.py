"""D0 source audit. No ROS, collision engine or dynamics verification is claimed."""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import tempfile
import xml.etree.ElementTree as ET

import xacro
import yaml


def audit(workspace):
    src = workspace / 'src'
    desc = src / 'dummy-ros2_description'
    config = src / 'dummy_moveit_config'
    root = ET.parse(desc / 'urdf/dummy-ros2.xacro').getroot()
    links = {e.attrib['name'] for e in root.findall('link')}
    joints = []
    children = set()
    for joint in root.findall('joint'):
        parent = joint.find('parent').attrib['link']
        child = joint.find('child').attrib['link']
        assert parent in links and child in links and child not in children
        children.add(child)
        limit = joint.find('limit')
        axis = joint.find('axis')
        joints.append(dict(name=joint.attrib['name'], type=joint.attrib['type'],
                           parent=parent, child=child,
                           origin=joint.find('origin').attrib,
                           axis=None if axis is None else axis.attrib['xyz'],
                           limits=None if limit is None else limit.attrib))
    assert links - children == {'world'}
    by_child = {j['child']: j for j in joints}
    for link in links:
        seen = set()
        while link != 'world':
            assert link not in seen, 'cycle'
            seen.add(link)
            link = by_child[link]['parent']
    active = [j for j in joints if j['type'] != 'fixed']
    assert [j['name'] for j in active] == [f'Joint{i}' for i in range(1, 7)]
    for j in active:
        assert float(j['limits']['lower']) < float(j['limits']['upper'])
        assert abs(sum(float(x)**2 for x in j['axis'].split()) - 1) < 1e-9

    meshes = []
    for link in root.findall('link'):
        for kind in ('visual', 'collision'):
            for mesh in link.findall(f'{kind}/geometry/mesh'):
                name = Path(mesh.attrib['filename']).name
                path = desc / 'meshes' / name
                data = path.read_bytes()
                n = struct.unpack_from('<I', data, 80)[0]
                assert len(data) == 84 + 50*n, f'Invalid binary STL: {path}'
                bounds = [[math.inf, -math.inf] for _ in range(3)]
                for i in range(n):
                    values = struct.unpack_from('<12f', data, 84 + 50*i)
                    assert all(math.isfinite(v) for v in values)
                    for k, value in enumerate(values[3:]):
                        axis = k % 3
                        bounds[axis][0] = min(bounds[axis][0], value)
                        bounds[axis][1] = max(bounds[axis][1], value)
                meshes.append(dict(link=link.attrib['name'], kind=kind,
                                   file=name, triangles=n, raw_bounds=bounds,
                                   scale=mesh.attrib.get('scale'),
                                   sha256=hashlib.sha256(data).hexdigest()))

    srdf = ET.parse(config / 'config/dummy-ros2.srdf').getroot()
    chain = srdf.find("group[@name='dummy_arm']/chain").attrib
    path_joints = []
    link = chain['tip_link']
    while link != chain['base_link']:
        joint = by_child[link]
        path_joints.insert(0, joint['name'])
        link = joint['parent']
    assert path_joints == [j['name'] for j in active]
    for e in srdf.findall('disable_collisions'):
        assert e.attrib['link1'] in links and e.attrib['link2'] in links
    for pose in srdf.findall('group_state'):
        values = {j.attrib['name']: float(j.attrib['value']) for j in pose}
        assert set(values) == set(path_joints)
        for j in active:
            assert float(j['limits']['lower']) <= values[j['name']] <= float(j['limits']['upper'])

    for path in src.rglob('*.py'):
        ast.parse(path.read_text(encoding='utf-8'))
    for path in src.rglob('*.yaml'):
        yaml.safe_load(path.read_text(encoding='utf-8'))

    # Actual xacro expansion, with ONLY package-path substitutions in temporary
    # copies. This is explicitly not a test of ament package discovery.
    with tempfile.TemporaryDirectory(prefix='dummy_d0_') as tmp:
        tmp = Path(tmp)
        for path in src.rglob('*'):
            if path.is_file() and path.suffix in ('.xacro', '.trans', '.gazebo', '.yaml'):
                target = tmp / path.relative_to(src)
                target.parent.mkdir(parents=True, exist_ok=True)
                text = path.read_text()
                text = re.sub(r'\$\(find ([^)]+)\)',
                              lambda m: (tmp / m[1]).as_posix(), text)
                target.write_text(text, encoding='utf-8')
        expanded = xacro.process_file(str(
            tmp / 'dummy_moveit_config/config/dummy-ros2.urdf.xacro')).toxml()
        urdf = ET.fromstring(expanded)
        assert len(urdf.findall('joint')) == 7
        hardware = urdf.find('ros2_control')
        assert hardware.find('hardware/plugin').text == 'mock_components/GenericSystem'
        assert [j.attrib['name'] for j in hardware.findall('joint')] == path_joints
    return dict(status='PASS_STATIC_ONLY', links=len(links), joints=joints,
                group='dummy_arm', group_variables=path_joints, chain=chain,
                end_effector_elements=len(srdf.findall('end_effector')),
                disabled_collisions=[e.attrib for e in srdf.findall('disable_collisions')],
                meshes=meshes, xacro_expansion='PASS with temporary package-path substitution',
                limitations=['No ament discovery or Jazzy build tested',
                             'No RViz or MoveIt runtime tested',
                             'No geometric self-collision validation',
                             'STL checks validate binary structure, not watertightness',
                             'No physical robot limits or mechanical variant equivalence verified'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = audit(Path(__file__).resolve().parents[1])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding='utf-8')
    print(f"{result['status']}: {result['links']} links, "
          f"{len(result['group_variables'])} planning variables, "
          f"{len(result['meshes'])} visual/collision mesh references")
