"""Optional IK engines; full robot/path validation remains in the common planner."""
from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from .model import ARM_NAMES, JOINT_NAMES, vector, pose

BACKENDS = ('scipy', 'pyroki', 'curobo')


def backend_name(config):
    name = config.get('ik_backend', 'pyroki')
    if name not in BACKENDS:
        raise ValueError('ik_backend must be scipy, pyroki or curobo')
    return name


def arm_names(side):
    if side not in ('left', 'right'):
        raise ValueError('invalid arm side')
    return ARM_NAMES[:7] if side == 'left' else ARM_NAMES[7:]


def write_arm_urdf(robot, side, path):
    """Export the exact base→grasp chain, folding configured fixed joint values.

    Fingers/other arm are NOT included in this IK-only model. They remain in
    the independent 28-joint collision checker and returned command vector.
    """
    names = arm_names(side)
    root = ET.Element('robot', name='deux_' + side + '_ik')
    ET.SubElement(root, 'link', name=robot.base)
    found = []
    for name, source in robot.ee_chains[side]:
        ET.SubElement(root, 'link', name=source['child'])
        transform = source['origin'].copy()
        active = name in names
        if active:
            found.append(name)
        elif source['kind'] != 'fixed':
            if name not in robot.fixed:
                raise ValueError('IK chain contains an unmeasured joint: ' + name)
            motion = np.eye(4);value = robot.fixed[name]
            if source['kind'] in ('revolute','continuous'):
                motion[:3,:3] = Rotation.from_rotvec(source['axis']*value).as_matrix()
            elif source['kind'] == 'prismatic':
                motion[:3,3] = source['axis']*value
            else:raise ValueError('unsupported IK chain joint')
            transform = transform @ motion
        joint = ET.SubElement(root, 'joint', name=name, type=source['kind'] if active else 'fixed')
        ET.SubElement(joint, 'parent', link=source['parent'])
        ET.SubElement(joint, 'child', link=source['child'])
        ET.SubElement(joint, 'origin', xyz=' '.join(map(str, transform[:3,3])),
                      rpy=' '.join(map(str, Rotation.from_matrix(transform[:3,:3]).as_euler('xyz'))))
        if active:
            ET.SubElement(joint, 'axis', xyz=' '.join(map(str, source['axis'])))
            ET.SubElement(joint, 'limit', **dict(source['limit'].attrib))
    if set(found) != set(names):
        raise ValueError('IK chain does not contain the expected seven arm joints')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


class CheckedIK:
    """Name-based joint mapping and independent FK/limits check for every solve."""
    def __init__(self, robot):
        self.robot = robot
        self.directory = TemporaryDirectory(prefix='deux_ik_')
        self.engines = {}

    def engine(self, side):
        if side not in self.engines:
            path = Path(self.directory.name) / (side + '.urdf')
            write_arm_urdf(self.robot, side, path)
            self.engines[side] = self.make_engine(side, path)
        return self.engines[side]

    def solve(self, side, target, seed, continuity_weight):
        target = pose(target);q = vector(seed,28).copy()
        if not np.isfinite(continuity_weight) or continuity_weight < 0:
            raise ValueError('invalid IK continuity weight')
        names = arm_names(side);indices = np.array([JOINT_NAMES.index(n) for n in names])
        values = vector(self.solve_arm(self.engine(side), target, q[indices], continuity_weight),7)
        return self.checked(side, target, q, values)

    def checked(self, side, target, q, values):
        q = vector(q,28).copy();values = vector(values,7)
        indices = np.array([JOINT_NAMES.index(n) for n in arm_names(side)])
        if np.any(values < self.robot.limits[indices,0]-1e-6) or np.any(values > self.robot.limits[indices,1]+1e-6):
            raise ValueError('IK backend joint limit violation')
        q[indices] = np.clip(values,self.robot.limits[indices,0],self.robot.limits[indices,1])
        actual = self.robot.ee_fk(side,q)
        position_error = np.linalg.norm(actual[:3,3]-target[:3,3])
        angle_error = Rotation.from_matrix(target[:3,:3]@actual[:3,:3].T).magnitude()
        if position_error > .002 or angle_error > .02:
            raise ValueError(f'IK backend residual exceeds tolerance: {position_error:.6f}m, {angle_error:.6f}rad')
        return q


    def solve_batch(self, side, targets, seeds, continuity_weight=.005):
        targets = np.array([pose(t) for t in targets]);seeds = np.array([vector(q,28) for q in seeds])
        if len(targets)!=len(seeds) or not len(targets):raise ValueError('invalid IK batch shape')
        if not np.isfinite(continuity_weight) or continuity_weight<0:raise ValueError('invalid continuity weight')
        indices = np.array([JOINT_NAMES.index(n) for n in arm_names(side)])
        values, errors = self.solve_arm_batch(side, targets, seeds[:,indices], continuity_weight)
        output=seeds.copy();reasons=[]
        for i in range(len(targets)):
            try:
                if errors[i]:raise ValueError(errors[i])
                output[i]=self.checked(side,targets[i],seeds[i],values[i])
                reasons.append('')
            except ValueError as error:reasons.append(str(error))
        self.robot.ik_calls += len(targets)
        return output,reasons


def attach_backend(robot, config):
    name = backend_name(config)
    if name == 'scipy':return
    try:
        if name == 'pyroki':
            from .ik_pyroki import PyrokiIK
            robot.ik_solver = PyrokiIK(robot)
        else:
            from .ik_curobo import CuroboIK
            robot.ik_solver = CuroboIK(robot)
    except ImportError as error:
        raise ValueError(f'{name} backend dependencies unavailable; install the optional planner environment') from error


def warmup(planner):
    if backend_name(planner.config) == 'scipy':return
    q = vector(planner.config['home_joints'],28)
    for side in ('left','right'):
        target = planner.robot.ee_fk(side,q)
        planner.robot.ik(side,target,q,continuity_weight=.02)
    temporal=planner.config.get('conveyor_tracking',{}).get('candidate_search',{}).get('time_search',{})
    if temporal.get('enabled',False):
        from .time_candidates import time_grid
        count=len(time_grid(temporal))*len(planner.config['conveyor_tracking']['candidate_search']['candidates'])
        planner.robot.ik_solver.solve_batch('right',np.repeat(planner.robot.ee_fk('right',q)[None],count,axis=0),np.repeat(q[None],count,axis=0))
    planner.robot.ik_calls = 0
