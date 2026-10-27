"""cuRobo V2 multi-seed IK; shared full-body checks validate returned paths."""
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from curobo.inverse_kinematics import InverseKinematics, InverseKinematicsCfg
from curobo.types import GoalToolPose, Pose, JointState
from .ik_backends import CheckedIK, arm_names
from .model import JOINT_NAMES


class CuroboIK(CheckedIK):
    def make_engine(self, side, path):
        if not torch.cuda.is_available():raise ValueError('cuRobo requires a working CUDA device')
        names = list(arm_names(side));indices = [JOINT_NAMES.index(n) for n in names]
        robot_config = {'robot_cfg': {'kinematics': {
            'urdf_path':str(path),'asset_root_path':str(path.parent),
            'base_link':self.robot.base,'tool_frames':[self.robot.ee[side]],
            'collision_link_names':[],'collision_spheres':None,'mesh_link_names':[],
            'cspace':{'joint_names':names,
                      'default_joint_position':np.mean(self.robot.limits[indices],axis=1).tolist(),
                      'null_space_weight':[1.]*7,'cspace_distance_weight':[1.]*7,
                      'max_acceleration':float(min(self.robot.acceleration[indices])),
                      'max_jerk':100.}}}}
        cfg = InverseKinematicsCfg.create(robot=robot_config, optimizer_configs=['ik/lbfgs_ik.yml'],
            num_seeds=8,seed_solver_num_seeds=32,use_cuda_graph=True,
            self_collision_check=False,load_collision_spheres=False,
            position_tolerance=.001,orientation_tolerance=.01,
            seed_position_weight=1.,seed_orientation_weight=.15,
            velocity_regularization_weight=0.,acceleration_regularization_weight=0.,
            success_requires_convergence=False)
        cfg.exit_early=False
        solver=InverseKinematics(cfg)
        order=list(solver.joint_names)
        if set(order)!=set(names):raise ValueError('cuRobo joint mapping mismatch')
        return solver,np.array([names.index(n) for n in order]),np.array([order.index(n) for n in names])

    def solve_arm(self, engine, target, seed, continuity_weight):
        solver,to_engine,to_common=engine
        def tensor(value):return torch.as_tensor(np.asarray(value),device='cuda',dtype=torch.float32)
        xyzw=Rotation.from_matrix(target[:3,:3]).as_quat()
        goal=Pose(position=tensor(target[:3,3][None]),quaternion=tensor(xyzw[[3,0,1,2]][None]))
        current=JointState.from_position(tensor(seed[to_engine][None]),joint_names=solver.joint_names)
        result=solver.solve_pose(GoalToolPose.from_poses({solver.tool_frames[0]:goal},
            ordered_tool_frames=solver.tool_frames,num_goalset=1),current_state=current,return_seeds=8)
        success=result.success.reshape(-1).detach().cpu().numpy().astype(bool)
        solutions=solver.get_active_js(result.js_solution).position.reshape(-1,7).detach().cpu().numpy()[:,to_common]
        success &= np.isfinite(solutions).all(axis=1)
        if not success.any():raise ValueError('cuRobo IK unreachable')
        solutions=solutions[success]
        # Preserve the seed branch when multiple GPU seeds solve the same pose.
        return solutions[np.argmin(np.linalg.norm(solutions-seed,axis=1))]
