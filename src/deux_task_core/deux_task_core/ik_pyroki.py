"""PyRoKi/JAX least-squares IK, compiled once per arm in the planner worker."""
import os
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE','false')
os.environ.setdefault('JAX_PLATFORMS','cpu')
import jax
import jax.numpy as jnp
import jaxlie
import jaxls
import numpy as np
import pyroki as pk
import yourdfpy
from scipy.spatial.transform import Rotation
from .ik_backends import CheckedIK, arm_names
from .model import JOINT_NAMES, vector, pose


class PyrokiIK(CheckedIK):
    def make_engine(self, side, path):
        robot = pk.Robot.from_urdf(yourdfpy.URDF.load(str(path), load_meshes=False))
        names = list(robot.joints.actuated_names)
        expected = list(arm_names(side))
        if set(names) != set(expected):raise ValueError('PyRoKi joint mapping mismatch')
        to_engine = np.array([expected.index(n) for n in names])
        to_common = np.array([names.index(n) for n in expected])
        link = jnp.array(robot.links.names.index(self.robot.ee[side]),dtype=jnp.int32)
        variable = robot.joint_var_cls(0)

        @jax.jit
        def solve(seed, wxyz, xyz, continuity):
            goal = jaxlie.SE3.from_rotation_and_translation(jaxlie.SO3(wxyz),xyz)
            costs = [pk.costs.pose_cost(robot,variable,goal,link,1.,.15),
                     pk.costs.rest_cost(variable,seed,jnp.maximum(continuity,1e-5)),
                     pk.costs.limit_constraint(robot,variable)]
            problem = jaxls.LeastSquaresProblem(costs,[variable]).analyze()
            values = problem.solve(initial_vals=jaxls.VarValues.make([variable.with_value(seed)]),verbose=False)
            return values[variable]
        def reader_residual(vals, variable, target_xyz, target_axis, ee_from_reader):
            ee = jaxlie.SE3(robot.forward_kinematics(vals[variable])[link]).as_matrix()
            reader = ee @ ee_from_reader
            # Vector difference penalizes antiparallel Z too, and is invariant to roll.
            return jnp.concatenate([reader[:3,3]-target_xyz,
                                    .15*(reader[:3,2]-target_axis)])
        reader_cost = jaxls.Cost.factory(reader_residual)

        @jax.jit
        def solve_reader(seed, xyz, axis, offset, continuity):
            costs=[reader_cost(variable,xyz,axis,offset),
                   pk.costs.rest_cost(variable,seed,jnp.maximum(continuity,1e-5)),
                   pk.costs.limit_constraint(robot,variable)]
            problem=jaxls.LeastSquaresProblem(costs,[variable]).analyze()
            values=problem.solve(initial_vals=jaxls.VarValues.make([variable.with_value(seed)]),verbose=False)
            return values[variable]
        if not hasattr(self,'reader_engines'): self.reader_engines={}
        self.reader_engines[side]=(solve_reader,to_engine,to_common)
        return solve,to_engine,to_common,jax.jit(jax.vmap(solve,in_axes=(0,0,0,None)))

    def solve_arm(self, engine, target, seed, continuity_weight):
        solve,to_engine,to_common,_ = engine
        xyzw = Rotation.from_matrix(target[:3,:3]).as_quat()
        values = solve(jnp.asarray(seed[to_engine],dtype=jnp.float32),
                       jnp.asarray(xyzw[[3,0,1,2]],dtype=jnp.float32),
                       jnp.asarray(target[:3,3],dtype=jnp.float32),
                       jnp.asarray(continuity_weight,dtype=jnp.float32))
        return np.asarray(values)[to_common]

    def solve_arm_batch(self, side, targets, seeds, continuity_weight):
        _,to_engine,to_common,solve=self.engine(side)
        xyzw=Rotation.from_matrix(targets[:,:3,:3]).as_quat()
        values=solve(jnp.asarray(seeds[:,to_engine],dtype=jnp.float32),
                     jnp.asarray(xyzw[:,[3,0,1,2]],dtype=jnp.float32),
                     jnp.asarray(targets[:,:3,3],dtype=jnp.float32),
                     jnp.asarray(continuity_weight,dtype=jnp.float32))
        return np.asarray(values)[:,to_common], ['']*len(targets)

    def solve_reader_axis(self, side, target_reader, ee_from_reader, seed, continuity_weight=0.):
        target_reader=pose(target_reader);offset=pose(ee_from_reader);q=vector(seed,28).copy()
        if not np.isfinite(continuity_weight) or continuity_weight < 0:
            raise ValueError('invalid IK continuity weight')
        self.engine(side)
        solve,to_engine,to_common=self.reader_engines[side]
        indices=np.array([JOINT_NAMES.index(n) for n in arm_names(side)])
        values=vector(np.asarray(solve(jnp.asarray(q[indices][to_engine],dtype=jnp.float32),
            jnp.asarray(target_reader[:3,3],dtype=jnp.float32),
            jnp.asarray(target_reader[:3,2],dtype=jnp.float32),
            jnp.asarray(offset,dtype=jnp.float32),
            jnp.asarray(continuity_weight,dtype=jnp.float32)))[to_common],7)
        attempted=q.copy();attempted[indices]=values
        actual=self.robot.ee_fk(side,attempted)
        # Independent common check: preserve solved free roll while requiring reader position.
        check_target=actual.copy()
        check_target[:3,3]=target_reader[:3,3]-actual[:3,:3]@offset[:3,3]
        self.robot.ik_calls+=1
        checked=self.checked(side,check_target,q,values)
        reader=self.robot.ee_fk(side,checked)@offset
        position_error=float(np.linalg.norm(reader[:3,3]-target_reader[:3,3]))
        axis_error=float(np.arccos(np.clip(reader[:3,2]@target_reader[:3,2],-1.,1.)))
        self.robot.last_ik_attempt.update(constraint='reader_position_z_axis',
            reader_target=target_reader.tolist(),reader_actual=reader.tolist(),
            reader_position_error_m=position_error,reader_axis_error_rad=axis_error)
        if position_error > self.robot.ik_position_tolerance_m or axis_error > self.robot.ik_rotation_tolerance_rad:
            raise ValueError(f'IK reader position/Z-axis residual exceeds tolerance: {position_error:.6f}m, {axis_error:.6f}rad')
        return checked
