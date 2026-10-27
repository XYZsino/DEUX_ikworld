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
        return solve,to_engine,to_common

    def solve_arm(self, engine, target, seed, continuity_weight):
        solve,to_engine,to_common = engine
        xyzw = Rotation.from_matrix(target[:3,:3]).as_quat()
        values = solve(jnp.asarray(seed[to_engine],dtype=jnp.float32),
                       jnp.asarray(xyzw[[3,0,1,2]],dtype=jnp.float32),
                       jnp.asarray(target[:3,3],dtype=jnp.float32),
                       jnp.asarray(continuity_weight,dtype=jnp.float32))
        return np.asarray(values)[to_common]
