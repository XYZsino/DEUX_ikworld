import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from deux_task_core.synthetic import fixture
from deux_task_core.model import RobotModel


def test_reader_axis_ignores_roll_with_translated_rotated_tool(tmp_path):
    pytest.importorskip('pyroki')
    from deux_task_core.ik_pyroki import PyrokiIK
    config=fixture(tmp_path);robot=RobotModel(config['robot']);solver=PyrokiIK(robot)
    q=np.zeros(28);offset=np.eye(4)
    offset[:3,3]=[.02,.01,.06]
    offset[:3,:3]=Rotation.from_euler('xyz',[.3,.2,.4]).as_matrix()
    target=robot.ee_fk('left',q)@offset
    solutions=[]
    for angle in [0,90,180]:
        rolled=target.copy()
        rolled[:3,:3]=target[:3,:3]@Rotation.from_euler('z',angle,degrees=True).as_matrix()
        answer=solver.solve_reader_axis('left',rolled,offset,q)
        actual=robot.ee_fk('left',answer)@offset
        np.testing.assert_allclose(actual[:3,3],target[:3,3],atol=1e-5)
        np.testing.assert_allclose(actual[:3,2],target[:3,2],atol=1e-5)
        np.testing.assert_array_equal(answer[7:],q[7:])
        solutions.append(answer)
    np.testing.assert_allclose(solutions[0],solutions[1],atol=1e-6)
    np.testing.assert_allclose(solutions[0],solutions[2],atol=1e-6)
