import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from deux_task_core.synthetic import fixture
from deux_task_core.model import RobotModel
from deux_task_core.ik_backends import CheckedIK

@pytest.mark.parametrize('position,angle,accepted',[(.012341,.069489,True),(.019,np.deg2rad(4.9),True),(.021,0.,False),(0.,np.deg2rad(5.1),False)])
def test_requested_residual_limits(tmp_path,position,angle,accepted):
    cfg=fixture(tmp_path)['robot'];cfg.update(ik_position_tolerance_m=.02,ik_rotation_tolerance_rad=np.deg2rad(5.))
    robot=RobotModel(cfg);q=np.zeros(28);target=robot.ee_fk('right',q).copy()
    target[0,3]+=position;target[:3,:3]=Rotation.from_rotvec([0,0,angle]).as_matrix()@target[:3,:3]
    solver=CheckedIK(robot)
    if accepted:np.testing.assert_allclose(solver.checked('right',target,q,q[14:21]),q)
    else:
        with pytest.raises(ValueError,match='residual'):solver.checked('right',target,q,q[14:21])

@pytest.mark.parametrize('value',[0.,-1.,float('nan'),float('inf')])
def test_invalid_position_tolerance_rejected(tmp_path,value):
    cfg=fixture(tmp_path)['robot'];cfg['ik_position_tolerance_m']=value
    with pytest.raises(ValueError,match='tolerance'):RobotModel(cfg)
