"""Small static geometry report. Never generates or dispatches a motion."""
from uuid import uuid4
import numpy as np
from scipy.spatial.transform import Rotation
from .motion_review import geometry

def report(robot,config,version,stage,start,obj,error):
    obj=np.asarray(obj);offset=np.asarray(config['grasp_object_from_right_ee'])
    grasp=obj@offset;goal=grasp.copy()
    if stage=='Approach':goal[2,3]+=config['approach_height']
    if stage=='Descend' and 'descend_distance_m' in config:
        goal=robot.fk(start)[robot.ee['right']].copy()
        goal[2,3]-=config['descend_distance_m']
    attempt=getattr(robot,'last_ik_attempt',None)
    out=dict(id=uuid4().hex,stage=stage,frame=robot.base,scene_version=version,error=error,
        current=geometry(robot,start),start_joints=np.asarray(start).tolist(),
        object_pose=obj.tolist(),grasp_transform=offset.tolist(),grasp_pose=grasp.tolist(),
        target_pose=goal.tolist(),approach_height_m=config['approach_height'],attempt=attempt,
        rejected=None,position_error_m=None,rotation_error_deg=None)
    if attempt:
        actual=np.asarray(attempt['actual']);target=np.asarray(attempt['target'])
        out['rejected']=geometry(robot,attempt['joints'])
        out['position_error_m']=float(np.linalg.norm(actual[:3,3]-target[:3,3]))
        out['rotation_error_deg']=float(np.degrees(Rotation.from_matrix(target[:3,:3]@actual[:3,:3].T).magnitude()))
    return out
