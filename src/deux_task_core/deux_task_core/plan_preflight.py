"""Cheap observation check for an already validated, fixed-time trajectory.

No inverse kinematics, timestamp refresh, or trajectory changes are performed.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from .conveyor import forecast


def compare_prediction(config,target,velocity,acquired,latest,latest_velocity,latest_acquired,close_at):
    if not acquired<=latest_acquired<=close_at:
        raise ValueError('invalid acquisition order for cached path')
    expected=forecast(target,velocity,latest_acquired-acquired)
    end=forecast(target,velocity,close_at-acquired)
    fresh_end=forecast(latest,latest_velocity,close_at-latest_acquired)
    position=max(float(np.linalg.norm(expected[:3,3]-latest[:3,3])),
                 float(np.linalg.norm(end[:3,3]-fresh_end[:3,3])))
    angle=max(float(Rotation.from_matrix(expected[:3,:3]@latest[:3,:3].T).magnitude()),
              float(Rotation.from_matrix(end[:3,:3]@fresh_end[:3,:3].T).magnitude()))
    reuse=position<=config['max_target_drift'] and angle<=config.get('max_target_angle_rad',.15)
    return dict(reuse=reuse,position_error_m=position,angle_error_rad=angle,
                reason='observation agrees; keep validated trajectory' if reuse else 'observation changed; search again')
