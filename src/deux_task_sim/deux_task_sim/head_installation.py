"""Fixed installation configuration without reading any simulated object pose."""
from importlib.resources import files
from pathlib import Path
import yaml
import numpy as np


def configure_head(sim, output, ik_backend=None):
    output=Path(output)
    config=yaml.safe_load(files('deux_task_sim').joinpath('head_calibration.yaml').read_text())
    if ik_backend is not None:config['ik_backend']=ik_backend
    config['robot']['urdf_path']=str(output/'robot.urdf')
    # Known installation command, not simulator object velocity/pose truth.
    config['conveyor_tracking']['nominal_velocity']=[0.,sim.configured_conveyor_speed,0.,0.]
    from hashlib import sha256
    from deux_conveyor import BREAD_ASSET, BREAD_MODEL
    mesh=BREAD_ASSET/BREAD_MODEL['mesh']
    config['model_id']='obj-sha256:'+sha256(mesh.read_bytes()).hexdigest()+':bounds_center:metres'
    # The source support is static installation geometry, not a detected object.
    geom=sim.model.geom('source_box_2_floor').id
    floor_z=float(sim.data.geom_xpos[geom,2]+sim.model.geom_size[geom,2])
    base=sim.data.body('base_link')
    wb=np.eye(4);wb[:3,:3]=base.xmat.reshape(3,3);wb[:3,3]=base.xpos
    wr=np.eye(4);wr[2,3]=floor_z
    config['base_from_source_support']=(np.linalg.inv(wb)@wr).tolist()
    (output/'installation.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    (output/'hand_mapping.yaml').write_text(files('deux_task_sim').joinpath('head_hand_mapping.yaml').read_text())
    return config
