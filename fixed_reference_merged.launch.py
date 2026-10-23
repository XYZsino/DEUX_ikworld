"""Load the selected user-supplied plane reference and start YOLO/ICP."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    root=Path(__file__).resolve().parent
    selected=json.loads((root/"fixed_reference_selection.json").read_text())
    entry=json.loads((root/"fixed_references.json").read_text())[selected["entry"]]
    t=np.asarray(entry["T_reference_camera"],dtype=float)
    if (t.shape!=(4,4) or not np.isfinite(t).all()
        or not np.allclose(t[3],[0,0,0,1])
        or not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-6)
        or not np.isclose(np.linalg.det(t[:3,:3]),1,atol=1e-6)):
        raise ValueError("Invalid T_reference_camera")
    # Existing camera URDF already parents the optical frame. Attach the plane
    # underneath optical using the inverse, avoiding two parents for optical.
    inverse=np.linalg.inv(t)
    quat=Rotation.from_matrix(inverse[:3,:3]).as_quat()
    xyz=inverse[:3,3]
    args=[]
    for name,value in zip(["x","y","z","qx","qy","qz","qw"],[*xyz,*quat]):
        args.extend(["--"+name,str(float(value))])
    args.extend(["--frame-id",selected["camera_frame"],"--child-frame-id","support_plane"])
    return LaunchDescription([
        Node(package="tf2_ros",executable="static_transform_publisher",
             name="fixed_support_plane_tf",arguments=args,output="screen"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory("deux_task_perception"))/"launch/merged_perception.launch.py")),
            launch_arguments={"config":str(root/"merged_perception.local.yaml")}.items())
    ])
