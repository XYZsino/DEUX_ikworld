"""Build a fixed-lift camera snapshot from URDF XY/yaw and measured floor normal."""
from pathlib import Path
import json,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
r=Path(__file__).resolve().parents[1]
floor=json.loads((r/"calibration/floor_plane.thor.json").read_text())
assert floor["plane_fit_accepted"]
urdf=r/"src/deux_task_bringup/urdf/deux_fixed_zed2i.urdf"
tree=ET.parse(urdf).getroot()
joints={j.find("child").get("link"):j for j in tree.findall("joint")}
def from_lift(link):
    out=np.eye(4)
    while link!="lift":
        j=joints[link];o=j.find("origin");t=np.eye(4)
        if o is not None:
            t[:3,3]=list(map(float,o.get("xyz","0 0 0").split()))
            t[:3,:3]=Rotation.from_euler("xyz",list(map(float,o.get("rpy","0 0 0").split()))).as_matrix()
        out=t@out;link=j.find("parent").get("link")
    return out
nominal=from_lift("zed_left_camera_frame_optical")
shoulder_z=from_lift("left_arm_1")[2,3]
body_opt=Rotation.from_quat([-.5,.5,-.5,.5]).as_matrix()
nominal_body=nominal[:3,:3]@body_opt.T
yaw=Rotation.from_matrix(nominal_body).as_euler("xyz")[2]
n=np.asarray(floor["plane"]["normal_camera"]);n/=np.linalg.norm(n)
world_y=np.cross(n,[0.,0.,1.]);world_y/=np.linalg.norm(world_y)
world_x=np.cross(world_y,n)
R=Rotation.from_euler("z",yaw).as_matrix()@np.stack((world_x,world_y,n))
Rbody=R@body_opt.T
# CAD-derived wheel contact level, not a physical measurement of tire compression.
base_height=.0006302772760391166
height=1.37910
translation=np.array([.114+nominal[0,3],nominal[1,3],height-base_height])
T=np.eye(4);T[:3,:3]=R;T[:3,3]=translation
assert np.allclose(R@n,[0,0,1],atol=1e-10)
assert np.isclose(np.linalg.det(R),1)
assert np.isclose(translation[2]+base_height,height)
assert np.isclose(Rotation.from_matrix(Rbody).as_euler("xyz")[2],yaw)
d=dict(schema="deux.camera_extrinsics.v1",camera_serial=33283654,parent_frame="base_link",child_frame="zed2i_left_camera_frame",optical_frame="zed2i_left_camera_optical_frame",translation_m=translation.tolist(),quaternion_xyzw=Rotation.from_matrix(Rbody).as_quat().tolist(),camera_body_rpy_deg=Rotation.from_matrix(Rbody).as_euler("xyz",degrees=True).tolist(),T_base_camera_optical=T.tolist(),camera_height_from_floor_m=height,user_shoulder_height_from_floor_m=1.34,nominal_urdf_camera_above_shoulder_m=float(nominal[2,3]-shoulder_z),measured_plane_camera_height_m=floor["plane"]["offset_m"],observed_floor_height_residual_m=height-floor["plane"]["offset_m"],base_height_from_floor_m=base_height,base_height_source="CAD wheel mesh contact: all four wheel minima z=-0.000630277m in base_link; approximate geometric datum, not measured tire compression",xy_and_yaw_source=str(urdf),orientation_source="accepted floor normal, URDF camera-body yaw preserved",z_source="user explicitly selected 1.37910m (shoulder measurement plus URDF mount difference); NOT depth-derived 1.35147m",mode="fixed measured lift snapshot, dynamic TF invalidated on lift movement command",xy_measured=False,full_extrinsic_calibration_confirmed=False)
(r/"calibration/zed2i_extrinsics.thor.json").write_text(json.dumps(d,indent=2))
snippet=ET.Element("robot",name="thor_calibrated_camera_snapshot")
ET.SubElement(snippet,"link",name="base_link");ET.SubElement(snippet,"link",name=d["child_frame"])
j=ET.SubElement(snippet,"joint",name="thor_calibrated_camera_snapshot_joint",type="fixed")
ET.SubElement(j,"parent",link="base_link");ET.SubElement(j,"child",link=d["child_frame"])
ET.SubElement(j,"origin",xyz=" ".join(map(str,translation)),rpy=" ".join(map(str,Rotation.from_matrix(Rbody).as_euler("xyz"))))
ET.ElementTree(snippet).write(r/"calibration/zed2i_snapshot.thor.urdf",encoding="utf-8",xml_declaration=True)
print(json.dumps(d,indent=2))
