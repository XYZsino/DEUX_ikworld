"""Conservative per-link URDF mesh silhouettes; no invented camera extrinsics."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import cv2
from scipy.spatial import ConvexHull
from deux_task_core.model import origin


def stl_vertices(path):
    data=Path(path).read_bytes()
    count=int.from_bytes(data[80:84],'little') if len(data)>=84 else 0
    if len(data)==84+50*count and count:
        dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
        points=np.frombuffer(data,dtype=dtype,offset=84)['vertices'].reshape(-1,3)
    else:
        points=np.array([[float(v) for v in line.split()[1:]] for line in data.decode().splitlines() if line.strip().startswith('vertex ')])
    points=np.unique(points,axis=0)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<4 or not np.isfinite(points).all():raise ValueError('invalid hand mesh')
    return points[ConvexHull(points).vertices]


class HandSilhouettes:
    def __init__(self,urdf,resolve_package=None):
        if resolve_package is None:
            from ament_index_python.packages import get_package_share_directory
            resolve_package=get_package_share_directory
        self.parts=[]
        for link in ET.parse(urdf).getroot().findall('link'):
            name=link.get('name')
            if not (name.startswith(('left_hand_','right_hand_')) or name in ('left_arm_6','left_arm_7','right_arm_6','right_arm_7')):continue
            for shape in link.findall('collision'):
                mesh=shape.find('geometry/mesh')
                if mesh is None:continue
                filename=mesh.get('filename')
                if filename.startswith('package://'):
                    package,relative=filename[len('package://'):].split('/',1);path=Path(resolve_package(package))/relative
                else:path=Path(urdf).parent/filename
                points=stl_vertices(path)*np.array(mesh.get('scale','1 1 1').split(),float)
                transform=origin(shape.find('origin'));points=np.c_[points,np.ones(len(points))]@transform.T
                self.parts.append((name,points))
        if not any(n.startswith('right_hand_') for n,_ in self.parts):raise ValueError('right hand geometry missing')

    def project(self,robot,q,camera_from_base,projection,shape,padding=8):
        mask=np.zeros(shape,np.uint8);transforms=robot.fk(q);p=np.asarray(projection).reshape(3,4)
        for name,points in self.parts:
            camera=points@(camera_from_base@transforms[name]).T
            if np.all(camera[:,2]<=.01):continue
            if np.any(camera[:,2]<=.01):raise ValueError('hand mesh crosses camera plane; exclusion uncertain')
            uv=camera@p.T;uv=uv[:,:2]/uv[:,2:3]
            if not np.isfinite(uv).all():raise ValueError('invalid hand projection')
            hull=cv2.convexHull(np.clip(np.rint(uv),-100000,100000).astype(np.int32))
            cv2.fillConvexPoly(mask,hull,255)
        if padding:mask=cv2.dilate(mask,np.ones((2*padding+1,2*padding+1),np.uint8))
        return mask.astype(bool)
