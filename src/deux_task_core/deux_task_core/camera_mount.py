"""Additive ZED 2i URDF assembly; image-based robot mount remains provisional."""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation


def mesh_vertices(path):
    data=Path(path).read_bytes();count=struct.unpack_from('<I',data,80)[0]
    if len(data)!=84+count*50:raise ValueError('expected binary STL in metres')
    dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
    return np.frombuffer(data,offset=84,count=count,dtype=dtype)['vertices'].reshape(-1,3).astype(float)


def mount_transform(config,mesh_path):
    pitch=np.deg2rad(float(config['pitch_deg']));rotation=Rotation.from_euler('y',pitch).as_matrix()
    anchor=np.asarray(config['eye_display_top_xyz'],float)
    gap=float(config['display_gap_m']);forward=float(config['mount_forward_offset_m'])
    if anchor.shape!=(3,) or not np.isfinite(np.r_[anchor,pitch,gap,forward]).all() or gap<0:
        raise ValueError('invalid camera mount dimensions')
    # Official camera_link -> center (z=.015), mesh visual origin (x=-.01).
    vertices=mesh_vertices(mesh_path)+[-.01,0.,.015]
    rotated=vertices@rotation.T
    xyz=anchor+[forward,0.,gap-float(rotated[:,2].min())]
    transform=np.eye(4);transform[:3,:3]=rotation;transform[:3,3]=xyz
    return transform,dict(lowest_camera_z=float(xyz[2]+rotated[:,2].min()),
        display_top_z=float(anchor[2]),vertical_gap_m=gap,calibration_confirmed=bool(config['calibration_confirmed']),
        optical_convention='x right, y down, z forward',source=config['provenance'])


def append_zed2i(root,config,mesh_path):
    if config['model']!='zed2i':raise ValueError('only the confirmed ZED 2i model is supported')
    name=config['name'];parent=config['parent_link']
    if root.find(f"link[@name='{parent}']") is None:raise ValueError('camera mounting parent absent')
    if root.find(f"link[@name='{name}_camera_link']") is not None:raise ValueError('camera already present')
    transform,audit=mount_transform(config,mesh_path)
    root.append(ET.Comment(' ZED 2i: '+('measured mount' if config['calibration_confirmed'] else 'PROVISIONAL SIM mount; eye display reference and forward offset require real calibration')+' '))
    def fixed(child,parent,xyz=(0,0,0),rpy=(0,0,0)):
        link=ET.SubElement(root,'link',name=child)
        joint=ET.SubElement(root,'joint',name=child+'_joint',type='fixed')
        ET.SubElement(joint,'parent',link=parent);ET.SubElement(joint,'child',link=child)
        ET.SubElement(joint,'origin',xyz=' '.join(map(str,xyz)),rpy=' '.join(map(str,rpy)))
        return link
    eye=name+'_eye_display_top_reference'
    fixed(eye,parent,config['eye_display_top_xyz'])
    relative=transform[:3,3]-np.asarray(config['eye_display_top_xyz'])
    fixed(name+'_camera_link',eye,relative,[0,np.deg2rad(config['pitch_deg']),0])
    center=fixed(name+'_camera_center',name+'_camera_link',[0,0,.015])
    for tag in ('visual','collision'):
        element=ET.SubElement(center,tag);ET.SubElement(element,'origin',xyz='-0.01 0 0',rpy='0 0 0')
        geometry=ET.SubElement(element,'geometry');ET.SubElement(geometry,'mesh',filename=str(Path(mesh_path).resolve()))
        if tag=='visual':
            material=ET.SubElement(element,'material',name=name+'_black');ET.SubElement(material,'color',rgba='.08 .08 .08 1')
    for side,y in [('left',.06),('right',-.06)]:
        frame=f'{name}_{side}_camera_frame'
        fixed(frame,name+'_camera_center',[-.005,y,0.])
        fixed(frame+'_optical',frame,rpy=[-np.pi/2,0,-np.pi/2])
    return audit


def main():
    """Generate a separate URDF; the source robot description is never modified."""
    import argparse
    import json
    import yaml
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--config',required=True,type=Path)
    parser.add_argument('--mesh',required=True,type=Path)
    args=parser.parse_args()
    if args.input.resolve()==args.output.resolve():parser.error('output must differ from the original URDF')
    config=yaml.safe_load(args.config.read_text())
    tree=ET.parse(args.input)
    audit=append_zed2i(tree.getroot(),config,args.mesh)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    tree.write(args.output,encoding='utf-8',xml_declaration=True)
    args.output.with_suffix('.mount.json').write_text(json.dumps(dict(config=config,audit=audit),indent=2))
    print(json.dumps(audit,indent=2))


if __name__=='__main__':main()
