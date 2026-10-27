"""Offline-only synthetic labels. Simulator truth never enters online perception.

Bag labels use visible render segmentation. Barcode labels use a second render
of the exact barcode UV region on the existing asset, intersected with that
bag's visible pixels. Occluded barcode faces therefore produce no label.
"""
import argparse
import json
import os
from pathlib import Path
os.environ.setdefault('MUJOCO_GL', 'egl')
import cv2
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from .world import load_sim, initialize_urdf_zero
from .rgbd import HeadRgbd
from .ready_pose import HOME_JOINTS


def bbox(mask, minimum):
    y,x=np.where(mask)
    if len(x)<minimum or x.max()-x.min()<3 or y.max()-y.min()<3: return None
    return [int(x.min()),int(y.min()),int(x.max()+1),int(y.max()+1)]


def generate(repository, output, counts=(900,180,180), seed=4100):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    sim=load_sim(repository, output/'fixture')
    initialize_urdf_zero(sim)
    from deux_task_core.model import JOINT_NAMES
    from deux_conveyor import BREAD_ASSET
    # Segmentation IDs must not be blended by MSAA at object boundaries.
    sim.model.vis.quality.offsamples=0
    sensor=HeadRgbd(sim)
    model,data=sim.model,sim.data
    texid=model.texture('breadbag_albedo').id
    adr=int(model.tex_adr[texid]);h=int(model.tex_height[texid]);w=int(model.tex_width[texid]);ch=int(model.tex_nchannel[texid])
    tex=model.tex_data[adr:adr+h*w*ch].reshape(h,w,ch)
    original=tex.copy()
    artwork=cv2.cvtColor(cv2.imread(str(BREAD_ASSET/'mesh/breadbag_albedo.png')),cv2.COLOR_BGR2RGB)
    artwork=cv2.resize(artwork,(w,h))
    # Infer actual loaded row orientation instead of assuming OpenGL texture conventions.
    flip=np.mean(abs(original[:,:,:3].astype(float)-artwork[::-1]))<np.mean(abs(original[:,:,:3].astype(float)-artwork))
    from .visual_barcode_reader import BARCODE_ATLAS_RECT
    x0,y0,x1,y1=BARCODE_ATLAS_RECT
    patch=np.zeros_like(original);patch[int(h*y0):int(h*y1),int(w*x0):int(w*x1),1]=255
    if ch==4:patch[:,:,3]=255
    if flip:patch=patch[::-1].copy()
    initial=data.qpos.copy()
    joints=[int(model.joint(n).qposadr[0]) for n in JOINT_NAMES]
    names=[n for n,_ in sim.objects]
    indices={n:int(model.joint(n+'_free').qposadr[0]) for n in names}
    geoms={n:np.flatnonzero(model.geom_bodyid==model.body(n).id) for n in names}
    diffuse=model.light_diffuse.copy();ambient=model.light_ambient.copy()
    manifest=[]
    try:
        for split,count in zip(('train','val','test'),counts):
            (output/'images'/split).mkdir(parents=True,exist_ok=True)
            (output/'labels'/split).mkdir(parents=True,exist_ok=True)
            (output/'segmentation'/'labels'/split).mkdir(parents=True,exist_ok=True)
            for i in range(count):
                scene_seed=seed+{'train':0,'val':100000,'test':200000}[split]+i
                rng=np.random.default_rng(scene_seed)
                data.qpos[:]=initial
                data.qpos[joints]=np.asarray(HOME_JOINTS)*rng.uniform(0,1)
                # Independent scene randomization, not adjacent frames split across sets.
                for n,a in indices.items():
                    data.qpos[a:a+2]+=rng.uniform(-.085,.085,2)
                    data.qpos[a+2]+=rng.uniform(-.005,.025)
                    r=Rotation.from_euler('xyz',[rng.uniform(-.15,.15),rng.uniform(-.15,.15),rng.uniform(-np.pi,np.pi)])
                    q=r.as_quat();data.qpos[a+3:a+7]=q[[3,0,1,2]]
                if i%3==0:
                    a=indices['bread_3']
                    data.qpos[a:a+3]=[rng.uniform(-.65,-.38),rng.uniform(-.28,.22),rng.uniform(.04,.23)]
                    q=Rotation.from_euler('xyz',rng.uniform([-.6,-.6,-np.pi],[.6,.6,np.pi])).as_quat()
                    data.qpos[a+3:a+7]=q[[3,0,1,2]]
                if i%13==0:
                    for a in indices.values():data.qpos[a+2]=-3.  # True negative images.
                model.light_diffuse[:]=diffuse*rng.uniform(.6,1.3)
                model.light_ambient[:]=ambient*rng.uniform(.7,1.5)
                mujoco.mj_forward(model,data)
                sensor.renderer.update_scene(data,camera=sensor.camera)
                sensor.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION]=False
                rgb=sensor.renderer.render().copy()
                sensor.renderer.enable_segmentation_rendering()
                segments=sensor.renderer.render().copy()
                sensor.renderer.disable_segmentation_rendering()
                tex[:]=patch
                mujoco.mjr_uploadTexture(model,sensor.renderer._mjr_context,texid)
                colored=sensor.renderer.render().copy()
                tex[:]=original
                mujoco.mjr_uploadTexture(model,sensor.renderer._mjr_context,texid)
                green=(colored[:,:,1].astype(float)>colored[:,:,0]+35.) & (colored[:,:,1].astype(float)>colored[:,:,2]+35.)
                annotations=[];polygons=[]
                for n in names:
                    mask=np.isin(segments[:,:,0],geoms[n]) & (segments[:,:,1]==int(mujoco.mjtObj.mjOBJ_GEOM))
                    for cls,mask_,minimum in ((0,mask,50),(1,mask&green,8)):
                        box=bbox(mask_,minimum)
                        if box is not None:
                            annotations.append((cls,box,n))
                            contours,_=cv2.findContours(mask_.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                            contour=max(contours,key=cv2.contourArea).reshape(-1,2)
                            if len(contour)>=3:
                                contour=contour.astype(float)/[640,480]
                                polygons.append(str(cls)+' '+' '.join(f'{v:.7f}' for v in contour.ravel()))
                stem=f'{scene_seed:07d}'
                cv2.imwrite(str(output/'images'/split/(stem+'.jpg')),rgb[:,:,::-1],[cv2.IMWRITE_JPEG_QUALITY,94])
                lines=[]
                for cls,(x1,y1,x2,y2),_ in annotations:
                    lines.append(f'{cls} {(x1+x2)/1280:.7f} {(y1+y2)/960:.7f} {(x2-x1)/640:.7f} {(y2-y1)/480:.7f}')
                (output/'labels'/split/(stem+'.txt')).write_text('\n'.join(lines)+'\n')
                (output/'segmentation'/'labels'/split/(stem+'.txt')).write_text('\n'.join(polygons)+'\n')
                manifest.append(dict(split=split,seed=scene_seed,boxes=annotations))
                if i%25==0: print(split,i,'labels',len(annotations),flush=True)
                if i<5:
                    preview=rgb[:,:,::-1].copy()
                    for cls,box,_ in annotations:
                        cv2.rectangle(preview,tuple(box[:2]),tuple(box[2:]),(0,255,0) if cls else (255,180,0),2)
                    cv2.imwrite(str(output/f'preview-{split}-{i}.jpg'),preview)
        (output/'manifest.json').write_text(json.dumps(dict(seed=seed,texture_flip=bool(flip),
            barcode_atlas_xyxy=[.064,.695,.323,.79],head_camera='zed2i_left',camera_mount=json.loads((output/'fixture/camera-mount.json').read_text()),scenes=manifest),indent=2))
        (output/'dataset.yaml').write_text(f'path: {output}\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: bread_bag\n  1: barcode\n')
        for split in ('train','val','test'):
            destination=output/'segmentation'/'images'/split
            destination.mkdir(parents=True,exist_ok=True)
            for source in (output/'images'/split).glob('*.jpg'):
                os.link(source,destination/source.name)
        (output/'segmentation'/'dataset.yaml').write_text(f'path: {output}/segmentation\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: bread_bag\n  1: barcode\n')
    finally:
        tex[:]=original;sensor.close();sim.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sim-repository',required=True);p.add_argument('--output',required=True)
    p.add_argument('--counts',type=int,nargs=3,default=[900,180,180]);p.add_argument('--seed',type=int,default=4100)
    a=p.parse_args();generate(a.sim_repository,a.output,a.counts,a.seed)

if __name__=='__main__':main()
