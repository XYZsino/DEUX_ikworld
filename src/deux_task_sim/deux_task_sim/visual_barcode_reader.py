"""Virtual optical reader bound to the actual printed barcode on the mesh.

Uses simulator body transforms only inside this sensor emulator, emitting raw
fixture text, never poses to control. The fixture value is not image decoding.
"""
from pathlib import Path
import numpy as np

# Same atlas rectangle used for offline barcode annotation; visible bars only.
BARCODE_ATLAS_RECT=(.064,.695,.323,.79)
x0,y0,x1,y1=BARCODE_ATLAS_RECT
BARCODE_UV_CENTER=np.array([(x0+x1)/2,1-(y0+y1)/2])
FIXTURE_VALUE='2087231250396'  # Printed EAN-13 on breadbag_albedo.png (checksum valid).


def atlas_surfaces(mesh_path,center_offset,uv=BARCODE_UV_CENTER):
    vertices=[];textures=[];normals=[];faces=[]
    for line in Path(mesh_path).read_text().splitlines():
        bits=line.split()
        if not bits:continue
        if bits[0]=='v':vertices.append(list(map(float,bits[1:4])))
        elif bits[0]=='vt':textures.append(list(map(float,bits[1:3])))
        elif bits[0]=='vn':normals.append(list(map(float,bits[1:4])))
        elif bits[0]=='f':
            indices=[tuple(int(x)-1 for x in b.split('/')) for b in bits[1:]]
            for i in range(1,len(indices)-1):faces.append([indices[0],indices[i],indices[i+1]])
    vertices=np.asarray(vertices);textures=np.asarray(textures);normals=np.asarray(normals);found=[]
    for face in faces:
        v=vertices[[f[0] for f in face]];t=textures[[f[1] for f in face]]
        basis=(t[1:]-t[0]).T
        if abs(np.linalg.det(basis))<1e-12:continue
        ab=np.linalg.solve(basis,np.asarray(uv)-t[0]);weights=np.r_[1-ab.sum(),ab]
        if min(weights)<-1e-8:continue
        point=weights@v-np.asarray(center_offset)
        if any(np.linalg.norm(point-s['point'])<1e-5 for s in found):continue
        normal=weights@normals[[f[2] for f in face]];normal/=np.linalg.norm(normal)
        horizontal=((v[1:]-v[0]).T@np.linalg.inv(basis))[:,0]
        horizontal-=normal*np.dot(horizontal,normal);horizontal/=np.linalg.norm(horizontal)
        found.append(dict(point=point,normal=normal,horizontal=horizontal))
    if len(found)!=2:raise ValueError(f'expected front/back printed barcode surfaces, got {len(found)}')
    return found


def metrics(scanner,body,surface,profile):
    point=body[:3,:3]@surface['point']+body[:3,3]
    normal=body[:3,:3]@surface['normal'];horizontal=body[:3,:3]@surface['horizontal'];vertical=np.cross(normal,horizontal)
    delta=point-scanner[:3,3];distance=float(np.linalg.norm(delta));direction=delta/max(distance,1e-9)
    aim=float(np.rad2deg(np.arccos(np.clip(direction@-scanner[:3,2],-1,1))))
    toward=-direction;front=float(toward@normal)
    pitch=float(np.rad2deg(np.arctan2(toward@vertical,front)));skew=float(np.rad2deg(np.arctan2(toward@horizontal,front)))
    low,high=profile['distance_m']
    aligned=low<=distance<=high and front>0 and aim<profile['aim_error_deg_less_than'] and abs(pitch)<=profile['pitch_abs_max_deg'] and abs(skew)<=profile['skew_abs_max_deg']
    return dict(distance_m=distance,aim_error_deg=aim,pitch_deg=pitch,skew_deg=skew,aligned=bool(aligned))


class VisualBarcodeSource:
    def __init__(self,sim):
        from deux_conveyor import BREAD_ASSET,BREAD_MODEL
        from reader_pick import SCAN_PROFILE
        self.sim=sim;self.since={};self.profile=SCAN_PROFILE.copy()
        # Group 3 contains invisible collision envelopes; optical visibility uses
        # the rendered mesh, including the barcode decal (group 2).
        self.ray_groups=np.ones(6,dtype=np.uint8);self.ray_groups[3]=0
        self.surfaces=atlas_surfaces(BREAD_ASSET/BREAD_MODEL['mesh'],BREAD_MODEL['center_offset_m'])

    def capture(self):
        import mujoco
        from transfer_sim_adapter import transform,BarcodeReading
        sim=self.sim;now=float(sim.data.time);sid=sim.model.site('left_fixed_reader_scan_origin').id
        scanner=np.eye(4);scanner[:3,:3]=sim.data.site_xmat[sid].reshape(3,3);scanner[:3,3]=sim.data.site_xpos[sid]
        readings=[]
        for name,_ in sim.objects:
            body=transform(sim.data,name);visible=False
            for surface in self.surfaces:
                if not metrics(scanner,body,surface,self.profile)['aligned']:continue
                point=body[:3,:3]@surface['point']+body[:3,3];delta=point-scanner[:3,3];distance=np.linalg.norm(delta)
                geom=np.array([-1],dtype=np.int32)
                hit=mujoco.mj_ray(sim.model,sim.data,scanner[:3,3],delta/distance,self.ray_groups,1,int(sim.model.site_bodyid[sid]),geom)
                if hit>=0 and int(sim.model.geom_bodyid[geom[0]])==sim.model.body(name).id and abs(hit-distance)<=.002:
                    visible=True;break
            if not visible:self.since.pop(name,None);continue
            self.since.setdefault(name,now)
            if now-self.since[name]>=self.profile['continuous_alignment_s']:
                readings.append(BarcodeReading(now,FIXTURE_VALUE,name,simulated=True))
        return tuple(readings)
