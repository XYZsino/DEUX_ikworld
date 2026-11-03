from pathlib import Path
import json,warnings
import numpy as np
from PIL import Image,ImageDraw
out=Path('.setup/box-plane-current');data=np.load(out/'capture.npz');depths=data['depths'];P=data['projection'].reshape(3,4);bc=data['base_from_camera']
polygons=[[(190,218),(240,220),(236,243),(178,240)],[(332,228),(383,230),(391,254),(327,251)]]
def points(a):
    with warnings.catch_warnings():
        warnings.simplefilter('ignore',RuntimeWarning)
        a=np.where(np.isfinite(a)&(a>.3)&(a<2),a,np.nan)
        med=np.nanmedian(a,axis=0);mad=np.nanmedian(np.abs(a-med),axis=0)
        keep=np.abs(a-med)<=np.maximum(.003,3*1.4826*mad)
        z=np.nanmedian(np.where(keep,a,np.nan),axis=0)
    ok=(keep.sum(axis=0)>=len(a)*.6)&np.isfinite(z)&(mad<.006)
    z=z[ok];q=np.column_stack(((u[ok]-P[0,2])*z/P[0,0],(v[ok]-P[1,2])*z/P[1,1],z))
    return q@bc[:3,:3].T+bc[:3,3],ok
def fit(q):
    rng=np.random.default_rng(20261030);s=q[rng.choice(len(q),min(3500,len(q)),replace=False)]
    best=None
    for _ in range(700):
        a,b,c=s[rng.choice(len(s),3,replace=False)];n=np.cross(b-a,c-a)
        if np.linalg.norm(n)<1e-9:continue
        n/=np.linalg.norm(n)
        if n[2]<0:n=-n
        if n[2]<np.cos(np.deg2rad(20)):continue
        d=-n@a;count=np.count_nonzero(np.abs(s@n+d)<.005)
        if best is None or count>best[0]:best=(count,n,d)
    if best is None:raise ValueError('No horizontal table plane')
    _,n,d=best
    for _ in range(8):
        r=q@n+d;sel=np.abs(r)<.005
        center=q[sel].mean(0);_,_,vh=np.linalg.svd(q[sel]-center,full_matrices=False)
        n=vh[-1]
        if n[2]<0:n=-n
        d=-n@center
    return n,float(d)

clouds=[];per_box=[];all_uv=[]
for poly in polygons:
 mask=Image.new('L',(640,360));ImageDraw.Draw(mask).polygon(poly,fill=1);v,u=np.nonzero(np.array(mask));all_uv.append((v,u));raw=depths[:,v,u];q,ok=points(raw);n,d=fit(q);clouds.append(q);res=q@n+d
 per_box.append(dict(points=len(q),normal=n.tolist(),offset_m=d,inlier_fraction=float(np.mean(abs(res)<.005)),rmse_mm=float(np.sqrt(np.mean(res[abs(res)<.005]**2))*1000),centroid=q.mean(0).tolist()))
q=np.vstack(clouds);n,d=fit(q);res=q@n+d;sel=abs(res)<.005;half=[]
for sl in (slice(0,75),slice(75,None)):
 combined=[]
 for v,u in all_uv:combined.append(points(depths[sl,v,u])[0])
 nh,dh=fit(np.vstack(combined));half.append((nh,dh))
center=q.mean(0);angle=float(np.degrees(np.arccos(np.clip(half[0][0]@half[1][0],-1,1))));offset=abs(half[0][0]@center+half[0][1]-half[1][0]@center-half[1][1])
report=dict(frame='base_link',normal=n.tolist(),offset_m=d,frames=len(depths),points=len(q),inlier_fraction=float(sel.mean()),rmse_mm=float(np.sqrt(np.mean(res[sel]**2))*1000),temporal_angle_deg=angle,temporal_offset_mm=float(offset*1000),plane_z_at_center_m=float(-(n[0]*center[0]+n[1]*center[1]+d)/n[2]),box_residual_medians_mm=[float(np.median(c@n+d)*1000) for c in clouds],boxes=per_box,roi_polygons=polygons,clearance_m=.07)
report['accepted']=bool(sel.mean()>.8 and report['rmse_mm']<5 and angle<1 and offset<.005 and max(abs(x) for x in report['box_residual_medians_mm'])<5)
(out/'plane.json').write_text(json.dumps(report,indent=2));im=Image.open(out/'rgb.png');dr=ImageDraw.Draw(im)
for i,p in enumerate(polygons):dr.line(p+[p[0]],fill='#00ff80',width=2);dr.text(p[0],str(i+1),fill='yellow')
im.save(out/'plane-rois.png');print(json.dumps(report,indent=2))
