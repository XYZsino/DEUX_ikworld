
"""Fit current ROS depth capture; no robot commands or camera TF changes."""
from pathlib import Path
import json,warnings
import numpy as np
from PIL import Image,ImageDraw
root=Path(__file__).resolve().parents[1]
out=root/Path((root/'.setup/latest-table-plane-path').read_text())
data=np.load(out/'capture.npz')
depths=data['depths'];P=data['projection'].reshape(3,4);bc=data['base_from_camera']
polygons=[[(388,174),(525,174),(535,251),(398,261)]]
mask=Image.new('L',(640,360));draw=ImageDraw.Draw(mask)
for polygon in polygons:draw.polygon(polygon,fill=1)
v,u=np.nonzero(np.array(mask));raw=depths[:,v,u]
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
q,ok=points(raw);n,d=fit(q);res=q@n+d;inliers=np.abs(res)<.005
a,_=points(raw[:75]);b,_=points(raw[75:])
na,da=fit(a);nb,db=fit(b)
angle=float(np.degrees(np.arccos(np.clip(na@nb,-1,1))))
center=q.mean(0);agreement=abs((na@center+da)-(nb@center+db))
report=dict(frame='base_link',normal=n.tolist(),offset_m=d,equation='normal dot point + offset = signed height above table',frames=len(depths),points=len(q),inlier_fraction=float(inliers.mean()),inlier_rmse_m=float(np.sqrt(np.mean(res[inliers]**2))),residual_abs_p95_m=float(np.quantile(abs(res),.95)),temporal_half_angle_deg=angle,temporal_half_offset_at_center_m=float(agreement),table_z_at_roi_center_m=float(-(n[0]*center[0]+n[1]*center[1]+d)/n[2]),roi_polygons=polygons)
report['accepted']=bool(len(q)>1000 and inliers.mean()>.85 and report['inlier_rmse_m']<.003 and angle<1 and agreement<.003)
(out/'plane.json').write_text(json.dumps(report,indent=2))
im=Image.open(out/'rgb.png').convert('RGB');draw=ImageDraw.Draw(im)
for poly in polygons:draw.line(poly+[poly[0]],fill='#00ff88',width=2)
draw.text((8,8),'Table plane: '+('ACCEPTED' if report['accepted'] else 'REVIEW'),fill='yellow')
im.save(out/'plane-roi.png')
print(json.dumps(report,indent=2))
