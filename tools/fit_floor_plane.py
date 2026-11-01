"""Robust multi-frame black-box plane estimate with temporal/spatial validation."""
from pathlib import Path
import json,warnings
import numpy as np
import cv2
r=Path(__file__).resolve().parents[1]
out=r/".setup/floor-plane-450"
meta=json.loads((out/"capture.json").read_text())
data=np.load(out/"depths-confidence.npz")
frames=data["depths"];confidence=data["confidence"]
poly=np.array([[540,510],[920,510],[950,610],[890,700],[430,700],[465,565]],np.int32)
mask=np.zeros(frames.shape[1:],np.uint8)
cv2.fillPoly(mask,[poly-[400,480]],1)
vv,uu=np.nonzero(mask)
u=uu+400;v=vv+480
raw=frames[:,vv,uu];conf=confidence[:,vv,uu]
rng=np.random.default_rng(20261029)
warnings.filterwarnings("ignore",category=RuntimeWarning)
valid=np.isfinite(raw)&(raw>.3)&(raw<3)
confidence_stats={str(t):float(np.mean(valid&(conf<=t))) for t in (30,50,70,90,95)}
def aggregate(z,c,limit=70):
    a=np.where(np.isfinite(z)&(z>.3)&(z<3)&(c<=limit),z,np.nan)
    med=np.nanmedian(a,axis=0)
    mad=np.nanmedian(np.abs(a-med),axis=0)
    temporal=np.abs(a-med)<=np.maximum(.003,3*1.4826*mad)
    a=np.where(temporal,a,np.nan)
    med=np.nanmedian(a,axis=0)
    good=(np.isfinite(a).sum(axis=0)>=len(z)*.6)&np.isfinite(med)&(mad<=.006)
    return med,good,mad,a
def xyz(z,good):
    return np.column_stack(((u[good]-meta["cx"])*z[good]/meta["fx"],(v[good]-meta["cy"])*z[good]/meta["fy"],z[good]))
def fit(points,iterations=800):
    if len(points)<100:raise ValueError("too few reliable plane points")
    sample=points[rng.choice(len(points),min(3000,len(points)),replace=False)]
    best=(-1,None,None)
    for i in range(iterations):
        a,b,c=sample[rng.choice(len(sample),3,replace=False)]
        n=np.cross(b-a,c-a);norm=np.linalg.norm(n)
        if norm<1e-10:continue
        n/=norm;d=-n@a
        count=np.count_nonzero(np.abs(sample@n+d)<.003)
        if count>best[0]:best=(count,n,d)
    _,n,d=best
    for i in range(15):
        residual=points@n+d
        sigma=max(.00035,1.4826*np.median(np.abs(residual-np.median(residual))))
        cutoff=min(.006,max(.002,4.685*sigma))
        scaled=residual/cutoff
        w=np.where(np.abs(scaled)<1,(1-scaled**2)**2,0)
        if w.sum()<50:raise ValueError("too few inliers")
        center=(points*w[:,None]).sum(axis=0)/w.sum()
        delta=points-center
        _,vec=np.linalg.eigh((delta*w[:,None]).T@delta)
        n=vec[:,0];d=-n@center
    if d<0:n=-n;d=-d
    return n,float(d)
z,good,mad,filtered=aggregate(raw,conf)
points=xyz(z,good)
n,d=fit(points)
err=points@n+d
inliers=np.abs(err)<.004
angle=lambda a,b:float(np.rad2deg(np.arccos(np.clip(a@b,-1,1))))
groups=[]
for indices in np.array_split(np.arange(len(raw)),6):
    zg,gg,_,_=aggregate(raw[indices],conf[indices])
    pg=xyz(zg,gg)
    ng,dg=fit(pg)
    groups.append(dict(frames=len(indices),points=len(pg),normal=ng.tolist(),offset_m=dg,normal_difference_deg=angle(n,ng)))
sensitivity=[]
for limit in (50,70,90):
    zs,gs,_,_=aggregate(raw,conf,limit)
    if gs.sum()<100:continue
    ns,ds=fit(xyz(zs,gs))
    sensitivity.append(dict(confidence_limit=limit,points=int(gs.sum()),normal=ns.tolist(),offset_m=ds,normal_difference_deg=angle(n,ns)))
spatial=[]
for name,sel in [("left",u<np.median(u)),("right",u>=np.median(u)),("far",v<np.median(v)),("near",v>=np.median(v))]:
    sg=good&sel
    if sg.sum()<100:continue
    ns,ds=fit(xyz(z,sg))
    spatial.append(dict(region=name,points=int(sg.sum()),normal=ns.tolist(),offset_m=ds,normal_difference_deg=angle(n,ns)))
# Spatial block bootstrap: correlated pixels are represented by 8x8 pixel tiles.
pu=u[good][inliers];pv=v[good][inliers];pin=points[inliers]
tile=(pv//8)*200+(pu//8)
blocks=np.array([np.median(pin[tile==i],axis=0) for i in np.unique(tile)])
boot=[]
for i in range(500):
    b=blocks[rng.integers(0,len(blocks),len(blocks))]
    center=b.mean(axis=0)
    _,_,vh=np.linalg.svd(b-center,full_matrices=False)
    nb=vh[-1];db=-nb@center
    if db<0:nb=-nb;db=-db
    boot.append([*nb,float(db),angle(n,nb)])
boot=np.array(boot)
holdout=np.arange(len(points))%2==0
nh,dh=fit(points[~holdout])
he=np.abs(points[holdout]@nh+dh)
x=np.array([1.,0.,0.]);x-=n*(x@n);x/=np.linalg.norm(x)
R=np.stack((x,np.cross(n,x),n))
T=np.eye(4);T[:3,:3]=R;T[2,3]=d
quality=dict(captured_frames=len(raw),capture_duration_s=(meta["timestamps_ns"][-1]-meta["timestamps_ns"][0])/1e9,roi_pixels=len(u),confidence_valid_fraction=confidence_stats,retained_pixels=int(good.sum()),retained_pixel_fraction=float(good.mean()),temporal_observations_retained=int(np.isfinite(filtered[:,good]).sum()),plane_inlier_pixels=int(inliers.sum()),inlier_fraction=float(inliers.mean()),inlier_rmse_m=float(np.sqrt(np.mean(err[inliers]**2))),all_point_residual_p95_m=float(np.percentile(np.abs(err),95)),held_out_inlier_fraction=float(np.mean(he<.004)),held_out_all_residual_p95_m=float(np.percentile(he,95)),temporal_group_normal_max_deviation_deg=max(g["normal_difference_deg"] for g in groups),temporal_group_offset_std_m=float(np.std([g["offset_m"] for g in groups])),spatial_half_max_normal_difference_deg=max(s["normal_difference_deg"] for s in spatial),confidence_sensitivity_max_angle_deg=max(s["normal_difference_deg"] for s in sensitivity),bootstrap_spatial_blocks=len(blocks),conditional_block_bootstrap_normal_deviation_p95_deg=float(np.percentile(boot[:,-1],95)),conditional_block_bootstrap_offset_95_interval_m=np.percentile(boot[:,3],[2.5,97.5]).tolist(),temporal_groups=groups,spatial_halves=spatial,confidence_sensitivity=sensitivity)
accepted=(quality["inlier_fraction"]>.85 and quality["retained_pixel_fraction"]>.4 and quality["temporal_group_normal_max_deviation_deg"]<1.0 and quality["spatial_half_max_normal_difference_deg"]<1.0 and quality["confidence_sensitivity_max_angle_deg"]<1.0)
result=dict(schema="deux.plane_calibration.v1",surface="room floor",camera_frame=meta["camera_frame"],camera_serial=meta["serial"],capture=meta,roi_polygon_uv=poly.tolist(),plane=dict(equation="normal_camera dot point_camera + offset_m = 0",normal_camera=n.tolist(),offset_m=d,normal_direction="toward camera"),quality=quality,plane_fit_accepted=bool(accepted),T_reference_camera=T.tolist(),reference_frame="floor_surface",reference_definition="Origin is perpendicular projection of camera on plane; +X is camera +X projected onto plane; +Z normal toward camera; +Y = +Z cross +X. Heading convention is not a robot yaw measurement.",robot_base_calibrated=False,floor_height_from_floor_m=0.0,shoulder_axis_height_from_floor_m=1.34,base_height_from_floor_m=None,method=dict(confidence_max=70,min_temporal_coverage=.6,temporal_outlier_gate="max(3mm, 3*1.4826*MAD)",max_temporal_depth_mad_m=.006,spatial_fit="RANSAC 3mm followed by Tukey IRLS",reported_spatial_inlier_threshold_m=.004),notes=["Bootstrap interval is conditional repeatability, not absolute calibration accuracy.","Spatial disagreement can indicate systematic stereo bias or a nonplanar bottom; collecting more frames cannot guarantee its removal.","A plane supplies two rotational constraints only; robot yaw and box-floor elevation remain unmeasured.","No robot TF published or previous calibration overwritten."])
(out/"plane.json").write_text(json.dumps(result,indent=2))
(r/"calibration").mkdir(exist_ok=True)
name="floor_plane.thor.json" if accepted else "floor_plane.thor.provisional.json"
(r/"calibration"/name).write_text(json.dumps(result,indent=2))
img=cv2.imread(str(out/"rgb-225.png"));over=img.copy()
gu=u[good];gv=v[good]
over[gv[inliers],gu[inliers]]=(0,220,0);over[gv[~inliers],gu[~inliers]]=(0,0,255)
img=cv2.addWeighted(img,.55,over,.45,0);cv2.polylines(img,[poly],True,(0,255,255),2)
lines=["Black box interior bottom | 450 frames / 60 seconds",f"Retained {good.sum()} pixels | plane inliers {inliers.mean():.1%}",f"Fit RMSE {quality['inlier_rmse_m']*1000:.2f} mm | distance {d:.4f} m",f"Temporal normal max {quality['temporal_group_normal_max_deviation_deg']:.2f} deg | spatial halves max {quality['spatial_half_max_normal_difference_deg']:.2f} deg",f"Status: {'ACCEPTED' if accepted else 'PROVISIONAL - systematic consistency check failed'}"]
for i,line in enumerate(lines):
    cv2.putText(img,line,(15,28+28*i),cv2.FONT_HERSHEY_SIMPLEX,.63,(0,0,0),4,cv2.LINE_AA)
    cv2.putText(img,line,(15,28+28*i),cv2.FONT_HERSHEY_SIMPLEX,.63,(255,255,255),1,cv2.LINE_AA)
cv2.imwrite(str(out/"plane-overlay.jpg"),img)
print(json.dumps(dict(plane=result["plane"],quality=quality,accepted=accepted,path=str(r/"calibration"/name)),indent=2))
