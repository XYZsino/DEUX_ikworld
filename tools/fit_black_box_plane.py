"""Fit the visually selected black-box interior bottom; no robot TF publication."""
from pathlib import Path
import json, warnings
import numpy as np
import cv2
r=Path(__file__).resolve().parents[1]
out=r/".setup/black-box-plane-90"
meta=json.loads((out/"capture.json").read_text())
frames=np.load(out/"depths.npz")["depths"]
poly=np.array([[643,419],[831,419],[873,508],[641,508]],np.int32)
mask=np.zeros(frames.shape[1:],np.uint8)
cv2.fillPoly(mask,[poly],1)
valid=np.isfinite(frames)&(frames>.3)&(frames<3)
clean=np.where(valid,frames,np.nan)
with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    depth=np.nanmedian(clean,axis=0)
    mad=np.nanmedian(np.abs(clean-depth),axis=0)
use=(mask>0)&(valid.sum(axis=0)>=72)&np.isfinite(depth)
v,u=np.nonzero(use)
z=depth[v,u]
points=np.column_stack(((u-meta["cx"])*z/meta["fx"],(v-meta["cy"])*z/meta["fy"],z))
if len(points)<300:raise RuntimeError("Insufficient valid black-bottom points")
rng=np.random.default_rng(42)
order=rng.permutation(len(points))
train=points[order[:len(order)//2]]
test=points[order[len(order)//2:]]
sample=train[rng.choice(len(train),min(len(train),5000),replace=False)]
best=None
threshold=.004
for i in range(800):
    a,b,c=sample[rng.choice(len(sample),3,replace=False)]
    n=np.cross(b-a,c-a)
    norm=np.linalg.norm(n)
    if norm<1e-9:continue
    n/=norm
    d=-n@a
    err=np.abs(sample@n+d)
    score=np.count_nonzero(err<threshold)
    if best is None or score>best[0]:best=(score,n,d)
_,n,d=best
for i in range(5):
    selected=train[np.abs(train@n+d)<threshold]
    center=selected.mean(axis=0)
    _,_,vh=np.linalg.svd(selected-center,full_matrices=False)
    n=vh[-1]
    d=-n@center
if d<0:n=-n;d=-d
residual=points@n+d
inside=np.abs(residual)<threshold
angles=[]
offsets=[]
frame_counts=[]
with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    temporal_groups=[np.nanmedian(group,axis=0) for group in np.array_split(clean,6)]
for f in temporal_groups:
    zz=f[v,u]
    ok=np.isfinite(zz)&inside
    pp=np.column_stack(((u[ok]-meta["cx"])*zz[ok]/meta["fx"],(v[ok]-meta["cy"])*zz[ok]/meta["fy"],zz[ok]))
    for _ in range(3):
        cen=pp.mean(axis=0)
        _,_,vh=np.linalg.svd(pp-cen,full_matrices=False)
        nn=vh[-1]
        if nn@n<0:nn=-nn
        dd=-nn@cen
        pp=pp[np.abs(pp@nn+dd)<.006]
    angles.append(float(np.rad2deg(np.arccos(np.clip(nn@n,-1,1)))))
    offsets.append(float(dd))
    frame_counts.append(len(pp))
x=np.array([1.,0.,0.])
x=x-n*(x@n)
x/=np.linalg.norm(x)
y=np.cross(n,x)
rotation=np.stack((x,y,n))
T=np.eye(4)
T[:3,:3]=rotation
T[2,3]=d
test_err=np.abs(test@n+d)
quality=dict(roi_pixels=int(mask.sum()),valid_stable_pixels=len(points),valid_roi_fraction=float(len(points)/mask.sum()),inlier_fraction=float(inside.mean()),inlier_rmse_m=float(np.sqrt(np.mean(residual[inside]**2))),held_out_inlier_fraction=float(np.mean(test_err<threshold)),held_out_abs_error_p95_m=float(np.percentile(test_err,95)),temporal_normal_max_deviation_deg=max(angles),temporal_offset_std_m=float(np.std(offsets)),median_pixel_depth_mad_m=float(np.median(mad[v,u])),per_15_frame_group_normal_deviation_deg=angles,per_15_frame_group_offsets_m=offsets)
accepted=quality["inlier_fraction"]>.85 and quality["held_out_abs_error_p95_m"]<.006 and max(angles)<1.0 and quality["valid_roi_fraction"]>.4
result=dict(schema="deux.plane_calibration.v1",surface="front black box interior bottom",camera_frame=meta["camera_frame"],camera_serial=meta["serial"],capture=meta,roi_polygon_uv=poly.tolist(),plane=dict(equation="normal_camera dot point_camera + offset_m = 0",normal_camera=n.tolist(),offset_m=float(d),normal_direction="toward camera, positive support height above box bottom"),quality=quality,plane_fit_accepted=accepted,T_reference_camera=T.tolist(),reference_frame="black_box_bottom",reference_definition="origin is camera perpendicular projection onto plane; +X is camera +X projected onto plane; +Z toward camera; +Y = +Z cross +X; in-plane heading is a convention, not measured robot yaw",robot_base_calibrated=False,box_bottom_height_from_floor_m=None,shoulder_axis_height_from_floor_m=1.34,base_height_from_floor_m=None,notes=["No robot-base TF has been inferred or published.","A plane supplies two rotational constraints; robot yaw remains from a separately established URDF/measured heading.","User measured floor-to-shoulder 1.340 m; base origin height remains unconfirmed.","Finite-depth and multiframe stability measure repeatability, not absolute calibration accuracy."])
(out/"plane.json").write_text(json.dumps(result,indent=2))
img=cv2.imread(str(out/"rgb.png"))
overlay=img.copy()
overlay[v[inside],u[inside]]=(0,200,0)
overlay[v[~inside],u[~inside]]=(0,0,255)
img=cv2.addWeighted(img,.55,overlay,.45,0)
cv2.polylines(img,[poly],True,(0,255,255),2)
lines=["BLACK BOX: interior bottom ROI",f"Valid {quality['valid_roi_fraction']:.1%}, inliers {quality['inlier_fraction']:.1%}",f"RMSE {quality['inlier_rmse_m']*1000:.2f} mm, plane distance {d:.4f} m",f"6 temporal groups (90 frames): normal max drift {max(angles):.3f} deg","Green: plane inliers | Yellow: selected ROI"]
for i,line in enumerate(lines):
    cv2.putText(img,line,(15,28+27*i),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,0),4,cv2.LINE_AA)
    cv2.putText(img,line,(15,28+27*i),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1,cv2.LINE_AA)
cv2.imwrite(str(out/"plane-overlay.jpg"),img)
if accepted:
    (r/"calibration").mkdir(exist_ok=True)
    (r/"calibration/black_box_plane.thor.json").write_text(json.dumps(result,indent=2))
print(json.dumps(dict(plane=result["plane"],quality=quality,accepted=accepted),indent=2))
if not accepted:raise RuntimeError("Plane quality gate failed; do not use calibration")
