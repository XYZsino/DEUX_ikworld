"""Validate RANSAC support selected only from first half on held-out second half."""
from pathlib import Path
import numpy as np,json,cv2,warnings
r=Path(__file__).resolve().parents[1];out=r/".setup/floor-plane-450"
# Reuse numerical routines without executing the full-report section.
source=(r/"tools/fit_floor_plane.py").read_text()
exec(source.split("z,good,mad,filtered=aggregate(raw,conf)")[0])
za,ga,_,_=aggregate(raw[:225],conf[:225])
na,da=fit(xyz(za,ga))
allpa=xyz(za,np.ones_like(ga))
support=ga&(np.abs(allpa@na+da)<.003)
image_mask=np.zeros(frames.shape[1:],np.uint8);image_mask[vv[support],uu[support]]=1
image_mask=cv2.erode(image_mask,np.ones((7,7),np.uint8))
support=image_mask[vv,uu]>0
if support.sum()<3000:raise RuntimeError("Insufficient independent support")
zb,gb,_,_=aggregate(raw[225:],conf[225:])
pb=xyz(zb,support&gb)
nb,db=fit(pb)
z,g,mad,filtered=aggregate(raw,conf)
selected=support&g
points=xyz(z,selected);n,d=fit(points)
err=points@n+d;inliers=np.abs(err)<.004
angle=lambda a,b:float(np.rad2deg(np.arccos(np.clip(a@b,-1,1))))
pgtrain=xyz(za,support&ga)
nt,dt=fit(pgtrain)
testres=pb@nt+dt
groups=[]
for indices in np.array_split(np.arange(450),6):
    zg,gg,_,_=aggregate(raw[indices],conf[indices])
    ng,dg=fit(xyz(zg,gg&support))
    groups.append(dict(start_frame=int(indices[0]),end_frame=int(indices[-1]),normal=ng.tolist(),offset_m=dg,difference_deg=angle(n,ng)))
spatial=[]
for name,sel in [("left",u<np.median(u[selected])),("right",u>=np.median(u[selected])),("far",v<np.median(v[selected])),("near",v>=np.median(v[selected]))]:
    ns,ds=fit(xyz(z,selected&sel))
    spatial.append(dict(region=name,normal=ns.tolist(),offset_m=ds,difference_deg=angle(n,ns)))
T=np.eye(4);x=np.array([1.,0.,0.]);x-=n*(x@n);x/=np.linalg.norm(x)
T[:3,:3]=np.stack((x,np.cross(n,x),n));T[2,3]=d
q=dict(total_frames=450,original_roi_pixels=len(u),training_selected_support_pixels=int(support.sum()),final_support_pixels=int(selected.sum()),final_inlier_pixels=int(inliers.sum()),final_inlier_fraction=float(inliers.mean()),inlier_rmse_m=float(np.sqrt(np.mean(err[inliers]**2))),held_out_225_frames_inlier_fraction=float(np.mean(np.abs(testres)<.004)),held_out_225_frames_residual_p95_m=float(np.percentile(np.abs(testres),95)),train_test_normal_difference_deg=angle(nt,nb),train_test_offset_difference_m=abs(dt-db),temporal_normal_max_difference_deg=max(a["difference_deg"] for a in groups),spatial_half_max_difference_deg=max(a["difference_deg"] for a in spatial),temporal_groups=groups,spatial_halves=spatial)
accepted=q["held_out_225_frames_inlier_fraction"]>.95 and q["train_test_normal_difference_deg"]<1 and q["temporal_normal_max_difference_deg"]<1 and q["spatial_half_max_difference_deg"]<1
res=dict(schema="deux.plane_calibration.v1",surface="room floor",camera_frame=meta["camera_frame"],camera_serial=meta["serial"],capture=meta,plane=dict(normal_camera=n.tolist(),offset_m=d,equation="normal_camera dot point_camera + offset_m = 0"),quality=q,plane_fit_accepted=bool(accepted),T_reference_camera=T.tolist(),reference_frame="floor_surface",reference_definition="Camera perpendicular projection origin; X camera-right projected on plane; Z plane normal toward camera. Not robot heading.",robot_base_calibrated=False,floor_height_from_floor_m=0.0,shoulder_axis_height_from_floor_m=1.34,base_height_from_floor_m=None,method="Pixel temporal MAD filtering, RANSAC + Tukey IRLS; support selected ONLY on first 225 frames with 3mm residual and eroded 3 pixels. Fixed support independently validated against last 225 frames; final fit uses all 450.",notes=["Excluded spatial regions can be stereo edge bias, floor nonplanarity or reflection; cause not determined.","Subset-fit residual is not absolute accuracy.","No robot TF published; robot base-floor offset and yaw unmeasured."])
(out/"plane-core.json").write_text(json.dumps(res,indent=2))
np.save(out/"selected-core-mask.npy",image_mask)
name="floor_plane.thor.json" if accepted else "floor_plane.thor.provisional.json"
(r/"calibration"/name).write_text(json.dumps(res,indent=2))
img=cv2.imread(str(out/"rgb-225.png"));over=img.copy()
over[v[selected],u[selected]]=(0,200,0)
over[v[selected][~inliers],u[selected][~inliers]]=(0,0,255)
img=cv2.addWeighted(img,.55,over,.45,0);cv2.polylines(img,[poly],True,(0,255,255),2)
lines=["Room floor | 450 frames | robust support",f"RANSAC support: {selected.sum()} pixels, inliers {inliers.mean():.1%}",f"Fit RMSE {q['inlier_rmse_m']*1000:.2f} mm | distance {d:.4f} m",f"Independent 225-frame normal difference: {q['train_test_normal_difference_deg']:.3f} deg",f"Spatial halves max difference: {q['spatial_half_max_difference_deg']:.3f} deg | {'ACCEPTED' if accepted else 'PROVISIONAL'}"]
for i,line in enumerate(lines):
    cv2.putText(img,line,(15,28+28*i),cv2.FONT_HERSHEY_SIMPLEX,.63,(0,0,0),4,cv2.LINE_AA)
    cv2.putText(img,line,(15,28+28*i),cv2.FONT_HERSHEY_SIMPLEX,.63,(255,255,255),1,cv2.LINE_AA)
cv2.imwrite(str(out/"plane-core-overlay.jpg"),img)
print(json.dumps(res,indent=2))
