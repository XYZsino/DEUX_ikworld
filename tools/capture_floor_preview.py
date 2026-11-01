"""Capture 450 depth/confidence ROI frames over 60 seconds; no robot control."""
from pathlib import Path
import json,time
import numpy as np
import pyzed.sl as sl
out=Path(__file__).resolve().parents[1]/".setup/floor-preview"
out.mkdir(parents=True,exist_ok=True)
z=sl.Camera();p=sl.InitParameters()
p.camera_resolution=sl.RESOLUTION.HD720;p.camera_fps=15
p.depth_mode=sl.DEPTH_MODE.ULTRA;p.coordinate_units=sl.UNIT.METER
p.coordinate_system=sl.COORDINATE_SYSTEM.IMAGE
p.depth_minimum_distance=.3;p.depth_maximum_distance=3.;p.depth_stabilization=0
e=z.open(p)
if e!=sl.ERROR_CODE.SUCCESS:raise RuntimeError(str(e))
try:
    info=z.get_camera_information();c=info.camera_configuration.calibration_parameters.left_cam
    meta=dict(serial=info.serial_number,sdk=z.get_sdk_version(),camera_frame="zed2i_left_camera_optical_frame",width=1280,height=720,fx=c.fx,fy=c.fy,cx=c.cx,cy=c.cy,coordinate_system="IMAGE: x right, y down, z forward",units="metres",depth_mode="ULTRA",shoulder_axis_height_from_floor_m=1.34,base_height_from_floor_m=None,timestamps_ns=[],crop_uvwh=[600,350,380,230],confidence_convention="0 best, 100 worst")
    runtime=sl.RuntimeParameters();runtime.confidence_threshold=95;runtime.texture_confidence_threshold=100
    depth=sl.Mat();confidence=sl.Mat();rgb=sl.Mat()
    frames=[];conf=[];count=0;end=time.monotonic()+95
    while len(frames)<1 and time.monotonic()<end:
        if z.grab(runtime)!=sl.ERROR_CODE.SUCCESS:continue
        count+=1
        if count<45 or count%2:continue
        z.retrieve_measure(depth,sl.MEASURE.DEPTH)
        z.retrieve_measure(confidence,sl.MEASURE.CONFIDENCE)
        frames.append(depth.get_data()[350:580,600:980].copy())
        conf.append(confidence.get_data()[350:580,600:980].astype(np.uint8))
        meta["timestamps_ns"].append(z.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds())
        if len(frames) in (1,):
            z.retrieve_image(rgb,sl.VIEW.LEFT);rgb.write(str(out/("rgb-"+str(len(frames))+".png")))
        if len(frames)%75==0:print("Captured",len(frames),flush=True)
    if len(frames)!=1:raise RuntimeError("insufficient frames")
    np.savez_compressed(out/"depths-confidence.npz",depths=np.array(frames),confidence=np.array(conf))
    (out/"capture.json").write_text(json.dumps(meta,indent=2))
    print("Saved 450 depth/confidence frames",flush=True)
finally:z.close()
