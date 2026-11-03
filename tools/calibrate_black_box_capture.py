"""Capture stationary ZED RGB/depth for black-box bottom-plane calibration."""
from pathlib import Path
import json,time
import numpy as np
import pyzed.sl as sl
out=Path(__file__).resolve().parents[1]/".setup/black-box-plane-90"
out.mkdir(parents=True,exist_ok=True)
z=sl.Camera()
p=sl.InitParameters()
p.camera_resolution=sl.RESOLUTION.HD720
p.camera_fps=15
p.depth_mode=sl.DEPTH_MODE.ULTRA
p.coordinate_units=sl.UNIT.METER
p.coordinate_system=sl.COORDINATE_SYSTEM.IMAGE
p.depth_minimum_distance=.3
p.depth_maximum_distance=3.
p.depth_stabilization=0
e=z.open(p)
if e!=sl.ERROR_CODE.SUCCESS:raise RuntimeError(str(e))
try:
    info=z.get_camera_information()
    c=info.camera_configuration.calibration_parameters.left_cam
    meta=dict(serial=info.serial_number,sdk=z.get_sdk_version(),camera_frame="zed2i_left_camera_optical_frame",width=1280,height=720,fx=c.fx,fy=c.fy,cx=c.cx,cy=c.cy,coordinate_system="IMAGE: x right, y down, z forward",units="metres",depth_mode="ULTRA",shoulder_axis_height_from_floor_m=1.34,base_height_from_floor_m=None,timestamps_ns=[])
    runtime=sl.RuntimeParameters()
    runtime.confidence_threshold=95
    runtime.texture_confidence_threshold=100
    depth=sl.Mat()
    rgb=sl.Mat()
    frames=[]
    count=0
    end=time.monotonic()+55
    while len(frames)<90 and time.monotonic()<end:
        if z.grab(runtime)!=sl.ERROR_CODE.SUCCESS:continue
        count+=1
        if count<30 or count%3:continue
        z.retrieve_measure(depth,sl.MEASURE.DEPTH)
        frames.append(depth.get_data().copy())
        meta["timestamps_ns"].append(z.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds())
        if len(frames)==1:
            z.retrieve_image(rgb,sl.VIEW.LEFT)
            rgb.write(str(out/"rgb.png"))
    if len(frames)!=90:raise RuntimeError("insufficient frames")
    np.savez_compressed(out/"depths.npz",depths=np.array(frames))
    (out/"capture.json").write_text(json.dumps(meta,indent=2))
    print(json.dumps(meta,indent=2))
finally:z.close()
