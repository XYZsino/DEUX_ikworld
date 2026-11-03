"""Versioned image ROI shared by GUI and the single-target perception worker."""
import json,os
from pathlib import Path
import cv2
import numpy as np

def default_path():return os.environ.get('DEUX_TRACK_ROI_FILE',str(Path.cwd()/'.setup/single-track-roi.json'))

def validate_roi(data):
 if not isinstance(data,dict):raise ValueError('ROI object required')
 w,h=data.get('width'),data.get('height');points=np.asarray(data.get('points'),float)
 if type(w) is not int or type(h) is not int or not 1<=w<=8192 or not 1<=h<=8192:raise ValueError('invalid image dimensions')
 if points.shape!=(4,2) or not np.isfinite(points).all():raise ValueError('select four finite corners')
 if np.any(points<0) or np.any(points[:,0]>w) or np.any(points[:,1]>h):raise ValueError('ROI outside image')
 if not cv2.isContourConvex(points.astype(np.float32)) or cv2.contourArea(points.astype(np.float32))<25:raise ValueError('select convex corners in boundary order')
 return dict(width=w,height=h,points=points.tolist())

def read_roi(path):
 p=Path(path)
 return validate_roi(json.loads(p.read_text())) if p.exists() else None

def write_roi(path,data):
 data=validate_roi(data);p=Path(path);p.parent.mkdir(parents=True,exist_ok=True)
 tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps(data));tmp.replace(p);return data
