"""ROI entry selects one bag; leaving the ROI terminates that identity."""
import json
from pathlib import Path
from uuid import uuid4
import numpy as np
from .yolo_backend import YoloDetector
from .roi_tracker import RoiBagTracker
from .tracking_roi import read_roi

class SingleTargetDetector(YoloDetector):
 def __init__(self,path,device,confidence,bag_names,barcode_name,roi_file,max_gap_s=.75,require_entry=True):
  super().__init__(path,device,.1)
  self.require_entry=bool(require_entry)
  self.bag_ids=[int(i) for i,n in self.model.names.items() if n in bag_names]
  self.barcode_name=barcode_name;self.high=confidence;self.roi_file=roi_file;self.max_gap_s=max_gap_s
  self.tracker=None;self.roi_key=None;self.last_stamp=None;self.last_seen=None;self.public_id='';self.session=uuid4().hex[:8];self.serial=0;self.hold_target=False
 def track(self,rgb,acquisition_s):
  # Use predict, never model.track: other bags have no persistent tracks.
  detections=self(rgb)
  # Same-frame raw detections are available to scan presence only; never use
  # these ephemeral identities for conveyor acquisition or depth/ICP tracking.
  self.scan_bags=[dict(class_id=int(cls),class_name=name,confidence=float(score),
      xyxy=list(map(float,xy)),track_id=f'frame-scan-bag-{i}')
      for i,(cls,name,score,xy) in enumerate(detections) if int(cls) in self.bag_ids and score>=.15]
  rows=np.asarray([list(xy)+[score,cls] for cls,name,score,xy in detections],float).reshape(-1,6)
  return self.update_rows(rows,rgb.shape[:2],acquisition_s)
 def update_rows(self,rows,shape,stamp):
  self.exit_event=None
  stamp=float(stamp)
  if not np.isfinite(stamp):raise ValueError('invalid acquisition timestamp')
  roi=read_roi(self.roi_file);key=json.dumps([roi,list(shape)],sort_keys=True)
  if key!=self.roi_key or (self.last_stamp is not None and stamp<=self.last_stamp):
   if self.public_id:self.exit_event=dict(target_id=self.public_id,stamp=stamp,reason='roi_changed')
   self.tracker=None;self.public_id='';self.last_seen=None;self.roi_key=key
   if roi:
    h,w=shape;poly=np.asarray(roi['points'])*[w/roi['width'],h/roi['height']]
    self.tracker=RoiBagTracker(poly,self.bag_ids,high=self.high,low=.1,buffer=30,require_entry=getattr(self,"require_entry",True))
  self.last_stamp=stamp
  labels=[dict(class_id=int(row[5]),class_name=self.model.names[int(row[5])],confidence=float(row[4]),xyxy=row[:4].tolist(),track_id=f'frame-barcode-{i}') for i,row in enumerate(rows) if self.model.names[int(row[5])]==self.barcode_name]
  if self.tracker is None:return labels
  owner=self.tracker;old=owner.tracker.track
  if self.last_seen is not None and stamp-self.last_seen>self.max_gap_s:
   if self.public_id:self.exit_event=dict(target_id=self.public_id,stamp=stamp,reason='tracking_expired')
   owner.tracker.reset();old=None;self.public_id='';self.last_seen=None
   # Never silently replace a selected task target after acquisition-time expiry.
   if self.hold_target:return labels
  if self.hold_target and not self.public_id:return labels
  target=owner.update(rows if owner.tracker.track is not None else rows[rows[:,4]>=self.high])
  if old is not None and owner.tracker.track is None:
   self.exit_event=dict(target_id=self.public_id,stamp=stamp,reason='roi_exit_or_tracking_lost')
   self.public_id='';self.last_seen=None
  if target is None:return labels
  if old is not owner.tracker.track or not self.public_id:
   self.serial+=1;self.public_id=f'single-{self.session}-{self.serial}'
  self.last_seen=stamp
  # ROI selects; raw observed box feeds depth/ICP, never Kalman-only predictions.
  return [dict(class_id=target.class_id,class_name=self.model.names[target.class_id],confidence=target.score,xyxy=list(owner.tracker.observed_xyxy),track_id=self.public_id)]+labels
