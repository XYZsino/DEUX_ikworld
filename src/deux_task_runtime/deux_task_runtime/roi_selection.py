"""Single acquired RGB frame + depth box admission, independent of motion planning."""
import numpy as np
from .common import current, seconds


def select_roi_target(node, requested_id=''):
    def blocked(reason):
        node.reason=reason
        return False
    m=node.tracks; candidates=node.candidates; req=node.perception_request
    if m is None:return blocked('RGB 검출 데이터 수신 대기')
    if not current(m.header.stamp,node.now(),node.config['max_object_age']):
        return blocked('RGB 검출 데이터가 오래되었거나 미래 시각입니다')
    if req is None or m.request_id!=req.request_id or seconds(m.header.stamp)<seconds(req.header.stamp):
        return blocked('현재 선택 요청의 RGB 검출 대기')
    if not m.selection_roi_valid:return blocked('오른손 선택 영역의 RGB 투영 ROI 대기: 카메라 TF 확인')
    roi=np.asarray(m.selection_roi_xyxy)
    if not np.isfinite(roi).all() or np.any(roi[2:]<=roi[:2]):
        return blocked('유효하지 않은 RGB ROI')
    bags=[b for b in m.boxes if b.class_name=='bread_bag' and b.track_id
          and np.isfinite(b.confidence) and 0<=b.confidence<=1
          and (not requested_id or b.track_id==requested_id)]
    if not bags:return blocked('지정 대상 또는 추적 가능한 빵 봉투 없음')
    inside=[]
    for b in bags:
        box=np.asarray(b.xyxy)
        if not np.isfinite(box).all() or np.any(box[2:]<=box[:2]):continue
        center=(box[:2]+box[2:])/2
        if np.all(center>=roi[:2]) and np.all(center<=roi[2:]):inside.append(b)
    if not inside:return blocked(f'RGB ROI 안에 봉투 중심이 없음: 검출 {len(bags)}개')
    if (candidates is None or candidates.request_id!=req.request_id
            or candidates.sequence!=m.sequence or candidates.header.stamp!=m.header.stamp
            or candidates.header.frame_id!=node.config['base_frame']):
        return blocked('ROI 안의 봉투와 같은 프레임의 3D 박스 대기')
    by_id={c.target_id:c for c in candidates.candidates}
    allowed=[]
    for b in inside:
        c=by_id.get(b.track_id)
        if c is None or not c.bag_visible or not c.bbox3d_valid:continue
        lo=np.array([c.bbox3d_min.x,c.bbox3d_min.y,c.bbox3d_min.z])
        hi=np.array([c.bbox3d_max.x,c.bbox3d_max.y,c.bbox3d_max.z])
        if not np.isfinite([lo,hi]).all() or np.any(hi<lo) or np.max(hi-lo)<.01 or np.any(hi-lo>.6):continue
        allowed.append((b,(lo+hi)/2))
    if not allowed:return blocked(f'RGB ROI 안 봉투 {len(inside)}개: 유효한 깊이 기반 3D 박스 대기')
    if node.config.get('moving_source',False):
        direction=np.asarray(node.config['conveyor_tracking']['flow_direction'])
        region=node.config['conveyor_tracking'].get('acquisition_region')
        def rank(item):
            # Preference only: never reject an otherwise valid ROI/depth box.
            # Favor the conveyor lane crossing the workspace over a parallel
            # lane beside the hand, then keep the upstream-first ordering.
            outside=0.
            if region:
                lo=np.asarray(region['min']);hi=np.asarray(region['max'])
                cross=item[1]+direction*np.dot((lo+hi)/2-item[1],direction)
                outside=np.linalg.norm(np.maximum(lo-cross,0)+np.maximum(cross-hi,0))
            return (outside,np.dot(item[1],direction))
        chosen=min(allowed,key=rank)[0]
    else:
        center=(roi[:2]+roi[2:])/2
        chosen=min(allowed,key=lambda item:np.linalg.norm((np.asarray(item[0].xyxy[:2])+item[0].xyxy[2:])/2-center))[0]
    node.target_id=chosen.track_id;node.reason=''
    return True
