"""Causal crossing-X selection and validated upper alignment; no robot commands."""
from collections import deque
import numpy as np
from upper_rendezvous import following_path
from right_only_trial import planning_robot
from right_clear_approach import segment
from deux_task_core.live_continuous_preview import roi_contains
from deux_task_core.tracking_policy import inside_xy


class AlignmentTooLate(ValueError):pass


def crossing(fixed,obj,v,grasp,stamp,now):
    if not np.isfinite([stamp,now]).all() or not 0<=now-stamp<=.5:raise ValueError('X 정렬: 관측 시각 만료')
    v=np.asarray(v);point=(np.asarray(obj)@np.asarray(grasp))[:3,3]
    if not np.isfinite(v).all() or abs(v[1])<.01:raise ValueError('X 정렬: Y 이동 속도 부족')
    dt=(fixed[1,3]-point[1])/v[1]
    return float(point[0]+v[0]*dt),float(stamp+dt)


class CrossingWindow:
    def __init__(self):self.target=None;self.rows=deque(maxlen=3)
    def update(self,identity,stamp,x):
        if identity!=self.target:self.target=identity;self.rows.clear()
        if not np.isfinite([stamp,x]).all():self.rows.clear();return None
        if self.rows and stamp<=self.rows[-1][0]:return None
        if self.rows and stamp-self.rows[-1][0]>.3:self.rows.clear()
        self.rows.append((stamp,x))
        if len(self.rows)<3 or self.rows[-1][0]-self.rows[0][0]<.15:return None
        values=np.array([r[1] for r in self.rows])
        if np.ptp(values)>.005:return None
        return float(np.median(values))


def select_point(base,x,config,max_shift=.06):
    if not np.isfinite(x) or abs(x-base[0,3])>max_shift:raise ValueError('X 정렬: 기준점에서 6cm 범위 초과')
    p=np.array(base).copy();p[0,3]=x
    if not inside_xy(config['tracking_grasp_policy'],[p[:3,3]],config['tracking_grasp_policy']['entry_margin_m']):
        raise ValueError('X 정렬: 작업영역 밖 예상 통과점')
    return p


def build_alignment(config,q,fixed,v,policy):
    r=planning_robot(config);initial=r.ee_fk('right',q);upper=np.array(fixed).copy();upper[2,3]+=config['approach_height']
    if np.max(abs(initial[[1,2],3]-upper[[1,2],3]))>.01:raise ValueError('X 정렬: 상부 Y/Z 대기 자세 미정착')
    plan,detail=segment(config,r,np.asarray(q),initial,upper,'AlignCrossingX')
    # Validate tracking and braking from the aligned endpoint before moving X.
    follow=following_path(config,np.array(plan['path'][-1]),fixed,np.asarray(v),policy)
    plan['x_alignment']=dict(detail,following_min_margin_m=follow['min_conveyor_margin_m'],
                              following_duration_s=follow['times'][-1],fixed_pose=np.asarray(fixed).tolist())
    return plan


def admission(plan,fixed,grasp,obj,v,stamp,now,geometry,policy):
    x,at=crossing(fixed,obj,v,grasp,stamp,now)
    if abs(x-fixed[0,3])>.005:raise ValueError('X 정렬: 계획 중 예상 통과 X가 5mm 이상 변경')
    # Scheduling lead + full alignment + quiet feedback + fresh following solve/dispatch + half-ramp.
    needed=.4+plan['times'][-1]+.2+policy['settle_s']+.8+policy['ramp_s']/2
    if at-now<needed:raise AlignmentTooLate(f'X 정렬 시간 부족: 도착 {at-now:.2f}초 / 필요 {needed:.2f}초')
    finish=at-policy['ramp_s']/2+plan['x_alignment']['following_duration_s']
    points=[(obj@grasp)[:3,3]+v[:3]*(t-stamp) for t in np.linspace(now,finish,30)]
    if not roi_contains(geometry,points):raise ValueError('X 정렬: 합류·정지 전에 ROI 이탈 예상')
    return at,needed
