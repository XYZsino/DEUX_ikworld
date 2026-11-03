"""Normal soft-yaw selection plus a hard axis-preserving transverse branch."""
import copy
import numpy as np
from scipy.spatial.transform import Rotation
from grid_grasp_runtime import GridLibrary,XY_TOLERANCE_M


def wrapped_degrees(angle):
    return (float(angle)+180.)%360.-180.


def transverse_yaw(angle):
    return 60.-1e-9 <= abs(wrapped_degrees(angle)) <= 120.+1e-9


def axis_error_degrees(a,b):
    return abs((float(a)-float(b)+90.)%180.-90.)


class ClassifiedGridLibrary(GridLibrary):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.normal_options={};self.transverse_options={};self.axis_headings={}
        for key,p in self.entries.items():
            point=tuple(p['contact_xyz_m'])
            if p.get('selection_class')=='transverse':
                if p.get('front_back_reversal_allowed') is not True:raise ValueError('explicit axis symmetry required')
                r=self.contact_rotations[key]@self.offset[:3,:3].T
                self.axis_headings[key]=float(np.degrees(np.arctan2(r[1,0],r[0,0])))
                self.transverse_options.setdefault(point,[]).append(key)
            else:self.normal_options.setdefault(point,[]).append(key)
        if not self.normal_options or not self.transverse_options:raise ValueError('both normal and transverse verified paths required')
        self.normal_view=copy.copy(self);self.normal_view.cell_options=self.normal_options

    def select(self,object_pose,velocity,stamp,now,lead=.8):
        grasp,_=self.grasp_now(object_pose,velocity,stamp,now)
        pose=np.asarray(object_pose)
        available=np.hypot(pose[0,0],pose[1,0])>1e-6
        yaw=float(np.degrees(np.arctan2(pose[1,0],pose[0,0]))) if available else 0.
        if not available or not transverse_yaw(yaw):
            result=GridLibrary.select_with_yaw(self.normal_view,grasp,object_pose,stamp,now,lead)
            result.update(grasp_mode='normal',orientation_exception_applied=False)
            return result
        # Axis reversal was explicitly authorized: +60..+120 map to -120..-60.
        canonical=wrapped_degrees(yaw)
        if canonical>0:canonical-=180.
        desired=Rotation.from_euler('z',canonical,degrees=True).as_matrix()@self.offset[:3,:3]
        choices=[]
        for cell,keys in self.transverse_options.items():
            eta=(cell[1]-grasp[1,3])/self.velocity[1]
            lateral=abs(float(grasp[0,3]-cell[0]))
            if not (3.+lead<=eta<=10. and lateral<=XY_TOLERANCE_M+1e-9):continue
            for key in keys:
                # 5deg heading bins plus <=0.5deg checked path-axis residual
                # reserve a total 3deg alignment budget throughout the path.
                if axis_error_degrees(self.entries[key]['axis_heading_deg'],yaw)>2.5+1e-9:continue
                axis_error=axis_error_degrees(self.axis_headings[key],yaw)
                if axis_error>3.+1e-9:continue
                error=float(Rotation.from_matrix(self.contact_rotations[key]@desired.T).magnitude())
                acceleration=self.entries[key]['checks']['acceleration_limit_ratio']
                choices.append((lateral,eta,error,acceleration,key,axis_error))
        if not choices:
            raise ValueError('transverse bag: no verified axis-preserving path within 3deg / 30mm / lead time; no normal fallback')
        lateral,eta,error,acceleration,key,axis_error=min(choices)
        p=self.entries[key]
        return dict(cell_id=key,contact_xyz_m=p['contact_xyz_m'],start_at_s=now+eta-3.,contact_at_s=now+eta,
            start_in_s=eta-3.,contact_in_s=eta,lateral_error_m=lateral,height_error_m=0.,
            grasp_mode='transverse_axis_locked',orientation_exception_applied=True,
            observed_yaw_deg=yaw,canonical_axis_yaw_deg=canonical,yaw_available=True,
            selected_axis_yaw_deg=self.axis_headings[key],axis_error_deg=axis_error,axis_match_tolerance_deg=3.,
            front_back_reversed=abs(wrapped_degrees(self.axis_headings[key]-yaw))>90.,
            rotation_error_deg=p['checks']['orientation_error_deg'],
            yaw_target_rotation_error_deg=float(np.degrees(error)),selected_rotation=p['rotation_search'],
            rotation_candidates_at_cell=sum(axis_error_degrees(self.axis_headings[k],yaw)<=3.+1e-9
                                             and axis_error_degrees(self.entries[k]['axis_heading_deg'],yaw)<=2.5+1e-9
                                             for k in self.transverse_options[tuple(p['contact_xyz_m'])]),
            acceleration_limit_ratio=acceleration,rotation_is_soft_preference=False,
            selection_priority='transverse axis match <=3deg required; then X error, ETA, target rotation, acceleration',
            acquired_at_s=stamp,grasp_now_m=grasp[:3,3].tolist(),velocity_m_s=self.velocity.tolist())

    def live_error(self,selected,object_pose,velocity,stamp,now):
        error=super().live_error(selected,object_pose,velocity,stamp,now)
        pose=np.asarray(object_pose);yaw=float(np.degrees(np.arctan2(pose[1,0],pose[0,0])))
        special=selected.get('grasp_mode')=='transverse_axis_locked'
        if special!=transverse_yaw(yaw):raise ValueError('bag orientation class changed before dispatch')
        if special and axis_error_degrees(self.entries[selected['cell_id']]['axis_heading_deg'],yaw)>2.5+1e-9:
            raise ValueError('transverse axis changed before dispatch; stored angle remains locked')
        return error
