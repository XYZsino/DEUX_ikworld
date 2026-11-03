"""Single-worker-owned ICP geometry and motion filter; no ROS node or subscriptions."""
from copy import deepcopy
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
from deux_task_msgs.msg import ObjectDetection, ObjectDetectionArray
from .core import normalize_depth, stamp_ns, depth_view
from .perception_core import load_model, roi_points, Tracker, masked_depth, filter_above_plane
from .yaw_icp import fit_icp
from deux_task_core.motion_filter import MotionFilter

ICP_DEFAULTS = {
            'input_topic': '/task/detection_frames', 'output_topic': '/task/object_detections',
            'model_path': '', 'model_origin': '', 'reference_frame': '', 'base_frame': 'base_link',
            'min_height_m': .01, 'max_distance_m': 1.5, 'max_rmse_m': .015,
            'min_coverage': .5, 'track_gate_m': .08, 'track_max_age_s': .6,
            'position_std_m': .02, 'yaw_std_rad': .2, 'unobserved_std_rad': math.pi,
            'width': 640, 'height': 480, 'sync_slop_s': .02,
            'target_classes': ['s1', 's1_1'], 'roi_only_depth': False, 'require_instance_mask': False, 'resting_on_reference_plane': False,
            'icp_adaptive_search': False, 'icp_acquisition_angles': 6, 'icp_local_yaw_offset_deg': 10.,
            'icp_initial_angles': 24, 'icp_yaw_span_deg': 360.0,
            'icp_coarse_point_limit': 0, 'icp_coarse_voxel_size_m': 0.0,
            'table_plane_filter_enabled': False, 'table_plane_base': [0.,0.,1.,0.],
            'table_plane_clearance_m': .01, 'table_plane_provisional': True,
            'filter_motion': False, 'filter_max_jump_m': .08,
            'filter_acceleration_std_m_s2': .015,
            'filter_velocity_prior_enabled': False, 'filter_initial_velocity': [0.,0.,0.,0.],
            'filter_velocity_prior_std_m_s': .01,
            'filter_max_speed_m_s': .3, 'filter_innovation_gate': 16.}


class IcpEstimator:
    def __init__(self, config, model_data=None):
        self.cfg = dict(config)
        if self.cfg.get('table_plane_filter_enabled', False):
            filter_above_plane(np.empty((0,3)), np.eye(4),
                               self.cfg['table_plane_base'], self.cfg['table_plane_clearance_m'])
        for key in ('max_distance_m', 'max_rmse_m', 'track_gate_m', 'track_max_age_s',
                    'position_std_m', 'yaw_std_rad', 'unobserved_std_rad', 'width', 'height', 'sync_slop_s'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(f'{key} must be positive and finite')
        if (type(self.cfg['icp_initial_angles']) is not int or self.cfg['icp_initial_angles'] < 1
                or not math.isfinite(self.cfg['icp_yaw_span_deg'])
                or not 0 < self.cfg['icp_yaw_span_deg'] <= 360
                or type(self.cfg['icp_coarse_point_limit']) is not int
                or (self.cfg['icp_coarse_point_limit'] != 0 and self.cfg['icp_coarse_point_limit'] < 40)
                or not math.isfinite(self.cfg['icp_coarse_voxel_size_m'])
                or (self.cfg['icp_coarse_voxel_size_m'] != 0 and self.cfg['icp_coarse_voxel_size_m'] < .003)
                or (self.cfg['icp_coarse_voxel_size_m'] != 0 and self.cfg['icp_coarse_point_limit'] != 0)):
            raise ValueError('invalid ICP seed coverage or coarse point limit')
        if (not self.cfg['reference_frame'] or not self.cfg['base_frame'] or
                not math.isfinite(self.cfg['min_height_m']) or
                not 0 <= self.cfg['min_coverage'] <= 1):
            raise ValueError('explicit reference/base frames and finite height/coverage required')
        self.model, self.normals, self.model_id = model_data if model_data is not None else load_model(
            self.cfg['model_path'], self.cfg['model_origin'])
        self.tracker = Tracker(self.cfg['track_gate_m'], self.cfg['track_max_age_s'])
        self.motion_filter = MotionFilter(max_gap=self.cfg['track_max_age_s'],
            max_jump=self.cfg['filter_max_jump_m'],max_speed=self.cfg['filter_max_speed_m_s'],
            innovation_gate=self.cfg['filter_innovation_gate'],
            acceleration_std=self.cfg['filter_acceleration_std_m_s2'],
            initial_velocity=self.cfg['filter_initial_velocity'] if self.cfg['filter_velocity_prior_enabled'] else None,
            initial_velocity_std=self.cfg['filter_velocity_prior_std_m_s'])
        if (type(self.cfg.get('icp_adaptive_search',False)) is not bool
                or type(self.cfg.get('icp_acquisition_angles',6)) is not int
                or not 1<=self.cfg.get('icp_acquisition_angles',6)<=24
                or not np.isfinite(self.cfg.get('icp_local_yaw_offset_deg',10.))
                or not 0<self.cfg.get('icp_local_yaw_offset_deg',10.)<=45):
            raise ValueError('invalid adaptive ICP search configuration')
        self.filter_rejections = 0
        self.search_seen=set();self.search_seeds={};self.search_epoch=None
        self.track_epoch = None

    def estimate(self, payload):
        started = time.perf_counter()
        self.last_timing = dict(roi_ms=0., fit_ms=0., fits=[], roi=[], rejections=[], filter_results=[])
        frame, rc, br, epoch = payload
        self.last_timing["input_boxes"] = len(frame.boxes)
        self.last_timing["thresholds"] = {k:self.cfg[k] for k in ("max_rmse_m", "min_coverage", "min_height_m", "max_distance_m")}
        if epoch != self.track_epoch:
            self.tracker.tracks.clear()
            self.motion_filter.tracks.clear()
            camera_epoch=epoch[0] if isinstance(epoch,(tuple,list)) else epoch
            if camera_epoch!=self.search_epoch:
                self.search_seen.clear();self.search_seeds.clear();self.search_epoch=camera_epoch
            self.track_epoch = epoch
        if self.cfg['roi_only_depth']:
            depth, depth_scale = depth_view(frame.depth)
        else:
            depth, _ = normalize_depth(frame.depth)
            depth_scale = 1.0
        info = frame.depth_info
        headers = [frame.header, frame.depth.header, info.header]
        times = [stamp_ns(h) for h in headers]
        if (len({h.frame_id for h in headers}) != 1 or not frame.header.frame_id or
                min(times) <= 0 or max(times)-min(times) > self.cfg['sync_slop_s']*1e9 or
                depth.shape != (self.cfg['height'], self.cfg['width']) or
                (info.height, info.width) != depth.shape or
                info.binning_x not in (0, 1) or info.binning_y not in (0, 1) or
                info.roi.x_offset or info.roi.y_offset or
                info.roi.width not in (0, info.width) or info.roi.height not in (0, info.height)):
            raise ValueError('depth/calibration acquisition mismatch')
        out = ObjectDetectionArray()
        out.header = deepcopy(frame.header)
        out.header.frame_id = self.cfg['base_frame']
        out.sequence, out.source, out.model_id = frame.sequence, 'yaw_icp/' + frame.detector_id, self.model_id
        out.request_id = frame.request_id
        self.last_timing["prepare_ms"] = (time.perf_counter()-started)*1000
        poses, scores, metrics, visual_ids = [], [], [], []
        for box in frame.boxes:
            if box.class_name not in self.cfg['target_classes']:
                continue
            if not math.isfinite(box.confidence) or not 0 <= box.confidence <= 1:
                raise ValueError('invalid detection confidence')
            if self.cfg["require_instance_mask"] and not box.polygon_xy:
                raise ValueError("visible instance mask required for selected-target ICP")
            roi_start = time.perf_counter()
            selected_depth = masked_depth(depth, box.polygon_xy) if box.polygon_xy else depth
            roi_diag = {"track_id":box.track_id}
            self.last_timing["roi"].append(roi_diag)
            points = roi_points(selected_depth, info.p, box.xyxy, rc,
                                self.cfg['min_height_m'], self.cfg['max_distance_m'], depth_scale=depth_scale, diagnostics=roi_diag)
            if self.cfg.get('table_plane_filter_enabled', False):
                points = filter_above_plane(points, br, self.cfg['table_plane_base'],
                                            self.cfg['table_plane_clearance_m'], roi_diag)
                roi_diag['table_plane_provisional'] = self.cfg['table_plane_provisional']
                roi_diag['table_plane_frame'] = self.cfg['base_frame']
            self.last_timing["roi_ms"] += (time.perf_counter()-roi_start)*1000
            fit_start = time.perf_counter()
            options=dict(initial_angles=self.cfg['icp_initial_angles'],yaw_span_deg=self.cfg['icp_yaw_span_deg'],
                coarse_point_limit=self.cfg['icp_coarse_point_limit'],coarse_voxel_size_m=self.cfg['icp_coarse_voxel_size_m'],
                support_plane_z=0. if self.cfg['resting_on_reference_plane'] else None)
            adaptive=self.cfg.get('icp_adaptive_search',False);mode='full';identity=(frame.request_id,box.track_id)
            first_search=box.track_id not in self.search_seen
            if adaptive:
                self.search_seen.add(box.track_id)
                options.update(initial_angles=self.cfg['icp_acquisition_angles'],coarse_rank_reduced=True)
                mode='acquisition'
                old=self.motion_filter.tracks.get(identity)
                stamp=stamp_ns(frame.header)*1e-9
                if old is not None and old.stamp<stamp<=old.stamp+self.motion_filter.max_gap and np.allclose(br[:3,:3],np.eye(3),atol=1e-8):
                    projected,_=self.motion_filter.project(old,stamp)
                    offset=np.deg2rad(self.cfg['icp_local_yaw_offset_deg'])
                    options.update(initial_yaws=projected[3]+np.array([0.,-offset,offset]),initial_translation=projected[:3]-br[:3,3])
                    mode='local'
                elif box.track_id in self.search_seeds:
                    last=self.search_seeds[box.track_id]
                    offset=np.deg2rad(self.cfg['icp_local_yaw_offset_deg'])
                    options.update(initial_yaws=last[0]+np.array([0.,-offset,offset]),initial_translation=last[1].copy())
                    mode='local_last_fit'
            fit=fit_icp(points,self.model,self.normals,rc[:3,3],**options)
            attempts=[dict(mode=mode,**{k:fit[k] for k in ('status','seed_count','timing_ms','trimmed_rmse_m','target_coverage_10mm') if k in fit})]
            def passes(value):
                return (value.get('status')=='fitted' and np.isfinite([value['trimmed_rmse_m'],value['target_coverage_10mm']]).all()
                        and value['trimmed_rmse_m']<=self.cfg['max_rmse_m'] and value['target_coverage_10mm']>=self.cfg['min_coverage'])
            if adaptive and first_search and not passes(fit) and fit.get('status')!='insufficient_depth':
                # Exact legacy full search is the quality fallback, not relaxed thresholds.
                fit=fit_icp(points,self.model,self.normals,rc[:3,3],initial_angles=self.cfg['icp_initial_angles'],
                    yaw_span_deg=self.cfg['icp_yaw_span_deg'],support_plane_z=options['support_plane_z'])
                attempts.append(dict(mode='fallback_full',**{k:fit[k] for k in ('status','seed_count','timing_ms','trimmed_rmse_m','target_coverage_10mm') if k in fit}))
            if adaptive and fit.get('status')=='fitted':
                fitted_pose=np.asarray(fit['T_reference_model'])
                if np.isfinite(fitted_pose).all():self.search_seeds[box.track_id]=(float(np.arctan2(fitted_pose[1,0],fitted_pose[0,0])),fitted_pose[:3,3].copy())
            self.last_timing.setdefault('search_attempts',[]).append(dict(track_id=box.track_id,first_frame=first_search,attempts=attempts))
            self.last_timing['fit_ms'] += (time.perf_counter()-fit_start)*1000
            self.last_timing['fits'].append({k:fit[k] for k in ('status','timing_ms','points','fit_points','coarse_fit_points','total_iterations','refined_candidates','trimmed_rmse_m','target_coverage_10mm') if k in fit})
            reasons = []
            if fit['status'] != 'fitted':
                reasons.append(fit['status'])
            else:
                if not np.isfinite([fit['trimmed_rmse_m'], fit['target_coverage_10mm']]).all():
                    reasons.append("nonfinite_fit_metrics")
                if fit['trimmed_rmse_m'] > self.cfg['max_rmse_m']:
                    reasons.append("rmse_above_limit")
                if fit['target_coverage_10mm'] < self.cfg['min_coverage']:
                    reasons.append("coverage_below_limit")
            if reasons:
                self.last_timing['rejections'].append(dict(track_id=box.track_id, reasons=reasons))
                continue
            pose = br @ np.array(fit['T_reference_model'])
            if not np.isfinite(pose).all():
                self.last_timing["rejections"].append(dict(track_id=box.track_id, reasons=["nonfinite_pose"]))
                continue
            poses.append(pose)
            scores.append(box.confidence)
            visual_ids.append(box.track_id)
            metrics.append((fit['trimmed_rmse_m'], fit['target_coverage_10mm']))
        post_start = time.perf_counter()
        ids = self.tracker.update([p[:3, 3] for p in poses], stamp_ns(frame.header)*1e-9)
        ids = [visual or fallback for visual, fallback in zip(visual_ids, ids)]
        # Configured uncertainty assumptions, not covariance estimated from ICP residuals.
        cov = np.diag([self.cfg['position_std_m']**2]*3 +
                      [self.cfg['unobserved_std_rad']**2]*2 + [self.cfg['yaw_std_rad']**2])
        axes = np.zeros((6, 6))
        axes[:3, :3] = axes[3:, 3:] = br[:3, :3]
        cov = axes @ cov @ axes.T
        for identity, pose, score, (rmse, coverage) in zip(ids, poses, scores, metrics):
            detection = ObjectDetection(track_id=identity, confidence=score, full_orientation=False)
            filtered_cov = cov.copy()
            if self.cfg['filter_motion']:
                yaw=math.atan2(pose[1,0],pose[0,0])
                state,reason=self.motion_filter.update((frame.request_id,identity),
                    stamp_ns(frame.header)*1e-9,frame.sequence,np.r_[pose[:3,3],yaw],np.diag(cov)[[0,1,2,5]])
                self.last_timing["filter_results"].append(dict(track_id=identity, accepted=state is not None, reason=reason))
                if state is None:
                    self.filter_rejections += 1
                    continue
                pose=pose.copy();pose[:3,3]=state.state[:3]
                pose[:3,:3]=Rotation.from_rotvec([0.,0.,state.state[3]-yaw]).as_matrix()@pose[:3,:3]
                detection.pose_filtered=True
                detection.motion_covariance_valid=True
                detection.motion_covariance=state.covariance.ravel().tolist()
                detection.filter_observations=state.observations
                detection.velocity_valid=state.observations>=3
                detection.velocity.linear.x,detection.velocity.linear.y,detection.velocity.linear.z=map(float,state.state[4:7])
                detection.velocity.angular.z=float(state.state[7])
                idx=[0,1,2,5];filtered_cov[np.ix_(idx,idx)]=state.covariance[:4,:4]
            detection.fit_metrics_valid = True
            detection.trimmed_rmse_m, detection.target_coverage_10mm = rmse, coverage
            detection.covariance_estimated = False
            p, q = detection.pose.pose.position, detection.pose.pose.orientation
            p.x, p.y, p.z = map(float, pose[:3, 3])
            q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(pose[:3, :3]).as_quat())
            detection.pose.covariance = filtered_cov.ravel().tolist()
            out.detections.append(detection)
        self.last_timing['accepted_detections'] = len(out.detections)
        if not frame.boxes:
            self.last_timing['rejections'].append(dict(reasons=["no_selected_detection"]))
        self.last_timing['filter_pack_ms'] = (time.perf_counter()-post_start)*1000
        self.last_timing['estimator_ms'] = (time.perf_counter()-started)*1000
        if self.cfg['filter_motion'] and not out.detections:
            # Do not overwrite the last accepted measurement with a fresh empty
            # acquisition after a rejected fit/spike. Its original timestamp
            # still expires normally in every downstream consumer.
            raise ValueError('no accepted filtered ICP measurement')
        return out
