"""Scalar LeaderGrip transport with measured feedback and fail-closed mode/profile checks."""
from pathlib import Path
import time
import numpy as np
import yaml
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import parameter_value_to_python
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterEvent
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from copy import deepcopy
from .common import current, LATCHED
from deux_hand_msgs.msg import LeaderGrip, HandCtlMessage, HandUiMessage
from deux_task_core.model import HAND_NAMES, vector
from .hand_adapter import HandAdapter


def grip_fraction(positions, initial, target, tolerance):
    q=vector(positions,7);a=vector(initial,7);b=vector(target,7);delta=b-a
    if np.dot(delta,delta)<1e-12:
        raise ValueError("scalar hand profile has no moving joint")
    fraction=float(np.clip(np.dot(q-a,delta)/np.dot(delta,delta),0.,1.))
    if np.max(np.abs(q-(a+fraction*delta)))>tolerance:
        raise ValueError("planned hand posture is outside the configured scalar profile")
    return fraction


class LeaderGripHandAdapter(HandAdapter):
    def __init__(self,node):
        self.node=node
        self.mapping=yaml.safe_load(Path(node.get_parameter("mapping_file").value).read_text())
        self.virtual_right=node.config.get('right_hand_disabled',False)
        self.virtual_angles=np.array(self.mapping['right']['initial_rad'],dtype=float)
        self.raw={};self.command_pubs={};self.feedback_subs=[];self.mode_state={}
        self.mode_pub=node.create_publisher(String,'/task/hardware_mode',LATCHED)
        self.mode_pub.publish(String(data='RIGHT HAND DISABLED: no right-hand RX/TX; its joint values are virtual. Arms and left hand are real.' if self.virtual_right else 'Both hands enabled'))
        self.profile_state={};self.last_external={};self.last_command={}
        self.profile_generation={"left":0,"right":0};self.feedback_seen={}
        self.feedback_publishers={};self.profile_reason={};self.pending_started=0.
        self.estop=False;self.stage=None
        self.tag=node.get_name()+"/leader_grip"
        self.validate_mapping()
        self.publisher=node.create_publisher(JointState,"/task/hand_state",qos_profile_sensor_data)
        enabled=node.get_parameter("hardware_enabled").value and not node.get_parameter("dry_run").value
        for side in ("left","right"):
            if side=="right" and self.virtual_right:continue
            cfg=self.mapping[side]
            self.feedback_subs += [
                node.create_subscription(HandCtlMessage,cfg["feedback_topic"],
                    lambda msg,s=side:self.receive_feedback(s,msg),qos_profile_sensor_data),
                node.create_subscription(HandUiMessage,f"/hand/{side}/ui_response",
                    lambda msg,s=side:self.receive_mode(s,msg),10),
                node.create_subscription(HandUiMessage,f"/hand/{side}/ui_request",
                    lambda msg,s=side:self.mode_requested(s),10),
                node.create_subscription(LeaderGrip,f"/hand/{side}/grip",self.observe_grip,qos_profile_sensor_data)]
            if enabled:
                self.command_pubs[side]=node.create_publisher(LeaderGrip,f"/hand/{side}/grip",qos_profile_sensor_data)
        self.feedback_subs.append(node.create_subscription(Bool,"/deux/estop",
            lambda msg:setattr(self,"estop",msg.data),10))
        # Both physical nodes currently share /deux_hand. Each response includes
        # hand_side, so an endpoint is never attributed by response order.
        self.client=node.create_client(GetParameters,"/deux_hand/get_parameters")
        self.feedback_subs.append(node.create_subscription(ParameterEvent,"/parameter_events",self.profile_event,10))
        self.pending=None
        self.profile_timer=node.create_timer(.2,self.refresh_profile)

    def validate_mapping(self):
        if self.mapping.get("protocol")!="leader_grip" or self.mapping.get("feedback_mapping_confirmed") is not True:
            raise ValueError("explicit scalar protocol and feedback mapping required")
        for side in ("left","right"):
            c=self.mapping[side];names=[n for n in HAND_NAMES if n.startswith(side)]
            if set(c["joints"])!=set(names) or sorted(m["id"] for m in c["joints"].values())!=list(range(1,8)):
                raise ValueError("exact measured motor ID mapping required")
            if c["feedback_type"]!="deux_hand_msgs/msg/HandCtlMessage":
                raise ValueError("LeaderGrip requires measured HandCtlMessage feedback")
            if type(c["leader_mode"]) is not int or not 1<=c["leader_mode"]<=255:
                raise ValueError("invalid leader mode")
            for key in ("initial_rad","target_rad"):
                vector(c[key],7)
            grip_fraction(c["initial_rad"],c["initial_rad"],c["target_rad"],1e-6)
            for m in c["joints"].values():
                if m["sign"] not in (-1,1) or not np.isfinite([m["offset"],m["min"],m["max"],m["velocity"]]).all() or m["min"]>=m["max"] or m["velocity"]<=0:
                    raise ValueError("invalid scalar feedback mapping")

    def invalidate_profile(self,side,reason='explicit invalidation'):
        if not hasattr(self,'profile_reason'):self.profile_reason={}
        self.profile_reason[side]=reason
        self.profile_state.pop(side,None)
        self.profile_generation[side]+=1

    def mode_requested(self,side):
        self.mode_state.pop(side,None)
        self.invalidate_profile(side,'mode request')

    def receive_mode(self,side,msg):
        old=self.mode_state.get(side)
        fields=("leader_mode","teleop_mode","run_flag","stop_flag")
        if old is not None and any(getattr(old[1],k)!=getattr(msg,k) for k in fields):
            self.invalidate_profile(side,'mode changed')
        self.mode_state[side]=(time.monotonic(),msg)

    def profile_event(self,msg):
        if msg.node.rsplit('/',1)[-1]!='deux_hand':return
        names=[p.name for p in (*msg.new_parameters,*msg.changed_parameters,*msg.deleted_parameters)]
        watched={"hand_side"}
        for side in ("left","right"):
            mode=self.mapping[side]['leader_mode']
            watched.update({f'leader_mode_{mode}.initial_deg',f'leader_mode_{mode}.target_deg'})
        if watched.intersection(names):
            # Both drivers share a node name; conservatively invalidate both.
            for side in ("left","right"):self.invalidate_profile(side,"driver parameter changed")

    def expire_disconnected_profiles(self):
        now=time.monotonic()
        for side in list(self.profile_state):
            if (now-self.feedback_seen.get(side,-1e9)>2.
                    or now-self.mode_state.get(side,(0,None))[0]>2.):
                self.invalidate_profile(side,'driver heartbeat absent >2s')

    def check_feedback_publishers(self):
        # Jazzy callback MessageInfo does not expose a publisher GID. Query graph
        # identities on the 5Hz profile timer, never on the control hot path.
        if not hasattr(self.node,'get_publishers_info_by_topic'):return
        for side in ('left','right'):
            if side=='right' and self.virtual_right:continue
            identities=tuple(sorted(bytes(info.endpoint_gid) for info in
                self.node.get_publishers_info_by_topic(self.mapping[side]['feedback_topic'])))
            if not identities:continue  # Sustained absence uses the heartbeat timeout.
            old=self.feedback_publishers.get(side)
            if old is not None and old!=identities:
                self.invalidate_profile(side,'feedback publisher restarted/changed')
            self.feedback_publishers[side]=identities

    def refresh_profile(self):
        self.check_feedback_publishers()
        self.expire_disconnected_profiles()
        sides=('left',) if self.virtual_right else ('left','right')
        if all(self.profile_state.get(s,(0,False))[1] for s in sides):return
        if self.pending is not None and not self.pending.done():
            if time.monotonic()-self.pending_started<=1.:return
            self.client.remove_pending_request(self.pending)
            self.pending.cancel();self.pending=None
        if not self.client.service_is_ready():return
        modes=sorted({self.mapping[s]["leader_mode"] for s in ("left","right")})
        names=["hand_side"]
        for mode in modes:
            names += [f"leader_mode_{mode}.initial_deg",f"leader_mode_{mode}.target_deg"]
        generation=dict(self.profile_generation)
        self.pending_started=time.monotonic()
        self.pending=self.client.call_async(GetParameters.Request(names=names))
        self.pending.add_done_callback(lambda future:self.receive_profile(names,future,generation))

    def receive_profile(self,names,future,generation=None):
        try:
            if hasattr(future,'cancelled') and future.cancelled():return
            values={k:parameter_value_to_python(v) for k,v in zip(names,future.result().values)}
            side=values["hand_side"]
            if side not in ("left","right"):return
            if generation is not None and generation[side]!=self.profile_generation[side]:return
            cfg=self.mapping[side];mode=cfg["leader_mode"]
            valid=all(np.asarray(values.get(f"leader_mode_{mode}.{key}_deg",[])).shape==(7,)
                and np.allclose(np.deg2rad(values[f"leader_mode_{mode}.{key}_deg"]),cfg[f"{key}_rad"],atol=1e-6,rtol=0)
                for key in ("initial","target"))
            previously_valid=self.profile_state.get(side,(0,False))[1]
            self.profile_state[side]=(time.monotonic(),valid)
            if not hasattr(self,'profile_reason'):self.profile_reason={}
            self.profile_reason[side]='verified' if valid else 'driver initial/target angles mismatch'
            if valid and not previously_valid:
                self.node.get_logger().info(f"{side} scalar profile verified and cached (no age expiry)")
        except Exception as error:
            self.node.get_logger().warning(f"hand profile read failed: {error}",throttle_duration_sec=5.)

    def observe_grip(self,msg):
        if msg.valid and msg.header.frame_id!=self.tag and msg.side in ("left","right"):
            self.last_external[msg.side]=time.monotonic()

    def receive_feedback(self,side,msg,publisher_gid=None):
        if side=="right" and self.virtual_right:return
        now=time.monotonic()
        # Profile parameters are static; sample freshness is checked separately.
        # Brief delayed/dropped feedback must not discard a verified mapping.
        if now-self.feedback_seen.get(side,now)>2.:
            self.invalidate_profile(side,'feedback reconnect after >2s')
        if publisher_gid:
            if not hasattr(self,'feedback_publishers'):self.feedback_publishers={}
            previous=self.feedback_publishers.get(side)
            if previous is not None and previous!=publisher_gid:
                self.invalidate_profile(side,'feedback publisher restarted/changed')
            self.feedback_publishers[side]=publisher_gid
        self.feedback_seen[side]=now
        if msg.side!=side or not all(m.torque for m in msg.motors):
            self.invalidate_profile(side,'wrong hand side or torque off')
            self.raw.pop(side,None);return
        self.raw.pop(side,None)
        self.feedback(side,msg)
        # Invalid/stale samples clear raw and still block dispatch, but do not
        # establish that driver profile parameters changed.
        if self.virtual_right and 'left' in self.raw:
            try:
                out=JointState();out.header=deepcopy(self.raw['left'][0])
                out.header.frame_id='left_measured_right_virtual'
                out.name=list(HAND_NAMES);out.position=self.measured_hand().tolist()
                self.publisher.publish(out)
            except ValueError:pass
        # Invalid/stale input must not leave a previous sample looking healthy.

    def measured_hand(self):
        if not self.virtual_right:return super().measured_hand()
        sample=self.raw.get('left')
        if sample is None or not current(sample[0].stamp,self.node.now(),self.node.config['max_joint_age']):
            raise ValueError('left hand feedback stale/missing; right hand explicitly disabled')
        left=[sample[1][n] for n in HAND_NAMES[:7]]
        return np.r_[left,self.virtual_angles]

    def guard(self):
        self.expire_disconnected_profiles()
        if self.estop:raise ValueError("hand e-stop active")
        if len(self.command_pubs)!=(1 if self.virtual_right else 2):raise ValueError("hand dispatch disabled")
        now=time.monotonic()
        for side in ("left","right"):
            if side=="right" and self.virtual_right:continue
            t,msg=self.mode_state.get(side,(0,None))
            if msg is None or now-t>2. or msg.leader_mode!=self.mapping[side]["leader_mode"] or msg.teleop_mode!=0 or not msg.run_flag or msg.stop_flag:
                raise ValueError(
                    f"{side} Leader mode not confirmed: "
                    f"received={None if msg is None else dict(leader_mode=msg.leader_mode, teleop_mode=msg.teleop_mode, run_flag=msg.run_flag, stop_flag=msg.stop_flag)}; "
                    f"age={now-t:.3f}s limit=2.000s; "
                    f"required_mode={self.mapping[side]['leader_mode']}, RUN, teleop=0"
                )
            t,valid=self.profile_state.get(side,(0,False))
            if not valid:raise ValueError(f"{side} scalar profile unverified/mismatched ({getattr(self, 'profile_reason', {}).get(side, 'not yet verified')}); waiting for driver profile confirmation")
            if now-self.last_external.get(side,-1e9)<1.:
                raise ValueError(f"{side} competing grip command received")
            if self.node.count_publishers(f"/hand/{side}/cmd"):
                raise ValueError(f"{side} competing direct hand command publisher")
            if self.command_pubs[side].get_subscription_count()==0:
                raise ValueError(f"{side} hand driver unavailable")
        self.measured_hand()

    def fractions(self,positions):
        values=vector(positions,14);out={}
        tolerance=min(float(self.node.config["tracking_tolerance"]),.15)
        for i,side in enumerate(("left","right")):
            c=self.mapping[side]
            out[side]=grip_fraction(values[i*7:(i+1)*7],c["initial_rad"],c["target_rad"],tolerance)
        if self.mapping.get("keep_left_closed",False) and self.stage!="Prepare":
            if np.max(np.abs(values[:7]-self.mapping["left"]["target_rad"]))>self.node.config["start_tolerance"]:
                raise ValueError("left scanner must remain closed outside Prepare")
            out["left"]=1.0
        return out

    def validate(self,t,q,stage=None):
        self.stage=stage
        self.guard()
        super().validate(t,q)
        if self.node.config.get("hand_command_only",False):
            # Fixed close/open commands take effect at phase start; the configured
            # phase duration is a dwell, not a measured arrival guarantee.
            self.binary_goal={}
            for i,side in enumerate(("left","right")):
                cfg=self.mapping[side];v=np.asarray(q[-1][i*7:(i+1)*7])
                self.binary_goal[side]=float(np.linalg.norm(v-cfg["target_rad"]) < np.linalg.norm(v-cfg["initial_rad"]))
            if stage in ('Prepare','ReadyPose','ObserveScene','Approach','Descend'):
                fraction=float(self.node.config.get('right_hand_ready_fraction',0.))
                if not np.isfinite(fraction) or not 0 <= fraction <= 1:raise ValueError('invalid right ready fraction')
                self.binary_goal['right']=fraction
            if stage=="Close":self.binary_goal["right"]=1.
            if stage=="Release":self.binary_goal["right"]=0.
            if self.mapping.get("keep_left_closed",False):self.binary_goal["left"]=1.
            if "left_hand_ready_fraction" in self.node.config:
                from .align_hand_pulse import stage_left_fraction
                self.binary_goal["left"] = stage_left_fraction(stage, float(self.node.config["left_hand_ready_fraction"]))
        else:
            for row in q:self.fractions(row)

    def command(self,positions, *, align_left_fraction=None, placement_right_fraction=None):
        self.guard()
        if self.node.config.get("hand_command_only",False):
            if not hasattr(self,"binary_goal"):raise ValueError("binary hand command requires validated plan")
            fractions=dict(self.binary_goal)
            if self.stage=="TrackGrasp":
                # Preserve the scheduled grasp phase. Never close at approach start.
                c=self.mapping["right"];v=np.asarray(positions[7:14])
                fractions["right"]=float(np.linalg.norm(v-c["target_rad"]) < np.linalg.norm(v-c["initial_rad"]))
        else:
            fractions=self.fractions(positions)
        if placement_right_fraction is not None:
            if self.stage!='PlaceApproach' or not self.node.config.get('placement',{}).get('continuous_motion',False) or placement_right_fraction not in (0.,1.):
                raise ValueError('invalid continuous placement hand command')
            fractions['right']=placement_right_fraction
        if align_left_fraction is not None:
            if self.stage != "AlignBarcode" or not np.isfinite(align_left_fraction) or not 0 <= align_left_fraction <= 1:
                raise ValueError("invalid alignment left hand fraction")
            fractions["left"] = float(align_left_fraction)
        for side,fraction in fractions.items():
            if side=='right' and self.virtual_right:
                cfg=self.mapping[side]
                self.virtual_angles=np.array(cfg['initial_rad'])+fraction*(np.array(cfg['target_rad'])-np.array(cfg['initial_rad']))
                continue
            msg=LeaderGrip();msg.header.stamp=self.node.get_clock().now().to_msg()
            msg.header.frame_id=self.tag;msg.side=side;msg.grasp=fraction;msg.valid=True
            self.command_pubs[side].publish(msg);self.last_command[side]=fraction

    def command_align_left_fraction(self, fraction):
        """Explicit partial opening only after the AlignBarcode arm motion."""
        if self.stage != "AlignBarcode" or not np.isfinite(fraction) or not 0 <= fraction <= 1:
            raise ValueError("invalid alignment left hand command")
        self.guard()
        msg = LeaderGrip()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.tag
        msg.side = "left"
        msg.grasp = float(fraction)
        msg.valid = True
        self.command_pubs["left"].publish(msg)
        self.last_command["left"] = float(fraction)

    def hold_measured(self):
        # Scalar feedback is not an inverse grasp command (contact/stall changes
        # angles). Reprojection would open a loaded hand. Driver holds last target.
        self.node.get_logger().warning("LeaderGrip dispatch stopped; driver retains last target, no open or torque-off sent")
