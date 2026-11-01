import numpy as np
from builtin_interfaces.msg import Time
from deux_task_msgs.msg import ObjectDetectionArray,ObjectDetection
from deux_task_gui.rgb_view import predict_preview

def test_prediction_caps_age_preserves_original_and_uses_velocity():
 obj=ObjectDetectionArray();obj.header.stamp=Time(sec=10)
 d=ObjectDetection(track_id='a',velocity_valid=True);d.pose.pose.orientation.w=1.;d.velocity.linear.x=.1;d.velocity.angular.z=.2;obj.detections=[d]
 out,m=predict_preview(obj,'a',Time(sec=10,nanosec=500000000));assert abs(out.detections[0].pose.pose.position.x-.05)<1e-9;assert not m['prediction_stale'];assert obj.detections[0].pose.pose.position.x==0
 out,m=predict_preview(obj,'a',Time(sec=12));assert abs(out.detections[0].pose.pose.position.x-.075)<1e-9;assert m['prediction_stale'];assert m['prediction_age_s']==2
 d.velocity_valid=False;obj.detections=[d];out,m=predict_preview(obj,'a',Time(sec=11));assert out.detections[0].pose.pose.position.x==0 and m['prediction_dt_s']==0
