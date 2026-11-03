from deux_task_msgs.msg import BoundingBox
from deux_task_perception.tracked_detection_node import single_object_slots

def test_changing_or_missing_tracker_id_uses_same_slot():
    for identity in ('visual-7', 'visual-30', ''):
        result=single_object_slots([BoundingBox(class_name='bread_bag',track_id=identity)],'bread_bag','barcode')
        assert result[0].track_id=='single-bag'

def test_ambiguous_or_absent_bag_has_no_target():
    for count in (0,2):
        assert single_object_slots([BoundingBox(class_name='bread_bag') for _ in range(count)],'bread_bag','barcode')==[]
