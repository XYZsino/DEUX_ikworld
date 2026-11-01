"""Fresh scan-image bbox presence; pixel areas refer to the original RGB."""
import math


def matching_boxes(message, request_id, arrival, now, max_age, bag_area, barcode_area):
    stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
    if message.request_id != request_id or stamp < arrival or not 0 <= now-stamp <= max_age:
        return []
    matches = []
    for box in message.boxes:
        threshold = {'bread_bag': bag_area, 'barcode': barcode_area}.get(box.class_name)
        if threshold is None:
            continue
        x0, y0, x1, y1 = box.xyxy
        if not all(math.isfinite(v) for v in (x0, y0, x1, y1)) or x1 <= x0 or y1 <= y0:
            continue
        area = (x1-x0)*(y1-y0)
        if area >= threshold:
            matches.append(dict(class_name=box.class_name, area_px2=area, xyxy=list(box.xyxy),
                                track_id=box.track_id, stamp_s=stamp))
    return matches
