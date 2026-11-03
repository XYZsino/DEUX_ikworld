"""Fresh scan-image bbox presence; pixel areas refer to the original RGB."""
import math


def matching_boxes(message, request_id, arrival, now, max_age, bag_area, barcode_area,
                   roi=None, image_size=None, min_confidence=.5):
    # Fail closed without an explicit scan ROI and the detector image dimensions.
    if roi is None or image_size is None or min(image_size)<=0:
        return []
    width,height=image_size
    polygon=[(x*width/roi['width'],y*height/roi['height']) for x,y in roi['points']]
    stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
    if message.request_id != request_id or stamp < arrival or not 0 <= now-stamp <= max_age:
        return []
    matches = []
    for box in message.boxes:
        confidence=float(box.confidence)
        if not math.isfinite(confidence) or not min_confidence<=confidence<=1.:continue
        threshold = {'bread_bag': bag_area, 'barcode': barcode_area}.get(box.class_name)
        if threshold is None:
            continue
        x0, y0, x1, y1 = box.xyxy
        if not all(math.isfinite(v) for v in (x0, y0, x1, y1)) or x1 <= x0 or y1 <= y0:
            continue
        if x0<0 or y0<0 or x1>width or y1>height:
            continue
        cx,cy=(x0+x1)/2,(y0+y1)/2
        cross=[(b[0]-a[0])*(cy-a[1])-(b[1]-a[1])*(cx-a[0])
               for a,b in zip(polygon,polygon[1:]+polygon[:1])]
        if not (all(v>=-1e-8 for v in cross) or all(v<=1e-8 for v in cross)):
            continue
        area = (x1-x0)*(y1-y0)
        if area >= threshold:
            matches.append(dict(class_name=box.class_name, confidence=confidence, area_px2=area, xyxy=list(box.xyxy),
                                track_id=box.track_id, stamp_s=stamp,polygon_xy=list(getattr(box,'polygon_xy',[]))))
    return matches
