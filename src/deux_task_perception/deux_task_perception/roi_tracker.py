"""Fixed-ROI entry selection without persistent tracks for other detections."""
import numpy as np
import cv2
from .single_bytetrack import SingleByteTracker


class RoiBagTracker:
    def __init__(self, roi, class_ids, require_entry=True, **kwargs):
        self.require_entry = bool(require_entry)
        self.roi = np.asarray(roi, dtype=float)
        if self.roi.shape == (4,):
            x1, y1, x2, y2 = self.roi
            if x2 <= x1 or y2 <= y1:
                raise ValueError('invalid ROI')
            self.roi = np.array([[x1,y1],[x2,y1],[x2,y2],[x1,y2]])
        if (self.roi.shape != (4, 2) or not np.isfinite(self.roi).all()
                or not cv2.isContourConvex(self.roi.astype(np.float32))
                or cv2.contourArea(self.roi.astype(np.float32)) <= 1):
            raise ValueError('ROI must be a convex quadrilateral in boundary order')
        self.class_ids = set(class_ids)
        self.tracker = SingleByteTracker(**kwargs)
        self.previous = None

    def inside(self, box):
        center = (np.asarray(box[:2]) + box[2:4]) / 2
        return cv2.pointPolygonTest(self.roi.astype(np.float32), tuple(map(float, center)), False) >= 0

    def update(self, result):
        rows = SingleByteTracker._rows(result)
        rows = rows[np.isin(rows[:, 5], list(self.class_ids))]
        previous, self.previous = self.previous, rows.copy()
        if self.tracker.track is not None:
            target = self.tracker.update(rows)
            if target is not None and not self.inside(rows[target.detection_index, :4]):
                self.tracker.reset()
                return None
            return target
        if not self.require_entry:
            inside = [i for i,row in enumerate(rows) if self.inside(row[:4])]
            if inside:
                chosen = max(inside, key=lambda i: (rows[i,4], -i))
                return self.tracker.select(rows, chosen)
            return None
        if previous is None:
            return None
        # Greedy one-frame association detects boundary crossings only; no IDs,
        # velocities or Kalman states are maintained for unselected objects.
        pairs = []
        for i, row in enumerate(rows):
            for j, old in enumerate(previous):
                if row[5] != old[5]:
                    continue
                wh = np.maximum(0, np.minimum(row[2:4], old[2:4])-np.maximum(row[:2], old[:2]))
                intersection = np.prod(wh)
                union = np.prod(row[2:4]-row[:2])+np.prod(old[2:4]-old[:2])-intersection
                iou = intersection / union
                if iou > .1:
                    pairs.append((-iou, i, j))
        used, used_old, entries = set(), set(), []
        for _, i, j in sorted(pairs):
            if i in used or j in used_old:
                continue
            used.add(i); used_old.add(j)
            if self.inside(rows[i, :4]) and not self.inside(previous[j, :4]):
                entries.append(i)
        if entries:
            return self.tracker.select(rows, min(entries))
        return None
