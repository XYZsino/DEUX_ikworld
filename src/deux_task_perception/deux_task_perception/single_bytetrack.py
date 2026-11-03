"""One selected track using Ultralytics ByteTrack primitives; no ROS dependencies."""
from dataclasses import dataclass
import numpy as np
from ultralytics.trackers.byte_tracker import STrack
from ultralytics.trackers.basetrack import TrackState
from ultralytics.trackers.utils.kalman_filter import KalmanFilterXYAH
from ultralytics.trackers.utils import matching


@dataclass(frozen=True)
class Target:
    xyxy: tuple
    score: float
    class_id: int
    detection_index: int


class SingleByteTracker:
    """Owns at most one persistent track. Call update once per consecutive frame.

    select() consumes the initialization frame; update() starts on the NEXT frame.
    Missing targets return None; predictions are never reported as observations.
    No automatic initialization or replacement after expiry.
    """
    def __init__(self, high=.25, low=.1, match_thresh=.8, buffer=30, fuse_score=True):
        if not 0 <= low < high <= 1 or not 0 <= match_thresh <= 1:
            raise ValueError('invalid confidence or matching thresholds')
        if isinstance(buffer, bool) or not isinstance(buffer, int) or buffer < 0:
            raise ValueError('buffer must be a nonnegative frame count')
        self.high, self.low, self.match_thresh = high, low, match_thresh
        self.buffer, self.fuse_score = buffer, fuse_score
        self.reset()

    def reset(self):
        self.track = None
        self.predicted_xyxy = None
        self.observed_xyxy = None
        self.frame = 0
        self.status = 'idle'

    @staticmethod
    def _rows(detections):
        # Ultralytics Results or an Nx6 array: x1,y1,x2,y2,confidence,class.
        if hasattr(detections, 'boxes'):
            b = detections.boxes
            if b is None:
                raise ValueError('axis-aligned YOLO detection Results required')
            def array(value):
                return value.detach().cpu().numpy() if hasattr(value, "detach") else np.asarray(value)
            detections = np.column_stack((array(b.xyxy), array(b.conf), array(b.cls)))
        a = np.asarray(detections, dtype=float)
        if a.size == 0:
            return np.empty((0, 6))
        if a.ndim != 2 or a.shape[1] != 6 or not np.isfinite(a).all():
            raise ValueError('expected finite Nx6 detections')
        if np.any(a[:, 2:4] <= a[:, :2]) or np.any((a[:, 4] < 0) | (a[:, 4] > 1)):
            raise ValueError('invalid bbox or score')
        if np.any(a[:, 5] < 0) or np.any(a[:, 5] != np.floor(a[:, 5])):
            raise ValueError('class IDs must be nonnegative integers')
        return a

    @staticmethod
    def _candidate(row, index):
        xywh = np.r_[(row[:2] + row[2:4]) / 2, row[2:4] - row[:2], index]
        return STrack(xywh, float(row[4]), int(row[5]))

    def select(self, detections, index):
        rows = self._rows(detections)
        if not isinstance(index, (int, np.integer)) or not 0 <= index < len(rows):
            raise ValueError('select a valid detection index')
        return self.select_bbox(rows[index, :4], int(rows[index, 5]), rows[index, 4], index)

    def select_bbox(self, xyxy, class_id=0, score=1., detection_index=-1):
        """Initialize from a manually drawn box. Subsequent frames still need YOLO detections."""
        row = self._rows([list(xyxy) + [score, class_id]])[0]
        track = self._candidate(row, detection_index)
        track.next_id = lambda: 1  # no global ID counter shared with other instances
        track.activate(KalmanFilterXYAH(), 1)
        self.predicted_xyxy = None
        self.observed_xyxy = tuple(row[:4])
        self.track, self.frame, self.status = track, 1, 'tracked'
        return self._target()

    def _target(self):
        t = self.track
        return Target(tuple(float(v) for v in t.xyxy), float(t.score), int(t.cls), int(t.idx))

    def _match(self, candidates, threshold, fuse=False):
        if not candidates:
            return None
        costs = matching.iou_distance([self.track], candidates)
        if fuse:
            costs = matching.fuse_score(costs, candidates)
        # A one-row assignment is simply its lowest admissible cost.
        index = int(np.argmin(costs[0]))
        return candidates[index] if costs[0, index] <= threshold else None

    def update(self, detections):
        rows = self._rows(detections)
        self.predicted_xyxy = None
        self.observed_xyxy = None
        self.frame += 1
        t = self.track
        if t is None:
            return None
        # Expiry is terminal; stale identities cannot be resurrected.
        if t.state == TrackState.Lost and self.frame - t.end_frame > self.buffer:
            self.track, self.status = None, 'removed'
            return None
        t.predict()
        self.predicted_xyxy = tuple(float(v) for v in t.xyxy)
        high, low = [], []
        for i, row in enumerate(rows):
            if row[4] >= self.high:
                high.append(self._candidate(row, i))
            elif row[4] > self.low:
                low.append(self._candidate(row, i))
        match = self._match(high, self.match_thresh, self.fuse_score)
        if match is None and t.state == TrackState.Tracked:
            match = self._match(low, .5)
        if match is not None:
            self.observed_xyxy = tuple(rows[int(match.idx), :4])
            if t.state == TrackState.Tracked:
                t.update(match, self.frame)
            else:
                t.re_activate(match, self.frame, new_id=False)
            self.status = 'tracked'
            return self._target()
        t.mark_lost()
        self.status = 'lost'
        if self.frame - t.end_frame > self.buffer:
            self.track, self.status = None, 'removed'
        return None
