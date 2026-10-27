"""Optional YOLO backend, also usable for offline recorded-image checks."""
from hashlib import sha256
from pathlib import Path
import numpy as np


class YoloDetector:
    def __init__(self, path, device, confidence):
        # Import is optional for observation/ICP users; no pip installs in callbacks.
        from ultralytics import YOLO
        self.model = YOLO(path)
        self.device, self.confidence = device, confidence
        self.identity = 'yolo-sha256:' + sha256(Path(path).read_bytes()).hexdigest()

    def __call__(self, rgb):
        # Ultralytics numpy sources are BGR; Observation.rgb is RGB.
        results = self.model.predict(source=np.ascontiguousarray(rgb[:, :, ::-1]),
                                     device=self.device, imgsz=640, conf=self.confidence,
                                     verbose=False, save=False)
        return [(int(b.cls.item()), self.model.names[int(b.cls.item())],
                 float(b.conf.item()), b.xyxy[0].cpu().tolist())
                for b in results[0].boxes]


class YoloTracker(YoloDetector):
    """Image-only ByteTrack IDs with acquisition-time expiry.

No simulator names or object transforms are accepted. A reappearing raw tracker
ID after the expiry gets a new public ID, so a task cannot silently reacquire a
previously lost target. Only boxes observed in the current frame are returned.
    """
    def __init__(self, path, device='cpu', confidence=.25, max_gap_s=.75):
        super().__init__(path, device, confidence)
        self.max_gap_s = float(max_gap_s)
        if not np.isfinite(self.max_gap_s) or self.max_gap_s <= 0:
            raise ValueError('max_gap_s must be positive and finite')
        self.previous_stamp = None
        self.identities = {}
        self.serial = 0

    def track(self, rgb, acquisition_s):
        stamp = float(acquisition_s)
        if not np.isfinite(stamp): raise ValueError('invalid acquisition time')
        if self.previous_stamp is not None and stamp <= self.previous_stamp:
            self.identities.clear()
            if self.model.predictor is not None:
                for tracker in getattr(self.model.predictor, 'trackers', []): tracker.reset()
        self.previous_stamp = stamp
        result = self.model.track(source=np.ascontiguousarray(rgb[:,:,::-1]),
            persist=True, tracker='bytetrack.yaml', device=self.device, imgsz=640,
            conf=self.confidence, verbose=False, save=False)[0]
        output = []
        for index, box in enumerate(result.boxes):
            if box.id is None: continue
            raw = int(box.id.item()); cls = int(box.cls.item())
            key = (cls, raw)
            old = self.identities.get(key)
            if old is None or stamp-old[1] > self.max_gap_s:
                self.serial += 1
                identity = f'visual-{self.serial}'
            else: identity = old[0]
            self.identities[key] = (identity, stamp)
            output.append(dict(track_id=identity, class_id=cls, class_name=self.model.names[cls],
                confidence=float(box.conf.item()), xyxy=box.xyxy[0].cpu().tolist(), acquired_s=stamp,
                polygon_xy=result.masks.xy[index].astype(float).ravel().tolist() if result.masks is not None else []))
        self.identities = {k:v for k,v in self.identities.items() if stamp-v[1] <= self.max_gap_s}
        return output
