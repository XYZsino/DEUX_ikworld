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
