"""Bounded asynchronous capture; storage diagnostics never decide bag presence."""
import copy,json,threading,shutil
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from PIL import Image

class FrameArchive:
    def __init__(self,root,error_callback=None):
        self.root=Path(root);self.root.mkdir(parents=True,exist_ok=True)
        self.pool=ThreadPoolExecutor(max_workers=1);self.slots=threading.BoundedSemaphore(32)
        self.error=None;self.pending=0;self.saved=0;self.failed=0;self.dropped=0
        self.lock=threading.Lock();self.error_callback=error_callback;self.reported_error=None

    def failure(self,error,dropped=False):
        message=str(error)
        with self.lock:
            self.error=message;self.failed+=not dropped;self.dropped+=dropped
            notify=message!=self.reported_error;self.reported_error=message
        if notify and self.error_callback:
            try:self.error_callback(message)
            except Exception:pass

    def status(self):
        with self.lock:return dict(pending=self.pending,saved=self.saved,failed=self.failed,dropped=self.dropped,last_error=self.error)

    def save(self,request_id,stamp,rgb,depth,excluded,metadata):
        if not self.slots.acquire(blocking=False):
            self.failure('inspection archive queue full; frame dropped',dropped=True)
            raise RuntimeError('inspection archive queue full; frame dropped')
        folder=self.root/(''.join(c for c in request_id if c.isalnum() or c in '-_') or 'unknown')/str(stamp)
        with self.lock:self.pending+=1
        try:
            # Snapshot ownership before returning to the ROS callback. No filesystem I/O here.
            depth=depth.copy();excluded=None if excluded is None else excluded.copy()
            metadata=copy.deepcopy(metadata)
            metadata['capture_formats']=dict(rgb=None,overlay=None,depth='depth_m.npy',hand_mask='hand_excluded.png' if excluded is not None else None)
            def write():
                try:
                    if shutil.disk_usage(self.root).free<512*1024**2:raise RuntimeError('inspection archive disk space below 512 MiB')
                    folder.mkdir(parents=True,exist_ok=False)
                    np.save(folder/'depth_m.npy',depth,allow_pickle=False)
                    if excluded is not None:Image.fromarray(excluded.astype(np.uint8)*255).save(folder/'hand_excluded.png')
                    (folder/'decision.json').write_text(json.dumps(metadata,indent=2,allow_nan=False)+'\n')
                    with self.lock:self.saved+=1
                except Exception as exc:self.failure(exc)
                finally:
                    with self.lock:self.pending-=1
                    self.slots.release()
            self.pool.submit(write)
        except BaseException as exc:
            with self.lock:self.pending-=1
            self.slots.release();self.failure(exc);raise
        return str(folder)

    def ready(self):
        with self.lock:return self.pending==0

    def close(self):self.pool.shutdown(wait=True)
