"""Reproducible YOLO11n training and held-out synthetic evaluation."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--model',default='yolo11n.pt',choices=['yolo11n.pt','yolo11n-seg.pt'])
    p.add_argument('--epochs',type=int,default=60)
    a=p.parse_args();a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=True)
    from ultralytics import YOLO, __version__
    model=YOLO(str(a.output/a.model))
    model.train(data=str(a.dataset.resolve()),project=str(a.output),name='detector',
        epochs=a.epochs,imgsz=640,batch=32,device=0,workers=4,seed=8301,
        deterministic=True,patience=15,close_mosaic=10,amp=False,
        degrees=15,fliplr=.5,flipud=0.,cache=False,exist_ok=False)
    best=Path(model.trainer.best)
    trained=YOLO(str(best))
    test=trained.val(data=str(a.dataset.resolve()),split='test',imgsz=640,batch=32,
        device=0,project=str(a.output),name='heldout',plots=True)
    report=dict(ultralytics=__version__,architecture=a.model.removesuffix('.pt'),tracker='ByteTrack',
        weights=str(best),weights_sha256=hashlib.sha256(best.read_bytes()).hexdigest(),
        dataset=str(a.dataset.resolve()),dataset_sha256=hashlib.sha256(a.dataset.read_bytes()).hexdigest(),
        manifest_sha256=hashlib.sha256(next(p for p in [a.dataset.parent/'manifest.json',a.dataset.parent.parent/'manifest.json'] if p.exists()).read_bytes()).hexdigest(),
        heldout=test.results_dict,classes=trained.names,scope='synthetic held-out scenes; not real-camera validation')
    (a.output/'training-result.json').write_text(json.dumps(report,indent=2))

if __name__=='__main__':main()
