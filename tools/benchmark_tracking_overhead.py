"""Offline overhead measurements; no middleware nodes or hardware clients."""
import sys,json,time,importlib.util
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path('tools').resolve()))
from async_upper import spline
from progressive_descent import next_profile
from deux_task_core.planning import scene_version
from deux_task_core.configuration import load_configuration
root=Path('.setup/tracking-workspace-update');p=json.loads((root/'inside-plan.json').read_text());c,_=load_configuration('src/deux_task_bringup/config/installation.yaml')
def bench(f,n):
 t=[]
 for _ in range(n):
  start=time.perf_counter();f();t.append((time.perf_counter()-start)*1000)
 return dict(median_ms=float(np.median(t)),p95_ms=float(np.quantile(t,.95)),n=n)
curve=spline(p)
spec=importlib.util.spec_from_file_location('old_profile',root/'before/progressive_descent.py');old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
out=dict(spline_rebuild_each_tick=bench(lambda:spline(p)(.5),300),cached_spline_tick=bench(lambda:curve(.5),300),profile_before=bench(lambda:old.next_profile({},0.,.8,.08,True),100),profile_after=bench(lambda:next_profile({},0.,.8,.08,True),100),full_config_and_urdf_hash=bench(lambda:scene_version(c),100))
a=old.next_profile({},0.,.8,.08,True);b=next_profile({},0.,.8,.08,True);assert a==b
(root/'overhead.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
