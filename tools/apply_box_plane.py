from pathlib import Path
import json,yaml,numpy as np
from PIL import Image,ImageDraw
from deux_task_perception.perception_core import filter_above_plane
out=Path('.setup/box-plane-current');report=json.loads((out/'plane.json').read_text());assert report['accepted']
p=Path('tracked_perception.thor.yaml');text=p.read_text();(out/'tracked_perception.before.yaml').write_text(text);c=yaml.safe_load(text);cfg=c['perception_node']['ros__parameters'];oldplane=cfg['table_plane_base'];oldclear=cfg['table_plane_clearance_m'];plane=report['normal']+[report['offset_m']]
cfg.update(table_plane_filter_enabled=True,table_plane_provisional=True,table_plane_base=plane,table_plane_clearance_m=.07)
data=np.load(out/'capture.npz');z=np.nanmedian(data['depths'],axis=0);P=data['projection'].reshape(3,4);bc=data['base_from_camera'];mask=Image.new('L',(640,360));draw=ImageDraw.Draw(mask)
for poly in report['roi_polygons']:draw.polygon([tuple(x) for x in poly],fill=1)
v,u=np.nonzero(np.asarray(mask)&np.isfinite(z));depth=z[v,u];points=np.column_stack([(u-P[0,2])*depth/P[0,0],(v-P[1,2])*depth/P[1,1],depth]);a={};b={}
filter_above_plane(points,bc,oldplane,oldclear,a);kept=filter_above_plane(points,bc,plane,.07,b)
if len(kept):assert np.all((kept@bc[:3,:3].T+bc[:3,3])@np.array(plane[:3])+plane[3]>.07)
summary=dict(before_plane=oldplane,before_clearance_m=oldclear,after_plane=plane,after_clearance_m=.07,box_surface_before=a,box_surface_after=b)
(out/'comparison.json').write_text(json.dumps(summary,indent=2));p.write_text(yaml.safe_dump(c,sort_keys=False));print(json.dumps(summary,indent=2))
