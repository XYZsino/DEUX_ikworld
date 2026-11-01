"""Stop only this launcher's idle preview process; preserve camera and perception."""
import json
import os
from pathlib import Path
import signal

root = Path(__file__).resolve().parents[1]
out = root/'.setup/grid-grasp-test'; record_file = out/'processes.json'
records = json.loads(record_file.read_text())
preview = json.loads((out/'preview.json').read_text())
if not preview.get('preview_only') or preview.get('motion_requested'):
    raise RuntimeError('only an idle preview can be stopped; cancel active Actions explicitly')
record = records.get('grid-preview')
if record:
    try:
        command = Path(f'/proc/{record["pid"]}/cmdline').read_bytes()
        if b'tools/grid_grasp_trial.py' not in command: raise RuntimeError('recorded PID identity changed')
        os.kill(record['pid'], signal.SIGTERM)
    except (FileNotFoundError, ProcessLookupError): pass
    records.pop('grid-preview'); record_file.write_text(json.dumps(records, indent=2))
print('Grid preview stopped; camera/perception/monitor preserved. No hardware commands sent.')
