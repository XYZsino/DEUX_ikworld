#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
python3 - <<'PY'
from pathlib import Path
import os,signal
p=Path('.setup/icp-live-test/pids')
if p.exists():
 for value in p.read_text().split():
  try:
   cmd=Path('/proc/'+value+'/cmdline').read_bytes()
   if b'tracking_icp_monitor' in cmd or b'tracked_perception_node' in cmd or b'grasp_preview_node' in cmd:
    os.killpg(int(value),signal.SIGTERM)
  except (OSError,ValueError):pass
print('Perception test stopped; GUI retained. No robot commands sent.')
PY
