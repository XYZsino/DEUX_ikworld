#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
.venv/bin/python - <<'PY'
from pathlib import Path
p=Path('src/deux_task_bringup/config/installation.yaml')
s=p.read_text()
assert 'temporary_axis_grasp_trial:' in s
import re
s=re.sub(r'^temporary_axis_grasp_trial:.*$', 'temporary_axis_grasp_trial: false', s, flags=re.M)
p.write_text(s)
print('Axis grasp trial disabled. Restart BT while idle to load the setting. No robot command sent.')
PY
