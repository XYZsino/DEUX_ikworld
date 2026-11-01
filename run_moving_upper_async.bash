#!/usr/bin/env bash
# Workspace tracking -> progressive descent -> right closure; GUI starts one trial.
set -eo pipefail
exec bash "$(dirname "${BASH_SOURCE[0]}")/run_real_grasp_trial.bash" "$@"
