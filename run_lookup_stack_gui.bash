#!/usr/bin/env bash
# Same real repeating lookup startup, with the loopback web GUI included.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
exec bash run_lookup_stack.bash "$@" --gui
