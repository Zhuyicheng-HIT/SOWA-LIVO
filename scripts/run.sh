#!/usr/bin/env bash
set -eo pipefail

workspace="${1:-$HOME/sowa_ws}"
if [[ $# -gt 0 ]]; then shift; fi
if [[ ! -f "$workspace/.sowa-package" || ! -f "$workspace/devel/setup.bash" ]]; then
  printf 'Run scripts/install.sh for this workspace first: %s\n' "$workspace" >&2
  exit 2
fi
source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"
set -u
estimator_package="$(cat "$workspace/.sowa-package")"
exec roslaunch "$estimator_package" sowa_mapping.launch "$@"
