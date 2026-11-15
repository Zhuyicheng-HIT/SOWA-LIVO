#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${1:-$HOME/sowa_fast_livo2_ws}"
image="sowa-livo-fast-livo2:noetic-v0.1.0"
workspace_name="$(basename "$workspace" | tr '[:upper:]' '[:lower:]' | tr -cs 'a-z0-9_.-' '-')"
container="sowa-fast-livo2-${workspace_name}"

command -v docker >/dev/null || {
  printf 'Docker is required for the WSL/Windows deployment path.\n' >&2
  exit 2
}

mkdir -p "$workspace"
docker build -t "$image" -f "$repo_root/docker/Dockerfile" "$repo_root"
if ! docker container inspect "$container" >/dev/null 2>&1; then
  docker run -d --name "$container" --restart unless-stopped \
    --volume "$workspace:/workspace" \
    --volume "$repo_root:/sowa:ro" \
    --workdir /sowa \
    "$image" sleep infinity
else
  docker start "$container" >/dev/null
fi
docker exec "$container" bash -lc 'source /opt/ros/noetic/setup.bash && bash /sowa/scripts/deploy_fast_livo2.sh /workspace'

printf '\nBuilt SOWA-integrated FAST-LIVO2 in %s\n' "$workspace"
printf 'Run it with: docker exec -it %s bash -lc "source /opt/ros/noetic/setup.bash && source /workspace/devel/setup.bash && roslaunch fast_livo sowa_mapping_avia.launch"\n' "$container"
