#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${1:-$HOME/sowa_ws}"
image="sowa:noetic-v0.1.0"
workspace_name="$(basename "$workspace" | tr '[:upper:]' '[:lower:]' | tr -cs 'a-z0-9_.-' '-')"
container="sowa-${workspace_name}"

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
docker exec "$container" bash -lc 'source /opt/ros/noetic/setup.bash && bash /sowa/scripts/install.sh /workspace'

printf '\nBuilt SOWA workspace in %s\n' "$workspace"
printf 'Run it with: docker exec -it %s bash -lc "bash /sowa/scripts/run.sh /workspace"\n' "$container"
