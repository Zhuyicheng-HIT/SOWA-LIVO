#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${1:-$HOME/sowa_ws}"
estimator_dir="$workspace/src/estimator"
upstream_url="https://github.com/hku-mars/FAST-LIVO2.git"
upstream_commit="0d2c0346107b75b59934975adec9a6eeeb913c64"
release_tag="v0.1.0"
asset_name="sowa-livo-extension-v0.1.0-ubuntu20.04-amd64.tar.gz"
release_url="https://github.com/Zhuyicheng-HIT/SOWA-LIVO/releases/download/${release_tag}"

if [[ "$(uname -m)" != "x86_64" ]]; then
  printf 'This release targets Linux x86_64.\n' >&2
  exit 2
fi
if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  printf 'ROS Noetic is required. Use Ubuntu 20.04 or the Docker installer.\n' >&2
  exit 2
fi
source /opt/ros/noetic/setup.bash
set -u
command -v git >/dev/null
command -v curl >/dev/null
command -v catkin_make >/dev/null
sudo_cmd=()
if [[ "$EUID" -ne 0 ]]; then
  sudo_cmd=(sudo)
fi

"${sudo_cmd[@]}" apt-get update
"${sudo_cmd[@]}" apt-get install -y build-essential cmake git curl libeigen3-dev libopencv-dev libpcl-dev libboost-all-dev libomp-dev ros-noetic-geometry-msgs ros-noetic-nav-msgs ros-noetic-sensor-msgs ros-noetic-roscpp ros-noetic-rospy ros-noetic-std-msgs ros-noetic-message-generation ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-pcl-ros ros-noetic-eigen-conversions ros-noetic-tf ros-noetic-rviz

mkdir -p "$workspace/src"
if [[ ! -d "$workspace/src/rpg_vikit/.git" ]]; then
  git clone https://github.com/xuankuzcr/rpg_vikit.git "$workspace/src/rpg_vikit"
fi

if [[ -e "$estimator_dir" ]]; then
  if [[ ! -f "$estimator_dir/.sowa_extension_installed" ]]; then
    printf 'Refusing to overwrite existing path: %s\n' "$estimator_dir" >&2
    exit 3
  fi
else
  git clone "$upstream_url" "$estimator_dir"
  git -C "$estimator_dir" checkout --detach "$upstream_commit"
fi

estimator_package="$(sed -n 's/.*<name>\([^<]*\)<\/name>.*/\1/p' "$estimator_dir/package.xml" | head -n 1)"
if [[ -z "$estimator_package" ]]; then
  printf 'Cannot determine estimator package name.\n' >&2
  exit 3
fi

if [[ ! -f "$estimator_dir/.sowa_extension_installed" ]]; then
  tmp_dir="$(mktemp -d)"
  trap 'rm -rf "$tmp_dir"' EXIT

  git clone https://github.com/strasdat/Sophus.git "$tmp_dir/Sophus"
  git -C "$tmp_dir/Sophus" checkout a621ff
  cmake -S "$tmp_dir/Sophus" -B "$tmp_dir/Sophus/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$tmp_dir/Sophus/build" --parallel
  "${sudo_cmd[@]}" cmake --install "$tmp_dir/Sophus/build"

  curl --retry 3 --connect-timeout 30 -fL "$release_url/$asset_name" -o "$tmp_dir/$asset_name"
  curl --retry 3 --connect-timeout 30 -fL "$release_url/SHA256SUMS.txt" -o "$tmp_dir/SHA256SUMS.txt"
  (cd "$tmp_dir" && grep " $asset_name$" SHA256SUMS.txt | sha256sum -c -)
  mkdir -p "$estimator_dir/sowa_runtime"
  tar -xzf "$tmp_dir/$asset_name" -C "$estimator_dir/sowa_runtime"
  mkdir -p "$estimator_dir/sowa_adapter/include/sowa" "$estimator_dir/sowa_adapter/src"
  install -m 0644 "$repo_root/adapter/src/doa_axis_scheduler.cpp" "$estimator_dir/sowa_adapter/src/doa_axis_scheduler.cpp"
  install -m 0644 "$repo_root/adapter/include/sowa/compat_api.h" "$estimator_dir/sowa_adapter/include/sowa/compat_api.h"
  install -m 0644 "$repo_root/adapter/include/doa_axis_scheduler.h" "$estimator_dir/include/doa_axis_scheduler.h"
  install -m 0644 "$repo_root/config/sowa_livo_long_corridor.yaml" "$estimator_dir/config/sowa_livo_long_corridor.yaml"
  sed "s/@ESTIMATOR_PACKAGE@/$estimator_package/g" "$repo_root/launch/sowa_mapping.launch.in" > "$estimator_dir/launch/sowa_mapping.launch"
  git -C "$estimator_dir" apply --recount "$repo_root/patches/sensor-update-v076.patch"
  touch "$estimator_dir/.sowa_extension_installed"
fi

cd "$workspace"
catkin_make -DCMAKE_BUILD_TYPE=Release -j2
printf '%s\n' "$estimator_package" > "$workspace/.sowa-package"
printf '\nBuilt SOWA workspace at %s\n' "$workspace"
printf 'Launch with: bash %s/scripts/run.sh %s\n' "$repo_root" "$workspace"
