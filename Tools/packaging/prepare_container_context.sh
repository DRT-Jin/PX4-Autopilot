#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "$script_dir/../.." && pwd)"
context_dir="${1:-docker-context}"

mkdir -p "$context_dir/px4_msgs/msg" "$context_dir/px4_msgs/srv"
cp "$script_dir"/Dockerfile.* "$script_dir"/*entrypoint.sh "$script_dir/px4-network.sh" "$context_dir/"
cp "$script_dir/ros2-install-dependencies.sh" "$context_dir/"

# Refresh generated definitions when reusing a context; keep downloaded .debs.
rm -f "$context_dir/px4_msgs/msg/"*.msg "$context_dir/px4_msgs/srv/"*.srv
cp "$repo_dir"/msg/*.msg "$repo_dir"/msg/versioned/*.msg "$context_dir/px4_msgs/msg/"
cp "$repo_dir"/srv/*.srv "$context_dir/px4_msgs/srv/"
