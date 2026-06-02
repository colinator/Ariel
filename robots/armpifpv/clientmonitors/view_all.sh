#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

host="${1:-192.168.0.195}"
width="${2:-640}"
height="${3:-480}"
fps="${4:-1}"

pids=()

cleanup() {
    for pid in "${pids[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
}

trap cleanup EXIT INT TERM

python -m robots.armpifpv.clientmonitors.view_camera_rgb "$host" "$width" "$height" "$fps" &
pids+=("$!")

python -m robots.armpifpv.clientmonitors.view_realsense_rgb "$host" "$width" "$height" "$fps" &
pids+=("$!")

"$script_dir/view_metrics.sh" &
pids+=("$!")

wait
