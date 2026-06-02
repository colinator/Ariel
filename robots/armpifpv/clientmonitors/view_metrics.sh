#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

broker_bind="${1:-0.0.0.0}"
broker_connect="${2:-127.0.0.1}"
broker_port="${3:-1883}"
metrics_topic="${4:-metrics}"

python_bin="${PYTHON_BIN:-$script_dir/.pyvenv/bin/python}"
if [[ ! -x "$python_bin" ]]; then
    python_bin="${PYTHON_BIN:-python}"
fi

if ! command -v mosquitto >/dev/null 2>&1; then
    echo "mosquitto not found on PATH" >&2
    exit 1
fi

mosquitto_config="$(mktemp "${TMPDIR:-/tmp}/armpifpv-mosquitto.XXXXXX.conf")"
mosquitto_pid=""

cleanup() {
    if [[ -n "$mosquitto_pid" ]]; then
        kill "$mosquitto_pid" 2>/dev/null || true
        wait "$mosquitto_pid" 2>/dev/null || true
    fi
    rm -f "$mosquitto_config"
}

trap cleanup EXIT INT TERM

cat > "$mosquitto_config" <<EOF
listener $broker_port $broker_bind
allow_anonymous true
EOF

echo "Starting mosquitto on $broker_bind:$broker_port"
mosquitto -c "$mosquitto_config" &
mosquitto_pid="$!"

sleep 0.5

echo "Starting MetricsTelevision for $broker_connect:$broker_port/$metrics_topic"
"$python_bin" -m roboflex.imgui.metrics_central \
    --broker_address "$broker_connect" \
    --broker_port "$broker_port" \
    --metrics_topic "$metrics_topic"
