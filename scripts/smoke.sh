#!/usr/bin/env bash
# End-to-end smoke test: simulator + real host binary + HTTP API.
# Requires `cargo build --workspace` first.
set -euo pipefail

free_port() {
    python3 -c 'import socket; s = socket.socket(); s.bind(("127.0.0.1", 0)); print(s.getsockname()[1]); s.close()'
}

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_PORT="${GSE_SMOKE_SIM_PORT:-$(free_port)}"
HTTP_PORT="${GSE_SMOKE_HTTP_PORT:-$(free_port)}"
WORK="$(mktemp -d)"
SIM_PID=""
APP_PID=""

cleanup() {
    status=$?
    if [[ -n "$APP_PID" ]]; then kill "$APP_PID" 2>/dev/null || true; fi
    if [[ -n "$SIM_PID" ]]; then kill "$SIM_PID" 2>/dev/null || true; fi
    if [[ $status -ne 0 ]]; then
        echo "--- simulator log ---" >&2
        tail -30 "$WORK/sim.log" >&2 || true
        echo "--- app log ---" >&2
        tail -60 "$WORK/app.log" >&2 || true
    fi
    rm -rf "$WORK"
    exit $status
}
trap cleanup EXIT

"$ROOT/target/debug/gse-sim" --listen "127.0.0.1:$SIM_PORT" --latency 10 \
    >"$WORK/sim.log" 2>&1 &
SIM_PID=$!

XDG_STATE_HOME="$WORK/state" \
    "$ROOT/target/debug/gse-app" --port "tcp:127.0.0.1:$SIM_PORT" --no-browser \
    --http-port "$HTTP_PORT" --config "$WORK/config.json" >"$WORK/app.log" 2>&1 &
APP_PID=$!

for _ in $(seq 1 100); do
    if curl -sf "http://127.0.0.1:$HTTP_PORT/api/health" >/dev/null; then
        break
    fi
    sleep 0.1
done

TOKEN="$(curl -sf "http://127.0.0.1:$HTTP_PORT/" | sed -n 's/.*name="gse-token" content="\([a-f0-9]*\)".*/\1/p')"
if [[ -z "$TOKEN" ]]; then
    echo "could not read the session token from the console page" >&2
    exit 1
fi

python3 "$ROOT/scripts/smoke.py" "http://127.0.0.1:$HTTP_PORT" "$TOKEN"
