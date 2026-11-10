#!/usr/bin/env bash
# Serve a recorded run to a browser over Tailscale (Chrome works best).
#   ./view_run.sh            newest run in ~/ramp-runs
#   ./view_run.sh FILE.rrd   a specific run
# Ctrl-C to stop. For big runs it is faster to copy the file to a laptop and
# open it there:  pip install rerun-sdk==0.42.0 && rerun FILE.rrd
set -euo pipefail
here=$(dirname "$(readlink -f "$0")")
run=${1:-$(ls -t ~/ramp-runs/*.rrd | head -1)}
ip=$(tailscale ip -4 | head -1)
echo "serving $run"
echo "open:  http://$ip:9090/?url=rerun%2Bhttp%3A%2F%2F$ip%3A9876%2Fproxy"
exec "$here/.venv/bin/rerun" --serve-web "$run"
