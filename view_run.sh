#!/usr/bin/env bash
# Serve a recorded run to a browser over Tailscale (Chrome works best).
#   ./view_run.sh            pick a run from the list of runs in ~/ramp-runs (Enter = newest)
#   ./view_run.sh FILE.rrd   a specific run
# Ctrl-C to stop. For big runs it is faster to copy the file to a laptop and
# open it there:  pip install rerun-sdk==0.42.0 && rerun FILE.rrd
set -euo pipefail
here=$(dirname "$(readlink -f "$0")")
runs_dir=~/ramp-runs
if [ $# -gt 0 ]; then
  run=$1
  [ -f "$run" ] || run=$runs_dir/$1          # a bare file name means one in ~/ramp-runs
  [ -f "$run" ] || { echo "no such run: $1" >&2; exit 1; }
else
  mapfile -t runs < <(ls -t "$runs_dir"/*.rrd 2>/dev/null)
  [ ${#runs[@]} -gt 0 ] || { echo "no runs in $runs_dir" >&2; exit 1; }
  run=${runs[0]}
  if [ -t 0 ] && [ ${#runs[@]} -gt 1 ]; then  # asked from a terminal: show the list, newest first
    for i in "${!runs[@]}"; do
      printf '%3d) %s  %7s\n' $((i + 1)) "$(basename "${runs[i]}" .rrd)" "$(du -h "${runs[i]}" | cut -f1)"
    done
    while true; do
      read -rp "which run? [1 = newest] " n || exit 1
      n=${n:-1}
      if [[ $n =~ ^[0-9]+$ ]] && [ "$n" -ge 1 ] && [ "$n" -le ${#runs[@]} ]; then run=${runs[n - 1]}; break; fi
      echo "enter a number from 1 to ${#runs[@]}"
    done
  fi
fi
ip=$(tailscale ip -4 | head -1)
# The page (port 9090) fetches the data from a second port (9876), and Rerun only lets pages served
# from localhost do that unless told otherwise. Allow this machine's Tailscale address and names.
name=$(tailscale status --self --json | sed -n 's/.*"DNSName": *"\([^"]*\)\.".*/\1/p' | head -1)
allow=()
for host in "$ip" ${name:+"$name" "${name%%.*}"}; do allow+=(--cors-allow-origin "http://$host:9090"); done
echo "serving $run"
echo "open:  http://$ip:9090/?url=rerun%2Bhttp%3A%2F%2F$ip%3A9876%2Fproxy"
echo "       (use this address, not a localhost one: the url= part has to name this machine"
echo "        as your browser reaches it, or the page loads with no data)"
exec "$here/.venv/bin/rerun" --serve-web "${allow[@]}" "$run"
