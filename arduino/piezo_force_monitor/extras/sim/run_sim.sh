#!/usr/bin/env bash
# Builds the host simulation of piezo_force_monitor.ino and runs every scenario.
# Usage: ./run_sim.sh            (prints a pass/fail line per scenario)
#        ./sim full              (one scenario with the full serial transcript)
set -euo pipefail
cd "$(dirname "$0")"
g++ -std=c++17 -O2 -Wall -Wextra -Istub sim_main.cpp -o sim
fail=0
for sc in full skip noeol inline; do
  if ./sim "$sc" quiet > "sim_$sc.log" 2>&1; then r=PASS; else r=FAIL; fail=1; fi
  echo "$sc: $r  ($(grep 'checks failed' "sim_$sc.log"))"
done
cal=$(grep -o 'CAL1 k=[^ ]* .*' sim_full.log | tail -1 | tr -d '\r')
if ./sim paste quiet "$cal" > sim_paste.log 2>&1; then r=PASS; else r=FAIL; fail=1; fi
echo "paste: $r  ($(grep 'checks failed' sim_paste.log))"
exit $fail
