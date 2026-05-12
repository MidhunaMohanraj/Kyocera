#!/usr/bin/env bash
# run.sh — launch kyocera_pmd_init on the LX2160A target.
# Requires DPRC env var and 1 GiB hugepages from Task 2.

set -euo pipefail

: "${DPRC:?ERROR: export DPRC=dprc.N first (see Task_2_Build_Environment/scripts/setup_dprc.sh)}"

HERE="$(cd "$(dirname "$0")" && pwd)"
APP="$HERE/../src/kyocera_pmd_init"
[[ -x "$APP" ]] || APP="$HERE/../src/build/kyocera_pmd_init"
if [[ ! -x "$APP" ]]; then
    echo "ERROR: kyocera_pmd_init binary not found. Run scripts/build.sh first." >&2
    exit 1
fi  

LCORES="${LCORES:-0-3}"
PREFIX="${PREFIX:-kpmd}"
LOG_LEVEL="${LOG_LEVEL:-lib.eal:info,bus.fslmc:info,pmd.net.dpaa2:info,user1:info}"

LOGDIR="$HERE/../logs"
mkdir -p "$LOGDIR"
LOG="$LOGDIR/run_$(date +%Y%m%d_%H%M%S).log"

echo "== kyocera_pmd_init =="
echo "DPRC=$DPRC  LCORES=$LCORES  PREFIX=$PREFIX"
echo "Log -> $LOG"
echo

# shellcheck disable=SC2024
sudo -E stdbuf -oL "$APP" \
    -l "$LCORES" \
    -n 1 \
    --file-prefix="$PREFIX" \
    --log-level="$LOG_LEVEL" \
    2>&1 | tee "$LOG"
