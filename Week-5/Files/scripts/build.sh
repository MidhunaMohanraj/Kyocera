#!/usr/bin/env bash
# build.sh — build kyocera_pmd_init on the LX2160A target (NXP LSDK 2004).
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
SRC_DIR="$HERE/../src"
  
# Sanity: confirm DPDK headers and libs exist
[[ -f /usr/local/include/dpdk/rte_ethdev.h ]] || { echo "ERROR: DPDK headers not found at /usr/local/include/dpdk/"; exit 1; }
[[ -f /usr/local/lib/libdpdk.a ]] || { echo "ERROR: libdpdk.a not found at /usr/local/lib/"; exit 1; }

echo "DPDK headers: /usr/local/include/dpdk/"
echo "DPDK libs:    /usr/local/lib/"

cd "$SRC_DIR"  
make clean
make -j"$(nproc)"  
echo ""
echo "Built: $SRC_DIR/kyocera_pmd_init"
 
