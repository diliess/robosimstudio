#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
./ctl up --port "${PORT:-8081}"
./ctl status --wait
./ctl resume
./ctl play 'Welding controls' --wait
