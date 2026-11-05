#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
./ctl reload
./ctl status --wait
./ctl play 'Welding controls' --wait
