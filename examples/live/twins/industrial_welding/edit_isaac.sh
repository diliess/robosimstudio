#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
# Save GUI edits in this small persistent layer, preserving the generated base.
exec bash start_isaac.sh --usd exports/usd/industrial_welding_edit.usda --editable-stage "$@"
