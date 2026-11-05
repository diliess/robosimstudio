#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
UV="${UV:-$(command -v uv || true)}"
UV="${UV:-$HOME/.local/bin/uv}"
if [[ ! -x "$UV" ]]; then
    echo 'uv is missing. Install uv, or set UV to its executable path.' >&2
    exit 1
fi
if [[ ! -x .venv-isaacsim/bin/python ]]; then
    "$UV" --no-config venv --python 3.12 .venv-isaacsim
fi
"$UV" --no-config pip install --python .venv-isaacsim/bin/python 'isaacsim[all,extscache]==6.2.0.0' 'torch==2.11.0' \
    --extra-index-url https://pypi.nvidia.com \
    --extra-index-url https://download.pytorch.org/whl/cu130 \
    --index-strategy unsafe-best-match
