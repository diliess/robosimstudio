#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ -n "${UV:-}" ]]; then
    scene_uv="$UV"
elif command -v uv >/dev/null 2>&1; then
    scene_uv="$(command -v uv)"
else
    scene_uv="$HOME/.local/bin/uv"
fi
if [[ ! -x "$scene_uv" ]]; then
    echo 'uv is missing. Install uv, or set UV to its executable path.' >&2
    exit 1
fi
if [[ ! -x .venv-scene/bin/python ]]; then
    "$scene_uv" --no-config venv --python 3.12 .venv-scene
fi
"$scene_uv" --no-config pip sync --python .venv-scene/bin/python --default-index https://pypi.org/simple --require-hashes requirements-scene.lock.txt
.venv-scene/bin/python scene_project.py check
