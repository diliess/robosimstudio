#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
scene_python="${SCENE_PYTHON:-$PWD/.venv-scene/bin/python}"
if [[ ! -x "$scene_python" ]]; then
    echo 'Scene environment missing. Run: bash install_scene.sh' >&2
    exit 1
fi
exec "$scene_python" scene_project.py "$@"
