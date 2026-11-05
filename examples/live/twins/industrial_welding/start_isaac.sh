#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export OMNI_KIT_ACCEPT_EULA="${OMNI_KIT_ACCEPT_EULA:-YES}"
isaac_python="${ISAAC_PYTHON:-$PWD/.venv-isaacsim/bin/python}"
if [[ ! -x "$isaac_python" ]]; then
    echo 'Isaac environment missing. Run bash install_isaac.sh, or set ISAAC_PYTHON.' >&2
    exit 1
fi
exec "$isaac_python" -u isaac_view.py "$@"
