#!/bin/sh

set -eu

WORKSPACE_DIR="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
ASKPASS="${WORKSPACE_DIR}/.vscode/sudo-askpass.sh"
LLDB_MI=""

for candidate in "${HOME}"/.vscode/extensions/ms-vscode.cpptools-*/debugAdapters/lldb-mi/bin/lldb-mi; do
    if [ -x "${candidate}" ]; then
        LLDB_MI="${candidate}"
    fi
done

if [ -z "${LLDB_MI}" ]; then
    echo "Could not find the C/C++ extension lldb-mi binary." >&2
    exit 1
fi

export SUDO_ASKPASS="${ASKPASS}"
exec /usr/bin/sudo -A -k "${LLDB_MI}" "$@"
