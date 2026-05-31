#!/usr/bin/env sh
set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
c_root=$(CDPATH= cd -- "$script_dir/.." && pwd)
repo_root=$(CDPATH= cd -- "$c_root/../.." && pwd)
doxyfile="$c_root/Doxyfile"
docs_html="$c_root/docs/html"
docs_index="$c_root/docs/html/index.html"

if ! command -v doxygen >/dev/null 2>&1; then
    echo "error: doxygen was not found in PATH. Install Doxygen first, then rerun this script." >&2
    exit 127
fi

if ! command -v pwsh >/dev/null 2>&1; then
    echo "error: pwsh was not found in PATH. Install PowerShell 7 or run the checked-in headers with doxygen manually." >&2
    exit 127
fi

pwsh -NoProfile -File "$script_dir/generate_headers.ps1"

rm -rf "$docs_html"

cd "$repo_root"
doxygen "$doxyfile"

printf 'Generated COPP C API docs at %s\n' "$docs_index"
