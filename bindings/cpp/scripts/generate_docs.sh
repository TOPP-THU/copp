#!/usr/bin/env sh
set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cpp_root=$(CDPATH= cd -- "$script_dir/.." && pwd)
repo_root=$(CDPATH= cd -- "$cpp_root/../.." && pwd)
doxyfile="$cpp_root/Doxyfile"
docs_html="$cpp_root/docs/html"
docs_index="$cpp_root/docs/html/index.html"

if ! command -v doxygen >/dev/null 2>&1; then
    echo "error: doxygen was not found in PATH. Install Doxygen first, then rerun this script." >&2
    exit 127
fi

rm -rf "$docs_html"

cd "$repo_root"
doxygen "$doxyfile"

printf 'Generated COPP C++ API docs at %s\n' "$docs_index"
