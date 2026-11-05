#!/usr/bin/env bash
# Downloads the recorded real-device sessions (release "real-sessions") into testdata/real/.
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
DEST="$ROOT/testdata/real"
mkdir -p "$DEST"
if ! gh release view real-sessions > /dev/null 2>&1; then
  echo "no real-sessions release yet: see docs/real-sessions.md"
  exit 0
fi
gh release download real-sessions --dir "$DEST" --pattern '*.tar' --clobber
for t in "$DEST"/*.tar; do
  tar xf "$t" -C "$DEST"
  rm "$t"
done
ls "$DEST"
