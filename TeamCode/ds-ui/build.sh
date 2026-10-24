#!/bin/sh
# Rebuild the driver station stylesheet. Requires the standalone Tailwind CLI
# binary in this directory (not committed):
#   curl -sL -o tailwindcss https://github.com/tailwindlabs/tailwindcss/releases/latest/download/tailwindcss-linux-x64
#   chmod +x tailwindcss
set -e
cd "$(dirname "$0")"
./tailwindcss -i input.css -o ../src/main/assets/ds/app.css --minify
