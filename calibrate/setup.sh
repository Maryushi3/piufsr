#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
python3 -m venv venv
# venv/bin/pip is called directly instead of `source venv/bin/activate`, which
# can trip `set -u` on some shells.
venv/bin/pip install --quiet --upgrade pip
venv/bin/pip install --quiet pyserial
echo "---"
echo "Ready. Run:  venv/bin/python cal.py <serial_port> [baud] [threshold]"
