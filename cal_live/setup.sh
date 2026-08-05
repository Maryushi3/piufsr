#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
python3 -m venv venv
# venv/bin/pip is called directly instead of `source venv/bin/activate`, which
# can trip `set -u` on some shells.
venv/bin/pip install --quiet --upgrade pip
venv/bin/pip install --quiet flask pyserial
echo "---"
echo "Ready. Run:  venv/bin/python app.py <serial_port> [baud]"
echo "Then open http://localhost:8767"
echo "Binds localhost only; --host 0.0.0.0 exposes an unauthenticated endpoint"
echo "that can overwrite this pad's saved calibration."
