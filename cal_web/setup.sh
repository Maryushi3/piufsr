#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
python3 -m venv venv
source venv/bin/activate
pip install flask pyserial
echo "---"
echo "Ready. Run:  python app.py <serial_port> [baud]"
echo "Then open http://localhost:5000"
