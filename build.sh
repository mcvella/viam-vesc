#!/bin/sh
cd `dirname $0`

# Create a virtual environment to run our code
VENV_NAME="venv"
PYTHON="$VENV_NAME/bin/python"

if ! $PYTHON -m pip install pyinstaller -Uqq; then
    exit 1
fi

# python-can loads backends like socketcan dynamically; collect the whole package
# or the frozen module cannot open SocketCAN.
$PYTHON -m PyInstaller --onefile \
  --collect-all viam \
  --collect-all can \
  --hidden-import=can.interfaces.socketcan \
  --hidden-import=googleapiclient \
  src/main.py
chmod +x dist/main 2>/dev/null || true
tar -czvf dist/archive.tar.gz ./dist/main
