#!/bin/zsh
# Lanzador del Configurador Aviot en macOS.
# Requisitos (una vez): brew install python-tk@3.13
#   /opt/homebrew/bin/python3.13 -m venv --system-site-packages ~/.venvs/configurador-aviot
#   ~/.venvs/configurador-aviot/bin/pip install pyserial
cd "$(dirname "$0")"
PY="$HOME/.venvs/configurador-aviot/bin/python"
if [ ! -x "$PY" ]; then
  echo "Falta el entorno. Ejecuta:"
  echo "  brew install python-tk@3.13"
  echo "  /opt/homebrew/bin/python3.13 -m venv --system-site-packages ~/.venvs/configurador-aviot"
  echo "  ~/.venvs/configurador-aviot/bin/pip install pyserial"
  read -k 1
  exit 1
fi
exec "$PY" configurador_aviot.py
