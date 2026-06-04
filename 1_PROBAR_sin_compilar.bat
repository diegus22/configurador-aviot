@echo off
REM ============================================================
REM  Ejecuta el Configurador Aviot (sin compilar) para probarlo.
REM  Necesita Python 3 instalado (lanzador "py").
REM ============================================================
chcp 65001 >nul
echo Instalando dependencia (pyserial)...
py -m pip install --quiet pyserial
echo Arrancando Configurador Aviot...
py configurador_aviot.py
echo.
echo (Si se cerro con error, la linea de arriba lo muestra.)
pause
