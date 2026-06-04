@echo off
REM ============================================================
REM  Compila el .exe de Windows (onefile) con PyInstaller.
REM  Resultado:  dist\Configurador_Aviot.exe
REM ============================================================
chcp 65001 >nul
echo Instalando dependencias (pyserial + pyinstaller)...
py -m pip install --upgrade pip
py -m pip install pyserial pyinstaller
echo.
echo Compilando... (puede tardar 1-2 minutos)
py -m PyInstaller --clean --noconfirm --onefile --windowed ^
  --name "Configurador_Aviot" --icon=aviot.ico ^
  --add-data "logo_aviot.png;." configurador_aviot.py
echo.
if exist dist\Configurador_Aviot.exe (
    echo ============================================================
    echo  LISTO. Ejecutable creado en:  dist\Configurador_Aviot.exe
    echo ============================================================
) else (
    echo *** ERROR: no se genero el .exe. Revisa los mensajes de arriba.
)
pause
