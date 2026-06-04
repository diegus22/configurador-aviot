# Configurador Aviot — Multi-dispositivo Modbus RTU

Aplicación de escritorio (Windows) para **detectar, comprobar y configurar**
todos los dispositivos Modbus RTU que utiliza Aviot, desde una sola herramienta.

## Dispositivos soportados

| Dispositivo | Baud por defecto | ID por defecto | Lectura | Cambio de ID |
|---|---|---|---|---|
| Sonda Temperatura / Humedad | 4800 8N1 | — | FC03 `0x0000` (HR/10), `0x0001` (Tª/10) | FC06 `0x07D0` |
| Sonda CO2 (SenseCAP S-CO2-03) | 9600 8N1 | 45 | FC03 `0x0000` CO2, `0x0001` Tª/100, `0x0002` HR/100 | FC06 `0x0010` |
| Módulo E/S 8 canales (Waveshare IO 8CH) | 9600 8N1 | 1 | FC01 relés / FC02 entradas / FC05 control | broadcast FC06 `0x4000` |
| Módulo E/S 16 canales (IO 16CH) | 9600 8N1 | 1 | variante 16 canales del IO 8CH | broadcast FC06 `0x4000` |

> **IO 16CH**: implementado como variante de 16 canales del protocolo Waveshare.
> Pendiente de verificar con el hardware real (mapa de registros y baudrate
> pueden diferir).

## Uso

1. Selecciona el **tipo de dispositivo** en el desplegable superior.
   El baudrate y el ID por defecto se ajustan solos.
2. Elige el **puerto** del adaptador USB-RS485 y pulsa **Conectar**.
3. Pulsa **DETECTAR / BUSCAR ID** (detección directa, prueba del ID indicado
   y, si hace falta, escaneo 1-247).
4. Usa **Lectura continua** para ver medidas en vivo; en los módulos de E/S
   puedes accionar los relés y ver el estado de las entradas.
5. **CAMBIAR ID** para reasignar la dirección del equipo.

Todo el tráfico TX/RX queda en el panel de log y en
`C:\Users\<usuario>\Aviot_Configurador_Logs\`.

## Arquitectura

Un núcleo común (puerto serie, CRC16, lectura/escritura Modbus, escaneo, log)
y una **clase de perfil** por dispositivo (`DeviceTH`, `DeviceCO2`, `DeviceIO`,
`DeviceIO16`). Añadir un equipo nuevo = añadir una clase de perfil que defina
su baudrate, registros, panel de UI y método de cambio de ID.

## Compilar

- `1_PROBAR_sin_compilar.bat` — ejecuta la app con Python (rápido).
- `2_COMPILAR_exe.bat` — genera `dist\Configurador_Aviot.exe` con PyInstaller.
- En GitHub Actions, cada push a `main` compila el `.exe` y el instalador
  (Inno Setup + driver CH340).

---
Aviot — Always Safe · Ingeniatic Desarrollo S.L.
