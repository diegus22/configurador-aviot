#!/usr/bin/env python3
"""
Configurador Aviot - Multi-dispositivo Modbus RTU
Ingeniatic Desarrollo S.L.

Una sola aplicacion para detectar, comprobar y configurar todos los
dispositivos Modbus RTU que utiliza Aviot:

  - Sonda de Temperatura / Humedad   (Modbus T/H, 4800 8N1)
  - Sonda de CO2  (SenseCAP S-CO2-03, 9600 8N1, ID 45)
  - Modulo de E/S 8 canales  (Waveshare Modbus RTU IO 8CH, 9600 8N1, ID 1)
  - Modulo de E/S 16 canales (variante 16CH - por verificar con hardware)

Arquitectura: un nucleo comun (puerto serie, CRC, log TX/RX, escaneo de IDs)
y un "perfil" (clase Device) por cada dispositivo, que define su baudrate,
mapa de registros, lecturas, panel de UI y como cambiarle el ID. Anadir un
equipo nuevo = anadir una clase de perfil.
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import struct
import time
import threading
import sys
import os
import platform
import subprocess

VERSION = "v1.0"

# ---------------------------------------------------------------------------
#  Colores corporativos Aviot
# ---------------------------------------------------------------------------
AVIOT_NARANJA = "#f39200"
AVIOT_NARANJA_HOVER = "#e08500"
AVIOT_OSCURO = "#1d1d1b"
AVIOT_BLANCO = "#ffffff"
AVIOT_GRIS = "#e0e0e0"
AVIOT_VERDE = "#4CAF50"
AVIOT_ROJO = "#e53935"
AVIOT_AMBAR = "#f9a825"


# ---------------------------------------------------------------------------
#  Utilidades de rutas / ficheros
# ---------------------------------------------------------------------------
def resource_path(rel):
    if hasattr(sys, '_MEIPASS'):
        return os.path.join(sys._MEIPASS, rel)
    return os.path.join(os.path.abspath("."), rel)


def logs_dir():
    carpeta = os.path.join(os.path.expanduser("~"), "Aviot_Configurador_Logs")
    try:
        os.makedirs(carpeta, exist_ok=True)
    except Exception:
        carpeta = os.path.expanduser("~")
    return carpeta


# ---------------------------------------------------------------------------
#  Nucleo Modbus RTU sobre pyserial
# ---------------------------------------------------------------------------
def crc16_modbus(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_frame(data):
    return data + struct.pack('<H', crc16_modbus(data))


def fmt_hex(b):
    return " ".join(f"{x:02X}" for x in b) if b else "(vacio)"


def transaccion(ser, frame_data, expected_len=None, settle=0.06, hard=0.6,
                log=None, verbose=True):
    """
    Envia una trama Modbus y lee la respuesta. Si se conoce expected_len, lee
    justo esa cantidad (rapido). Si no, lee hasta `settle` s sin nuevos bytes.
    """
    frame = build_frame(frame_data)
    try:
        ser.reset_input_buffer()
        ser.write(frame)
    except Exception as e:
        if log:
            log(f"   ERROR de E/S: {e}", "err")
        return b""

    if log and verbose:
        log(f"   TX -> {fmt_hex(frame)}", "tx")

    deadline = time.monotonic() + hard
    resp = bytearray()
    last_rx = time.monotonic()
    while time.monotonic() < deadline:
        try:
            n = ser.in_waiting
        except Exception:
            break
        if n:
            resp.extend(ser.read(n))
            last_rx = time.monotonic()
            if expected_len and len(resp) >= expected_len:
                break
        else:
            if resp and (time.monotonic() - last_rx) > settle:
                break
            time.sleep(0.008)

    if log and verbose:
        log(f"   RX <- {fmt_hex(bytes(resp))}", "tx")
    return bytes(resp)


def crc_ok(resp):
    if len(resp) < 4:
        return False
    rx = resp[-2] | (resp[-1] << 8)
    return crc16_modbus(resp[:-2]) == rx


def read_holding(ser, slave_id, start, count, fc=0x03, log=None, verbose=False):
    """FC03/FC04: lee `count` registros uint16. Devuelve lista o None."""
    frame = bytes([slave_id, fc, (start >> 8) & 0xFF, start & 0xFF,
                   (count >> 8) & 0xFF, count & 0xFF])
    expected = 3 + 2 * count + 2
    resp = transaccion(ser, frame, expected_len=expected, log=log, verbose=verbose)
    if len(resp) < expected or not crc_ok(resp):
        return None
    if resp[0] != slave_id or resp[1] != fc or resp[2] != 2 * count:
        return None
    return [(resp[3 + 2 * i] << 8) | resp[4 + 2 * i] for i in range(count)]


def write_single(ser, slave_id, addr, value, log=None, verbose=True):
    """FC06: escribe un registro. Devuelve True si el eco coincide y CRC OK."""
    frame = bytes([slave_id, 0x06, (addr >> 8) & 0xFF, addr & 0xFF,
                   (value >> 8) & 0xFF, value & 0xFF])
    resp = transaccion(ser, frame, expected_len=8, log=log, verbose=verbose)
    if len(resp) < 8 or not crc_ok(resp):
        return False
    return resp[:6] == frame[:6]


def to_signed16(v):
    return v - 65536 if v > 32767 else v


# ===========================================================================
#  PERFILES DE DISPOSITIVO
#  Cada clase define: etiqueta, baudrate por defecto, lista de baudrates,
#  ID por defecto, como leer una medida/estado, construir su panel, refrescar
#  y cambiar el ID.
# ===========================================================================
class Device:
    KEY = "base"
    LABEL = "Dispositivo"
    DEFAULT_BAUD = 9600
    BAUDS = [4800, 9600, 19200, 38400, 57600, 115200]
    DEFAULT_ID = 1
    ID_MIN, ID_MAX = 1, 247
    # Texto de ayuda mostrado al seleccionar el dispositivo
    HELP = ""

    def build_panel(self, parent, app):
        """Crea los widgets especificos en `parent`. Guarda refs en self."""
        raise NotImplementedError

    def poll(self, ser, slave_id, log=None, verbose=False):
        """Lee el estado/medida del equipo. Devuelve dict (ok=True/False)."""
        raise NotImplementedError

    def update_panel(self, data):
        """Refleja en la UI el dict devuelto por poll()."""
        pass

    def detect(self, ser, log=None):
        """Devuelve el ID detectado sin escanear, o None si no aplica."""
        return None

    def change_id(self, ser, current_id, new_id, log=None):
        """Cambia el ID. Devuelve (ok: bool, id_real: int|None)."""
        raise NotImplementedError

    def change_baud(self, ser, slave_id, new_baud, log=None):
        """
        Cambia el baudrate. Devuelve (status, mode):
          status: True=confirmado, False=fallo de comunicacion, None=no soportado.
          mode:   'reconnect' = reconectar al nuevo baud,
                  'reboot'    = apagar/encender el equipo y reconectar,
                  'unsupported' = este equipo no permite cambiar el baud por ahora.
        """
        return None, "unsupported"


# --------------------------------------------------------------------------
class DeviceTH(Device):
    KEY = "th"
    LABEL = "Sonda Temperatura / Humedad"
    DEFAULT_BAUD = 4800
    BAUDS = [4800, 9600, 19200]
    DEFAULT_ID = 1
    HELP = ("Sonda Modbus de temperatura y humedad. FC03 registros 0x0000 (HR/10) "
            "y 0x0001 (Tª/10, con signo). Cambio de ID por FC06 en 0x07D0. "
            "Por defecto 4800 8N1.")

    def build_panel(self, parent, app):
        self.app = app
        f = ttk.Frame(parent)
        f.pack(fill="both", expand=True)
        self.lbl_temp = ttk.Label(f, text="Temperatura:  ---", style="Big.TLabel")
        self.lbl_temp.pack(anchor="w", pady=4)
        self.lbl_hum = ttk.Label(f, text="Humedad:       ---", style="Big.TLabel")
        self.lbl_hum.pack(anchor="w", pady=4)
        self.lbl_veredicto = ttk.Label(
            f, text="Detecta la sonda y pulsa 'Lectura continua' para comprobar.",
            style="Info.TLabel", wraplength=620, justify="left")
        self.lbl_veredicto.pack(anchor="w", pady=(10, 0))

    def poll(self, ser, slave_id, log=None, verbose=False):
        regs = read_holding(ser, slave_id, 0x0000, 2, log=log, verbose=verbose)
        if not regs:
            return {"ok": False}
        hum = regs[0] / 10.0
        temp = to_signed16(regs[1]) / 10.0
        return {"ok": True, "temp": temp, "hum": hum,
                "raw_temp": regs[1], "raw_hum": regs[0],
                "summary": f"T={temp}C H={hum}%"}

    def update_panel(self, data):
        if not data.get("ok"):
            return
        self.lbl_temp.config(text=f"Temperatura:  {data['temp']} °C")
        self.lbl_hum.config(text=f"Humedad:       {data['hum']} %")
        if data["raw_temp"] == 0 and data["raw_hum"] == 0:
            self.lbl_veredicto.config(
                text="⚠ Lectura 0.0/0.0: el sensor podria estar averiado.",
                style="Warn.TLabel")
        else:
            self.lbl_veredicto.config(text="✅ Mide y comunica correctamente.",
                                      style="OK.TLabel")

    def change_id(self, ser, current_id, new_id, log=None):
        ok = write_single(ser, current_id, 0x07D0, new_id, log=log)
        time.sleep(0.3)
        data = self.poll(ser, new_id, log=log, verbose=True)
        return ok, (new_id if data.get("ok") else None)

    def change_baud(self, ser, slave_id, new_baud, log=None):
        # Esta sonda cambia el ID en 0x07D0 (no usa el mapa estandar XY-MD02 de
        # catalogo, que seria 0x0102). El registro de baudrate de ESTA sonda no
        # esta confirmado, asi que NO se escribe nada para no dejarla inaccesible.
        if log:
            log("Cambio de baudrate NO soportado para esta sonda: registro sin "
                "confirmar. Aporta el datasheet/modelo exacto para habilitarlo.", "warn")
        return None, "unsupported"


# --------------------------------------------------------------------------
class DeviceCO2(Device):
    KEY = "co2"
    LABEL = "Sonda CO2 (SenseCAP S-CO2-03)"
    DEFAULT_BAUD = 9600
    BAUDS = [1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200]
    BAUD_IDX = {1200: 0, 2400: 1, 4800: 2, 9600: 3, 19200: 4,
                38400: 5, 57600: 6, 115200: 7}
    DEFAULT_ID = 45
    HELP = ("Sensor CO2 + Tª + HR. FC03 0x0000 CO2(ppm), 0x0001 Tª(/100, con signo), "
            "0x0002 HR(/100). Config en 0x0010-0x0021. Cambio de ID por FC06 en "
            "0x0010. Por defecto 9600 8N1, ID 45.")

    def build_panel(self, parent, app):
        self.app = app
        f = ttk.Frame(parent)
        f.pack(fill="both", expand=True)
        g = ttk.Frame(f)
        g.pack(fill="x")
        for i, (titulo, attr, unidad) in enumerate(
                [("CO2", "lbl_co2", "ppm"), ("Temperatura", "lbl_temp", "°C"),
                 ("Humedad", "lbl_hum", "%RH")]):
            cell = ttk.LabelFrame(g, text=f" {titulo} ", padding=12)
            cell.grid(row=0, column=i, padx=8, pady=4, sticky="nsew")
            g.columnconfigure(i, weight=1)
            lbl = ttk.Label(cell, text="---", style="Big.TLabel")
            lbl.pack()
            ttk.Label(cell, text=unidad, style="Info.TLabel").pack()
            setattr(self, attr, lbl)
        self.lbl_cfg = ttk.Label(f, text="", style="Info.TLabel",
                                 wraplength=620, justify="left")
        self.lbl_cfg.pack(anchor="w", pady=(10, 0))

    def poll(self, ser, slave_id, log=None, verbose=False):
        regs = read_holding(ser, slave_id, 0x0000, 3, log=log, verbose=verbose)
        if not regs:
            return {"ok": False}
        co2 = regs[0]
        temp = to_signed16(regs[1]) / 100.0
        hum = regs[2] / 100.0
        return {"ok": True, "co2": co2, "temp": temp, "hum": hum,
                "summary": f"CO2={co2}ppm T={temp:.1f}C H={hum:.1f}%"}

    def update_panel(self, data):
        if not data.get("ok"):
            return
        self.lbl_co2.config(text=str(data["co2"]))
        self.lbl_temp.config(text=f"{data['temp']:.2f}")
        self.lbl_hum.config(text=f"{data['hum']:.2f}")
        if data["co2"] == 0:
            self.lbl_cfg.config(
                text="⚠ CO2 = 0: el sensor puede estar calentando (warm-up). Espera.",
                style="Warn.TLabel")
        else:
            self.lbl_cfg.config(text="✅ Midiendo correctamente.", style="OK.TLabel")

    def change_id(self, ser, current_id, new_id, log=None):
        ok = write_single(ser, current_id, 0x0010, new_id, log=log)
        time.sleep(0.3)
        data = self.poll(ser, new_id, log=log, verbose=True)
        return ok, (new_id if data.get("ok") else None)

    def change_baud(self, ser, slave_id, new_baud, log=None):
        code = self.BAUD_IDX.get(new_baud)
        if code is None:
            return None, "unsupported"
        # FC06 al registro 0x0011 con el indice de baudrate. El sensor responde al
        # baud actual; el nuevo baud se aplica tras reiniciar (apagar/encender).
        ok = write_single(ser, slave_id, 0x0011, code, log=log)
        return (True if ok else False), "reboot"


# --------------------------------------------------------------------------
class DeviceIO(Device):
    """Modulo Waveshare Modbus RTU IO con N canales (8 o 16)."""
    KEY = "io8"
    LABEL = "Modulo E/S 8 canales (Waveshare IO 8CH)"
    DEFAULT_BAUD = 9600
    BAUDS = [4800, 9600, 19200, 38400, 57600, 115200, 128000, 256000]
    DEFAULT_ID = 1
    N = 8
    HELP = ("Modulo Waveshare con entradas digitales y reles. FC01 lee reles, "
            "FC02 lee entradas, FC05 controla reles (0xFF00 ON / 0x0000 OFF / "
            "0x5500 toggle). Detecta/cambia ID por broadcast FC06 en 0x4000. "
            "Por defecto 9600 8N1, ID 1.")

    def build_panel(self, parent, app):
        self.app = app
        f = ttk.Frame(parent)
        f.pack(fill="both", expand=True)

        # Salidas
        ttk.Label(f, text="Salidas (reles):", style="Info.TLabel").pack(anchor="w")
        fout = ttk.Frame(f)
        fout.pack(fill="x", pady=(2, 8))
        self.led_out, self.btn_out = [], []
        for i in range(self.N):
            col = ttk.Frame(fout)
            col.grid(row=0, column=i, padx=2)
            fout.columnconfigure(i, weight=1)
            ttk.Label(col, text=f"DO{i+1}", font=("Arial", 8, "bold"),
                      background=AVIOT_BLANCO).pack()
            led = tk.Canvas(col, width=22, height=22, bg=AVIOT_BLANCO,
                            highlightthickness=0)
            led.pack()
            self._led(led, False)
            self.led_out.append(led)
            b = ttk.Button(col, text="⏻", width=3,
                           command=lambda ch=i: self.app.run_async(
                               lambda: self._toggle(ch)))
            b.pack(pady=(2, 0))
            self.btn_out.append(b)

        bar = ttk.Frame(f)
        bar.pack(fill="x", pady=(0, 8))
        ttk.Button(bar, text="ALL ON",
                   command=lambda: self.app.run_async(lambda: self._all(0xFF00))
                   ).pack(side="left", padx=3)
        ttk.Button(bar, text="ALL OFF",
                   command=lambda: self.app.run_async(lambda: self._all(0x0000))
                   ).pack(side="left", padx=3)
        ttk.Button(bar, text="Forzar modo NORMAL",
                   command=lambda: self.app.run_async(self._force_normal)
                   ).pack(side="right", padx=3)

        # Entradas
        ttk.Label(f, text="Entradas (DI):", style="Info.TLabel").pack(anchor="w")
        fin = ttk.Frame(f)
        fin.pack(fill="x", pady=(2, 0))
        self.led_in = []
        for i in range(self.N):
            col = ttk.Frame(fin)
            col.grid(row=0, column=i, padx=2)
            fin.columnconfigure(i, weight=1)
            ttk.Label(col, text=f"DI{i+1}", font=("Arial", 8, "bold"),
                      background=AVIOT_BLANCO).pack()
            led = tk.Canvas(col, width=22, height=22, bg=AVIOT_BLANCO,
                            highlightthickness=0)
            led.pack()
            self._led(led, False)
            self.led_in.append(led)

    def _led(self, canvas, on):
        canvas.delete("all")
        canvas.create_oval(2, 2, 20, 20, fill=AVIOT_VERDE if on else "#444",
                           outline="#222")

    # --- acciones (corren en el hilo de trabajo via app.run_async) ---
    def _toggle(self, ch):
        ser, sid = self.app.ser, self.app.slave_id
        self.app.log(f"Toggle rele DO{ch+1}")
        write_single_fc05(ser, sid, ch, 0x5500, log=self.app.log)
        time.sleep(0.05)
        data = self.poll(ser, sid, log=self.app.log)
        self.app.root.after(0, lambda: self.update_panel(data))

    def _all(self, value):
        ser, sid = self.app.ser, self.app.slave_id
        self.app.log(f"Comando ALL {'ON' if value else 'OFF'}")
        write_single_fc05(ser, sid, 0xFF, value, log=self.app.log)
        time.sleep(0.05)
        data = self.poll(ser, sid, log=self.app.log)
        self.app.root.after(0, lambda: self.update_panel(data))

    def _force_normal(self):
        ser, sid = self.app.ser, self.app.slave_id
        self.app.log("Forzando modo NORMAL en los canales (0x1000+ = 0)")
        data = bytes([sid, 0x10, 0x10, 0x00, (self.N >> 8) & 0xFF, self.N & 0xFF,
                      2 * self.N]) + bytes(2 * self.N)
        resp = transaccion(ser, data, expected_len=8, log=self.app.log)
        ok = crc_ok(resp) and len(resp) >= 6 and resp[1] == 0x10
        self.app.log("Modo NORMAL aplicado." if ok else "No se pudo aplicar.",
                     "ok" if ok else "err")

    def poll(self, ser, slave_id, log=None, verbose=False):
        nbytes = (self.N + 7) // 8
        outs = self._read_bits(ser, slave_id, 0x01, log, verbose)
        ins = self._read_bits(ser, slave_id, 0x02, log, verbose)
        if outs is None and ins is None:
            return {"ok": False}
        return {"ok": True, "outs": outs or [False] * self.N,
                "ins": ins or [False] * self.N,
                "summary": f"DO={self._bits_str(outs)} DI={self._bits_str(ins)}"}

    def _read_bits(self, ser, slave_id, fc, log=None, verbose=False):
        nbytes = (self.N + 7) // 8
        frame = bytes([slave_id, fc, 0x00, 0x00, (self.N >> 8) & 0xFF, self.N & 0xFF])
        resp = transaccion(ser, frame, expected_len=3 + nbytes + 2,
                           log=log, verbose=verbose)
        if len(resp) < 3 + nbytes + 2 or not crc_ok(resp):
            return None
        if resp[0] != slave_id or resp[1] != fc:
            return None
        nb = resp[2]
        val = 0
        for i in range(min(nb, nbytes)):
            val |= resp[3 + i] << (8 * i)
        return [bool(val & (1 << i)) for i in range(self.N)]

    def _bits_str(self, bits):
        if not bits:
            return "?"
        return "".join("1" if b else "0" for b in bits)

    def update_panel(self, data):
        if not data.get("ok"):
            return
        for i, on in enumerate(data["outs"]):
            self._led(self.led_out[i], on)
        for i, on in enumerate(data["ins"]):
            self._led(self.led_in[i], on)

    def detect(self, ser, log=None):
        """Broadcast lectura registro 0x4000 -> ID actual."""
        resp = transaccion(ser, bytes([0x00, 0x03, 0x40, 0x00, 0x00, 0x01]),
                           expected_len=7, log=log)
        if len(resp) >= 7 and crc_ok(resp) and resp[1] == 0x03 and resp[2] == 0x02:
            return (resp[3] << 8) | resp[4]
        return None

    # Codigos de baudrate del modulo Waveshare IO (registro 0x2000)
    IO_BAUD_CODE = {4800: 0, 9600: 1, 19200: 2, 38400: 3,
                    57600: 4, 115200: 5, 128000: 6, 256000: 7}

    def change_id(self, ser, current_id, new_id, log=None):
        # Broadcast a 0x4000
        ok = write_single(ser, 0x00, 0x4000, new_id, log=log)
        time.sleep(0.2)
        real = self.detect(ser, log=log)
        return ok, real

    def change_baud(self, ser, slave_id, new_baud, log=None):
        code = self.IO_BAUD_CODE.get(new_baud)
        if code is None:
            return None, "unsupported"
        # Broadcast FC06 al registro 0x2000: byte alto = paridad (0=ninguna),
        # byte bajo = codigo de baudrate. Tras el eco hay que reconectar al nuevo baud.
        value = (0x00 << 8) | code
        ok = write_single(ser, 0x00, 0x2000, value, log=log)
        return (True if ok else False), "reconnect"


class DeviceIO16(DeviceIO):
    KEY = "io16"
    LABEL = "Modulo E/S 16 canales (IO 16CH) [por verificar]"
    N = 16
    HELP = ("Variante de 16 canales del modulo Waveshare. Mismo protocolo que el "
            "8CH pero con 16 entradas/reles. PENDIENTE de verificar con el hardware "
            "real (mapa de registros y baudrate pueden variar).")


def write_single_fc05(ser, slave_id, channel, value, log=None):
    """FC05 para reles del modulo IO. channel 0-15 o 0xFF para todos."""
    frame = bytes([slave_id, 0x05, 0x00, channel & 0xFF,
                   (value >> 8) & 0xFF, value & 0xFF])
    resp = transaccion(ser, frame, expected_len=8, log=log)
    return crc_ok(resp) and len(resp) >= 6 and resp[1] == 0x05


# Orden de los dispositivos en el selector
DEVICE_CLASSES = [DeviceTH, DeviceCO2, DeviceIO, DeviceIO16]


# ===========================================================================
#  Aplicacion
# ===========================================================================
class App:
    def __init__(self, root):
        self.root = root
        self.root.title(f"Configurador Aviot - Multi-dispositivo  ({VERSION})")
        self.root.configure(bg=AVIOT_BLANCO)
        self.root.minsize(720, 560)
        self.ser = None
        self.slave_id = 1
        self.ocupado = False
        self.device = None
        self._poll_on = False

        # Log
        self._log_lock = threading.Lock()
        self.log_path = os.path.join(
            logs_dir(), f"configurador_{time.strftime('%Y%m%d_%H%M%S')}.log")
        try:
            self._log_file = open(self.log_path, "a", encoding="utf-8")
        except Exception:
            self._log_file = None

        try:
            self.logo_img = tk.PhotoImage(file=resource_path("logo_aviot.png"))
            self.root.iconphoto(True, self.logo_img)
        except Exception:
            self.logo_img = None

        self._build_styles()
        self._build_ui()

        self.log("=" * 60)
        self.log(f"Configurador Aviot {VERSION}")
        self.log(f"Sistema: {platform.platform()}")
        self.log(f"Fecha:   {time.strftime('%Y-%m-%d %H:%M:%S')}")
        self.log("=" * 60)

        self._select_device(0)
        self.actualizar_puertos()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ----------------------------- estilo -----------------------------
    def _build_styles(self):
        s = ttk.Style()
        s.theme_use('clam')
        s.configure(".", background=AVIOT_BLANCO, foreground=AVIOT_OSCURO)
        s.configure("TFrame", background=AVIOT_BLANCO)
        s.configure("TLabelframe", background=AVIOT_BLANCO, font=("Arial", 9, "bold"))
        s.configure("TLabelframe.Label", background=AVIOT_BLANCO,
                    foreground=AVIOT_NARANJA, font=("Arial", 9, "bold"))
        s.configure("Aviot.TButton", font=("Arial", 9, "bold"), padding=4,
                    background=AVIOT_NARANJA, foreground=AVIOT_BLANCO)
        s.map("Aviot.TButton",
              background=[('active', AVIOT_NARANJA_HOVER), ('pressed', AVIOT_OSCURO)])
        s.configure("Dark.TButton", font=("Arial", 9, "bold"), padding=4,
                    background=AVIOT_OSCURO, foreground=AVIOT_BLANCO)
        s.map("Dark.TButton", background=[('active', "#333"), ('pressed', AVIOT_NARANJA)])
        s.configure("Header.TLabel", font=("Arial", 14, "bold"),
                    foreground=AVIOT_NARANJA, background=AVIOT_BLANCO)
        s.configure("Sub.TLabel", font=("Arial", 8), foreground="#888",
                    background=AVIOT_BLANCO)
        s.configure("Info.TLabel", font=("Arial", 10), background=AVIOT_BLANCO)
        s.configure("Big.TLabel", font=("Arial", 15, "bold"), background=AVIOT_BLANCO)
        s.configure("OK.TLabel", font=("Arial", 10, "bold"),
                    foreground=AVIOT_VERDE, background=AVIOT_BLANCO)
        s.configure("Warn.TLabel", font=("Arial", 10, "bold"),
                    foreground=AVIOT_AMBAR, background=AVIOT_BLANCO)
        s.configure("Err.TLabel", font=("Arial", 10, "bold"),
                    foreground=AVIOT_ROJO, background=AVIOT_BLANCO)

    # ----------------------------- UI -----------------------------
    def _build_ui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        top = ttk.Frame(self.root, padding=(14, 6, 14, 2))
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(0, weight=1)

        header = ttk.Frame(top)
        header.grid(row=0, column=0, pady=(0, 6))
        if self.logo_img:
            try:
                self.small_logo = self.logo_img.subsample(
                    max(1, self.logo_img.width() // 170),
                    max(1, self.logo_img.height() // 46))
                ttk.Label(header, image=self.small_logo, background=AVIOT_BLANCO).pack()
            except Exception:
                pass
        ttk.Label(header, text="CONFIGURADOR AVIOT", style="Header.TLabel").pack()
        ttk.Label(header, text=f"Multi-dispositivo Modbus RTU  |  {VERSION}",
                  style="Sub.TLabel").pack()
        tk.Frame(top, height=3, bg=AVIOT_NARANJA).grid(row=1, column=0, sticky="ew",
                                                      pady=(0, 8))

        # --- Selector de dispositivo ---
        fdev = ttk.LabelFrame(top, text=" Dispositivo ", padding=10)
        fdev.grid(row=2, column=0, sticky="ew", pady=(0, 6))
        fdev.columnconfigure(0, weight=1)
        self.combo_dev = ttk.Combobox(fdev, state="readonly", font=("Arial", 9),
                                      values=[c.LABEL for c in DEVICE_CLASSES])
        self.combo_dev.current(0)
        self.combo_dev.grid(row=0, column=0, sticky="ew")
        self.combo_dev.bind("<<ComboboxSelected>>",
                            lambda e: self._select_device(self.combo_dev.current()))
        self.lbl_help = ttk.Label(fdev, text="", style="Sub.TLabel",
                                  wraplength=720, justify="left")
        self.lbl_help.grid(row=1, column=0, sticky="w", pady=(6, 0))

        # --- Conexion ---
        fcon = ttk.LabelFrame(top, text=" Conexion ", padding=10)
        fcon.grid(row=3, column=0, sticky="ew", pady=(0, 6))
        fcon.columnconfigure(1, weight=1)
        ttk.Label(fcon, text="Puerto:", font=("Arial", 9)).grid(row=0, column=0, sticky="w")
        self.combo_puerto = ttk.Combobox(fcon, width=26, font=("Arial", 9), state="readonly")
        self.combo_puerto.grid(row=0, column=1, padx=5, sticky="ew")
        ttk.Button(fcon, text="Actualizar", command=self.actualizar_puertos).grid(row=0, column=2, padx=4)

        ttk.Label(fcon, text="Baudrate:", font=("Arial", 9)).grid(row=1, column=0, sticky="w", pady=(6, 0))
        self.combo_baud = ttk.Combobox(fcon, width=10, font=("Arial", 9), state="readonly")
        self.combo_baud.grid(row=1, column=1, padx=5, sticky="w", pady=(6, 0))
        ttk.Label(fcon, text="ID:", font=("Arial", 9)).grid(row=1, column=2, sticky="e", pady=(6, 0))
        self.spin_id = ttk.Spinbox(fcon, from_=1, to=247, width=5, font=("Arial", 9))
        self.spin_id.grid(row=1, column=3, padx=5, pady=(6, 0))

        self.btn_conectar = ttk.Button(fcon, text="Conectar", style="Aviot.TButton",
                                       command=self.toggle_conexion)
        self.btn_conectar.grid(row=2, column=0, columnspan=4, pady=(10, 0), sticky="ew")
        self.lbl_estado = ttk.Label(fcon, text="Desconectado", style="Err.TLabel")
        self.lbl_estado.grid(row=3, column=0, columnspan=4, pady=(4, 0))

        # --- Acciones comunes ---
        fact = ttk.LabelFrame(top, text=" Detectar / Configurar ", padding=10)
        fact.grid(row=4, column=0, sticky="ew", pady=(0, 6))
        fact.columnconfigure(3, weight=1)
        ttk.Button(fact, text="DETECTAR / BUSCAR ID", style="Aviot.TButton",
                   command=self.detectar).grid(row=0, column=0, columnspan=2, sticky="ew")
        self.btn_poll = ttk.Button(fact, text="Lectura continua", style="Dark.TButton",
                                   command=self.toggle_poll)
        self.btn_poll.grid(row=0, column=2, padx=6)
        self.lbl_detect = ttk.Label(fact, text="", style="Info.TLabel")
        self.lbl_detect.grid(row=1, column=0, columnspan=4, sticky="w", pady=(6, 0))

        ttk.Label(fact, text="Nuevo ID:", font=("Arial", 9)).grid(row=2, column=0, sticky="e", pady=(8, 0))
        self.spin_newid = ttk.Spinbox(fact, from_=1, to=247, width=5, font=("Arial", 9))
        self.spin_newid.set(2)
        self.spin_newid.grid(row=2, column=1, sticky="w", pady=(8, 0))
        ttk.Button(fact, text="CAMBIAR ID", style="Aviot.TButton",
                   command=self.cambiar_id).grid(row=2, column=2, pady=(8, 0))
        self.lbl_id = ttk.Label(fact, text="", style="Info.TLabel")
        self.lbl_id.grid(row=3, column=0, columnspan=4, sticky="w", pady=(4, 0))

        ttk.Label(fact, text="Nuevo baud:", font=("Arial", 9)).grid(row=4, column=0, sticky="e", pady=(8, 0))
        self.combo_newbaud = ttk.Combobox(fact, width=8, font=("Arial", 9), state="readonly")
        self.combo_newbaud.grid(row=4, column=1, sticky="w", pady=(8, 0))
        ttk.Button(fact, text="CAMBIAR BAUDRATE", style="Aviot.TButton",
                   command=self.cambiar_baud).grid(row=4, column=2, pady=(8, 0))
        self.lbl_baud = ttk.Label(fact, text="", style="Info.TLabel")
        self.lbl_baud.grid(row=5, column=0, columnspan=4, sticky="w", pady=(4, 0))

        # --- Panel del dispositivo (dinamico) ---
        self.fpanel = ttk.LabelFrame(top, text=" Estado / Medidas ", padding=10)
        self.fpanel.grid(row=5, column=0, sticky="ew", pady=(0, 6))
        self.fpanel.columnconfigure(0, weight=1)
        self.panel_inner = ttk.Frame(self.fpanel)
        self.panel_inner.pack(fill="both", expand=True)

        # --- Log ---
        flog = ttk.LabelFrame(self.root, text=" Registro tecnico ", padding=10)
        flog.grid(row=1, column=0, sticky="nsew", padx=20, pady=(0, 10))
        flog.columnconfigure(0, weight=1)
        flog.rowconfigure(0, weight=1)
        self.txt_log = scrolledtext.ScrolledText(flog, height=6, font=("Consolas", 8),
                                                 bg="#111", fg="#d6d6d6",
                                                 insertbackground="#fff", wrap="none")
        self.txt_log.grid(row=0, column=0, sticky="nsew")
        self.txt_log.tag_config("ok", foreground="#6fdc6f")
        self.txt_log.tag_config("warn", foreground="#ffc451")
        self.txt_log.tag_config("err", foreground="#ff6b6b")
        self.txt_log.tag_config("tx", foreground="#7fb3ff")
        bar = ttk.Frame(flog)
        bar.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        ttk.Button(bar, text="Abrir carpeta de logs", command=self.abrir_logs).pack(side="left")
        ttk.Button(bar, text="Copiar log", command=self.copiar_log).pack(side="left", padx=6)
        ttk.Button(bar, text="Limpiar", command=lambda: self.txt_log.delete("1.0", "end")).pack(side="left")
        ttk.Label(bar, text=f"Aviot - Always Safe  |  {VERSION}", font=("Arial", 8),
                  foreground="#aaa", background=AVIOT_BLANCO).pack(side="right")

    # ----------------------------- seleccion de dispositivo -----------------------------
    def _select_device(self, idx):
        if self.ser and self.ser.is_open:
            messagebox.showinfo("Aviso", "Desconecta antes de cambiar de dispositivo.")
            # revertir el combo a la seleccion actual
            self.combo_dev.current(DEVICE_CLASSES.index(type(self.device)))
            return
        self._poll_on = False
        self.device = DEVICE_CLASSES[idx]()
        self.lbl_help.config(text=self.device.HELP)
        # baudrates
        self.combo_baud['values'] = [str(b) for b in self.device.BAUDS]
        self.combo_baud.set(str(self.device.DEFAULT_BAUD))
        self.combo_newbaud['values'] = [str(b) for b in self.device.BAUDS]
        self.combo_newbaud.set(str(self.device.DEFAULT_BAUD))
        self.spin_id.delete(0, "end")
        self.spin_id.insert(0, str(self.device.DEFAULT_ID))
        self.slave_id = self.device.DEFAULT_ID
        # reconstruir panel
        for w in self.panel_inner.winfo_children():
            w.destroy()
        self.device.build_panel(self.panel_inner, self)
        self.lbl_detect.config(text="")
        self.lbl_id.config(text="")
        self.lbl_baud.config(text="")
        self.log(f"Dispositivo seleccionado: {self.device.LABEL}")

    # ----------------------------- logging -----------------------------
    def log(self, msg, level="info"):
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {msg}"
        try:
            with self._log_lock:
                if self._log_file:
                    self._log_file.write(line + "\n")
                    self._log_file.flush()
        except Exception:
            pass
        self.root.after(0, lambda: self._append_log(line, level))

    def _append_log(self, line, level):
        tag = level if level in ("ok", "warn", "err", "tx") else "info"
        self.txt_log.insert("end", line + "\n", tag)
        self.txt_log.see("end")

    def copiar_log(self):
        self.root.clipboard_clear()
        self.root.clipboard_append(self.txt_log.get("1.0", "end"))
        messagebox.showinfo("Log copiado", "El registro se ha copiado al portapapeles.")

    def abrir_logs(self):
        carpeta = os.path.dirname(self.log_path)
        try:
            if platform.system() == "Windows":
                os.startfile(carpeta)  # noqa
            elif platform.system() == "Darwin":
                subprocess.run(["open", carpeta])
            else:
                subprocess.run(["xdg-open", carpeta])
        except Exception as e:
            messagebox.showinfo("Carpeta de logs", f"{carpeta}\n\n({e})")

    # ----------------------------- puertos / conexion -----------------------------
    def actualizar_puertos(self):
        puertos = serial.tools.list_ports.comports()
        encontrados = []
        self.log("Buscando puertos serie...")
        for p in puertos:
            texto = f"{p.device} {p.description} {p.hwid}".lower()
            self.log(f"   Puerto: {p.device}  ({p.description})")
            if any(k in texto for k in ('usb', 'serial', 'ch340', 'cp210', 'ftdi')):
                encontrados.append(f"{p.device} - {p.description}")
        self.combo_puerto['values'] = encontrados if encontrados else ["No se detectan puertos"]
        if encontrados:
            self.combo_puerto.current(0)
            self.log(f"{len(encontrados)} adaptador(es) compatible(s).", "ok")
        else:
            self.log("No se detecta ningun adaptador USB-Serial.", "warn")

    def toggle_conexion(self):
        if self.ser and self.ser.is_open:
            self._poll_on = False
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.btn_conectar.config(text="Conectar")
            self.lbl_estado.config(text="Desconectado", style="Err.TLabel")
            self.log("Puerto cerrado.", "warn")
            return
        puerto = self.combo_puerto.get().split(" - ")[0].strip()
        if not puerto or "No se detectan" in puerto:
            messagebox.showerror("Error", "Conecta el adaptador USB-RS485 y pulsa Actualizar.")
            return
        try:
            baud = int(self.combo_baud.get())
        except ValueError:
            baud = self.device.DEFAULT_BAUD
        try:
            self.ser = serial.Serial(port=puerto, baudrate=baud, parity='N',
                                     stopbits=1, bytesize=8, timeout=0.4)
            self.btn_conectar.config(text="Desconectar")
            self.lbl_estado.config(text=f"Conectado a {puerto} @ {baud} 8N1", style="OK.TLabel")
            self.log(f"Puerto abierto: {puerto} @ {baud} 8N1.", "ok")
        except serial.SerialException as e:
            messagebox.showerror("Error", f"No se pudo abrir el puerto:\n{e}")
            self.log(f"ERROR abriendo puerto: {e}", "err")

    def check_conexion(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Aviso", "Primero conecta el adaptador USB-RS485.")
            return False
        return True

    def run_async(self, fn):
        """Ejecuta fn() en un hilo si hay conexion y no hay otra operacion."""
        if not self.check_conexion() or self.ocupado:
            return
        self.ocupado = True

        def worker():
            try:
                fn()
            except Exception as e:
                self.log(f"ERROR: {e}", "err")
            finally:
                self.ocupado = False

        threading.Thread(target=worker, daemon=True).start()

    # ----------------------------- detectar -----------------------------
    def detectar(self):
        if not self.check_conexion():
            return
        self._sync_slave_id()

        def work():
            self.log(f"--- DETECTAR ({self.device.LABEL}) ---")
            # 1) deteccion directa por broadcast si el equipo la soporta
            det = self.device.detect(self.ser, log=self.log)
            if det is not None:
                self.slave_id = det
                self.root.after(0, lambda: self.spin_id.set(str(det)))
                data = self.device.poll(self.ser, det, log=self.log, verbose=True)
                self.root.after(0, lambda: self._detect_ok(det, data))
                return
            # 2) probar el ID indicado
            sid = self.slave_id
            data = self.device.poll(self.ser, sid, log=self.log, verbose=True)
            if data.get("ok"):
                self.root.after(0, lambda: self._detect_ok(sid, data))
                return
            # 3) escaneo 1..247
            self.log("Escaneando IDs 1-247...")
            for s in range(1, 248):
                self.root.after(0, lambda s=s: self.lbl_detect.config(
                    text=f"Escaneando ID {s}/247...", style="Info.TLabel"))
                d = self.device.poll(self.ser, s, verbose=False)
                if d.get("ok"):
                    self.slave_id = s
                    self.root.after(0, lambda s=s: self.spin_id.set(str(s)))
                    self.root.after(0, lambda s=s, d=d: self._detect_ok(s, d))
                    return
            self.root.after(0, self._detect_fail)

        self.run_async(work)

    def _detect_ok(self, sid, data):
        self.slave_id = sid
        self.lbl_detect.config(text=f"Detectado en ID {sid}.  {data.get('summary','')}",
                               style="OK.TLabel")
        self.log(f"Detectado en ID {sid}. {data.get('summary','')}", "ok")
        self.device.update_panel(data)

    def _detect_fail(self):
        self.lbl_detect.config(
            text="No responde. Revisa cableado A+/B-, alimentacion, baudrate e ID.",
            style="Err.TLabel")

    # ----------------------------- lectura continua -----------------------------
    def toggle_poll(self):
        if self._poll_on:
            self._poll_on = False
            self.btn_poll.config(text="Lectura continua")
            return
        if not self.check_conexion():
            return
        self._sync_slave_id()
        self._poll_on = True
        self.btn_poll.config(text="Detener lectura")
        threading.Thread(target=self._poll_loop, daemon=True).start()

    def _poll_loop(self):
        while self._poll_on and self.ser and self.ser.is_open:
            if not self.ocupado:
                data = self.device.poll(self.ser, self.slave_id, verbose=False)
                if data.get("ok"):
                    self.root.after(0, lambda d=data: self.device.update_panel(d))
            time.sleep(0.3)

    # ----------------------------- cambiar ID -----------------------------
    def _sync_slave_id(self):
        try:
            self.slave_id = int(self.spin_id.get())
        except ValueError:
            pass

    def cambiar_id(self):
        if not self.check_conexion():
            return
        self._sync_slave_id()
        try:
            nuevo = int(self.spin_newid.get())
        except ValueError:
            return
        if not self.device.ID_MIN <= nuevo <= self.device.ID_MAX:
            messagebox.showerror("Error", f"ID fuera de rango "
                                 f"({self.device.ID_MIN}-{self.device.ID_MAX}).")
            return
        if not messagebox.askyesno("Confirmar",
                                   f"Cambiar el ID a {nuevo} (0x{nuevo:02X})?"):
            return

        def work():
            self.log(f"--- CAMBIAR ID a {nuevo} ({self.device.LABEL}) ---")
            ok, real = self.device.change_id(self.ser, self.slave_id, nuevo, log=self.log)
            self.root.after(0, lambda: self._id_result(ok, real, nuevo))

        self.run_async(work)

    def _id_result(self, ok, real, nuevo):
        if real == nuevo:
            self.slave_id = nuevo
            self.spin_id.set(str(nuevo))
            self.lbl_id.config(text=f"ID cambiado correctamente a {nuevo}.", style="OK.TLabel")
            self.log(f"ID confirmado: {nuevo}.", "ok")
        elif real is not None:
            self.lbl_id.config(text=f"Comando enviado; el equipo responde con ID {real}.",
                               style="Warn.TLabel")
        else:
            self.lbl_id.config(text="Cambio enviado pero no se pudo verificar el nuevo ID.",
                               style="Err.TLabel")

    # ----------------------------- cambiar baudrate -----------------------------
    def cambiar_baud(self):
        if not self.check_conexion():
            return
        self._sync_slave_id()
        try:
            nuevo = int(self.combo_newbaud.get())
        except ValueError:
            return
        actual = int(self.combo_baud.get())
        if nuevo == actual:
            messagebox.showinfo("Aviso", "El nuevo baudrate es igual al actual.")
            return
        if not messagebox.askyesno(
                "Confirmar",
                f"Cambiar el baudrate del equipo a {nuevo} bps?\n\n"
                "Tras el cambio tendras que reconectar al nuevo baudrate "
                "(y en el sensor de CO2, apagar y encender primero)."):
            return

        def work():
            self.log(f"--- CAMBIAR BAUDRATE a {nuevo} ({self.device.LABEL}) ---")
            status, mode = self.device.change_baud(self.ser, self.slave_id, nuevo, log=self.log)
            self.root.after(0, lambda: self._baud_result(status, mode, nuevo))

        self.run_async(work)

    def _baud_result(self, status, mode, nuevo):
        if status is None:
            self.lbl_baud.config(
                text="Cambio de baudrate no disponible para este dispositivo todavia.",
                style="Warn.TLabel")
            messagebox.showinfo(
                "No disponible",
                "El registro de baudrate de este dispositivo no esta confirmado, "
                "asi que no se cambia para no dejarlo inaccesible.\n\n"
                "Si tienes el datasheet/modelo exacto, lo habilitamos.")
            return
        if not status:
            self.lbl_baud.config(text="No se confirmo el cambio (sin eco). "
                                 f"Prueba a reconectar a {nuevo} por si se aplico igualmente.",
                                 style="Err.TLabel")
            return
        # Exito: cerrar conexion y dejar preparado el nuevo baud
        self.combo_baud.set(str(nuevo))
        self._poll_on = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.btn_conectar.config(text="Conectar")
        self.lbl_estado.config(text="Desconectado", style="Err.TLabel")
        if mode == "reboot":
            self.lbl_baud.config(
                text=f"Baudrate cambiado a {nuevo}. APAGA y ENCIENDE el sensor y "
                     f"pulsa Conectar (ya a {nuevo}).", style="OK.TLabel")
        else:
            self.lbl_baud.config(
                text=f"Baudrate cambiado a {nuevo}. Pulsa Conectar (ya a {nuevo}).",
                style="OK.TLabel")
        self.log(f"Baudrate cambiado a {nuevo}. Reconecta al nuevo baud.", "ok")

    # ----------------------------- cierre -----------------------------
    def _on_close(self):
        self._poll_on = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        try:
            if self._log_file:
                self._log_file.close()
        except Exception:
            pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
