# ============================================================
#  Dashboard — Sistema de Monitoreo
#  Freno Magnético · Motor 57BLDC
#
#  Autores:
#    Juan Andrés Sanchez
#    Sofía Vega
#    Andrés Felipe Trujillo
#
#  Pantallas:
#    0 · Bienvenida
#    1 · Configuración + Calibración
#    2 · Selección de prueba
#    3 · Gráficas en tiempo real (P1 / P2 captura)
#    P2 extra · 4 curvas características vs TL
# ============================================================

import csv
import json
import math
import os
import ssl
import sys
import threading
import time
import bisect
from datetime import datetime

import paho.mqtt.client as mqtt
import serial
from serial.tools import list_ports

from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QPushButton, QFrame, QStackedWidget,
    QScrollArea, QFileDialog, QMessageBox,
    QSizePolicy, QDoubleSpinBox,
)
import pyqtgraph as pg


# ============================================================
#  CREDENCIALES — HiveMQ Cloud
# ============================================================
MQTT_BROKER     = "0e44beba4fc7422cb74bc8bbdcc67b2f.s1.eu.hivemq.cloud"
MQTT_PORT       = 8883
MQTT_USER       = "ESPMICROUNO"
MQTT_PASSWORD   = "espmicrouno"
TOPIC_TELEMETRY = "micro1/motor1/telemetry"
TOPIC_CMD       = "micro1/motor1/cmd"

# ============================================================
#  CONFIGURACION SERIAL
# ============================================================
SERIAL_BAUD = 115200

# ============================================================
#  FUENTE PROGRAMABLE KEYSIGHT  —  PyVISA  (Canal 1)
# ============================================================
VISA_ADDRESS    = "USB0::0x2A8D::0x3302::MY61004643::0::INSTR"
FUENTE_INTERVAL = 0.5   # segundos entre lecturas
FUENTE_I_LIMITE = 1.0   # límite de corriente CH1 [A]

# ============================================================
#  PARAMETROS DE ADQUISICION
# ============================================================
MAX_POINTS           = 600   # puntos en memoria para graficar
RPM_CHANGE_THRESHOLD = 5.0   # RPM mínimo de cambio para actualizar torque

# ============================================================
#  ESTADO COMPARTIDO  (hilos de datos → GUI)
# ============================================================
_lock = threading.Lock()

# Series de tiempo (graficadas)
_t_buf:   list[float] = []
_n_buf:   list[float] = []   # RPM final
_w_buf:   list[float] = []   # omega [rad/s]
_V_buf:   list[float] = []   # voltaje [V]
_I_buf:   list[float] = []   # corriente [A]
_FL_buf:  list[float] = []   # fuerza galga [N]
_TL_buf:  list[float] = []   # torque [N·m]
_Pe_buf:  list[float] = []   # potencia eléctrica [W]
_Pm_buf:  list[float] = []   # potencia mecánica [W]
_eta_buf: list[float] = []   # eficiencia [%]

def _resetear_tiempo():
    """Vacía buffers y pone t=0 en el origen del ensayo."""
    global _inicio_tiempo
    with _lock:
        for buf in (_t_buf, _n_buf, _w_buf, _FL_buf, _TL_buf,
                    _V_buf, _I_buf, _Pe_buf, _Pm_buf, _eta_buf):
            buf.clear()
    _inicio_tiempo = time.time()

# Registro completo (todos los campos, para CSV)
_registros: list[dict] = []

# Puntos de operación estables capturados manualmente
_puntos_op: list[dict] = []

# Variables auxiliares del filtro de torque
_ultima_rpm_ref  = None
_ultimo_torque_v = 0.0

# Tiempos
_inicio_tiempo = time.time()

# Valores actuales de la fuente (actualizados por hilo PyVISA)
_V_fuente    = 0.0
_fuente_inst = None   # handle pyvisa.Resource compartido con controles GUI
# Estado de salida deseado. Permite que la GUI defina ON/OFF incluso
# si la fuente todavía no terminó de conectarse por PyVISA.
_fuente_output_deseada = True

# Prueba de escalones de voltaje
_escalon_stop = threading.Event()   # set() para detener la secuencia

# Captura de puntos Prueba 2
_p2_puntos: list[dict] = []          # cada entrada: {d, TL, n, I, Pm, eta}
_captura_p2_stop = threading.Event() # set() para cancelar captura activa
_p2_distancia_mm: float = 12.0       # distancia imán–eje vigente hasta que el usuario la cambie

# Base FEM del freno (RPM × airgap → TL teórico)
_FEM_CSV = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "DatosTeoricos", "eddy_brake_fem_database.csv",
))
_fem_rpms: list[float] = []
_fem_gaps: list[float] = []
_fem_T: dict[tuple[float, float], float] = {}


def _cargar_fem_csv(ruta: str = _FEM_CSV) -> bool:
    """Carga rpm, air_gap_mm, torque_Nm del CSV FEM."""
    global _fem_rpms, _fem_gaps, _fem_T
    _fem_T = {}
    rpms: set[float] = set()
    gaps: set[float] = set()
    try:
        with open(ruta, newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                try:
                    rpm = round(float(row["rpm"]), 4)
                    gap = round(float(row["air_gap_mm"]), 4)
                    tl  = float(row["torque_Nm"])
                except (KeyError, TypeError, ValueError):
                    continue
                _fem_T[(rpm, gap)] = tl
                rpms.add(rpm)
                gaps.add(gap)
    except OSError:
        return False
    _fem_rpms = sorted(rpms)
    _fem_gaps = sorted(gaps)
    return bool(_fem_T)


def _vecinos(sorted_vals: list[float], x: float) -> tuple[float, float]:
    """Devuelve (x0, x1) que acotan x en una lista ordenada (clamp en bordes)."""
    if not sorted_vals:
        return (x, x)
    if x <= sorted_vals[0]:
        return (sorted_vals[0], sorted_vals[0])
    if x >= sorted_vals[-1]:
        return (sorted_vals[-1], sorted_vals[-1])
    i = bisect.bisect_left(sorted_vals, x)
    if abs(sorted_vals[i] - x) < 1e-9:
        return (sorted_vals[i], sorted_vals[i])
    return (sorted_vals[i - 1], sorted_vals[i])


def _interp1(x: float, x0: float, x1: float, y0: float, y1: float) -> float:
    if abs(x1 - x0) < 1e-12:
        return y0
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0)


def _lookup_tl_fem(rpm: float, airgap_mm: float) -> float | None:
    """Interpolación bilineal de TL teórico en la malla FEM (RPM × airgap)."""
    if not _fem_T:
        return None
    r0, r1 = _vecinos(_fem_rpms, rpm)
    g0, g1 = _vecinos(_fem_gaps, airgap_mm)
    try:
        T00 = _fem_T[(r0, g0)]
        T01 = _fem_T[(r0, g1)]
        T10 = _fem_T[(r1, g0)]
        T11 = _fem_T[(r1, g1)]
    except KeyError:
        return None
    T_g_r0 = _interp1(airgap_mm, g0, g1, T00, T01)
    T_g_r1 = _interp1(airgap_mm, g0, g1, T10, T11)
    return _interp1(rpm, r0, r1, T_g_r0, T_g_r1)


_cargar_fem_csv()

# Confirmación de tara desde ESP32
_tara_ok_event = threading.Event()  # set() cuando llega {"status":"tara_ok"}
_I_fuente = 0.0

# Tipo de prueba activa ("prueba1" | "prueba2")
_prueba_activa: str = "prueba1"

# Handles de conexión
_mqtt_client:  mqtt.Client | None   = None
_serial_port:  serial.Serial | None = None


# ============================================================
#  SEÑALES Qt
# ============================================================
class _Señales(QObject):
    conexion_msg      = pyqtSignal(str)
    datos_nuevos      = pyqtSignal()
    punto_capturado   = pyqtSignal(int)   # emite el número de punto
    escalon_progreso  = pyqtSignal(str)   # texto de estado del escalón
    captura_p2_prog   = pyqtSignal(str)   # progreso captura P2 ("⏳ 7 s restantes …")
    captura_p2_lista  = pyqtSignal()      # emitida cuando el punto queda listo


señales = _Señales()


# ============================================================
#  CALCULOS DERIVADOS
# ============================================================
def _calcular_derivadas(n_rpm: float, TL_Nm: float, V: float, I: float):
    """Retorna (omega, Pe, Pm, eta) a partir de las variables base."""
    omega = 2.0 * math.pi * n_rpm / 60.0
    Pe    = V * I
    Pm    = TL_Nm * omega
    eta   = (Pm / Pe * 100.0) if Pe > 1e-6 else 0.0
    return omega, Pe, Pm, eta


# ============================================================
#  PROCESAMIENTO DE DATOS  (ESP32 → buffer compartido)
# ============================================================
def _procesar(data: dict):
    global _ultima_rpm_ref, _ultimo_torque_v, _V_fuente, _I_fuente

    n_rpm     = float(data.get("rpm_final",  0))
    torq_raw  = float(data.get("torque_Nm",  0))
    FL_N      = float(data.get("force_N",    0))
    t         = time.time() - _inicio_tiempo

    # Lógica de actualización de torque (solo cuando RPM cambia ≥ umbral)
    if _ultima_rpm_ref is None:
        _ultimo_torque_v = torq_raw
        _ultima_rpm_ref  = n_rpm
    elif abs(n_rpm - _ultima_rpm_ref) >= RPM_CHANGE_THRESHOLD:
        _ultimo_torque_v = torq_raw
        _ultima_rpm_ref  = n_rpm

    TL = _ultimo_torque_v

    # Variables de la fuente (leídas en otro hilo)
    V = _V_fuente
    I = _I_fuente

    omega, Pe, Pm, eta = _calcular_derivadas(n_rpm, TL, V, I)

    # Registro completo
    registro = {
        "t_s":          round(t, 3),
        "V_V":          round(V, 4),
        "I_A":          round(I, 4),
        "n_rpm":        round(n_rpm, 2),
        "omega_rads":   round(omega, 4),
        "FL_N":         round(FL_N, 6),
        "TL_Nm":        round(TL, 8),
        "Pe_W":         round(Pe, 4),
        "Pm_W":         round(Pm, 6),
        "eta_pct":      round(eta, 4),
        # Auxiliares ESP32
        "rpm_filtered": round(float(data.get("rpm_filtered", 0)), 2),
        "rpm_count":    round(float(data.get("rpm_count",    0)), 2),
        "rpm_period":   round(float(data.get("rpm_period",   0)), 2),
        "mass_g":       round(float(data.get("mass_g",       0)), 4),
        "kalman_gain":  round(float(data.get("kalman_gain",  0)), 5),
        "raw":          round(float(data.get("raw",          0)), 2),
        "zero_raw":     round(float(data.get("zero_raw",     0)), 2),
        "seq":          int(data.get("seq", 0)),
    }

    with _lock:
        _registros.append(registro)

        _t_buf.append(t);     _n_buf.append(n_rpm)
        _w_buf.append(omega); _FL_buf.append(FL_N)
        _TL_buf.append(TL);   _V_buf.append(V)
        _I_buf.append(I);     _Pe_buf.append(Pe)
        _Pm_buf.append(Pm);   _eta_buf.append(eta)

        # Limitar buffer de graficación
        if len(_t_buf) > MAX_POINTS:
            for buf in (_t_buf, _n_buf, _w_buf, _FL_buf, _TL_buf,
                        _V_buf, _I_buf, _Pe_buf, _Pm_buf, _eta_buf):
                buf.pop(0)

    señales.datos_nuevos.emit()


# ============================================================
#  FUENTE KEYSIGHT — PyVISA  (hilo background)
# ============================================================
def _leer_fuente_loop():
    """
    Lee V e I del canal 1 de la fuente Keysight en loop.
    Guarda el handle en _fuente_inst para que la GUI pueda enviar comandos.
    """
    global _V_fuente, _I_fuente, _fuente_inst, _fuente_output_deseada
    try:
        import pyvisa  # type: ignore
        rm   = pyvisa.ResourceManager()
        inst = rm.open_resource(VISA_ADDRESS)
        inst.timeout           = 5000
        inst.write_termination = "\n"
        inst.read_termination  = "\n"

        # Seleccionar CH1, limitar corriente a 1 A, voltaje 0
        # y respetar el estado de salida solicitado por la interfaz.
        inst.write("INST:SEL CH1")
        inst.write(f"CURR {FUENTE_I_LIMITE:.3f}")
        inst.write("VOLT 0")
        inst.write("OUTP ON" if _fuente_output_deseada else "OUTP OFF")

        _fuente_inst = inst   # exponer handle para controles GUI

        señales.conexion_msg.emit(
            f"Fuente Keysight conectada · {inst.query('*IDN?').strip()[:40]}"
        )

        while True:
            try:
                _V_fuente = float(inst.query("MEAS:VOLT?").strip())
                _I_fuente = float(inst.query("MEAS:CURR?").strip())
            except Exception:
                pass
            time.sleep(FUENTE_INTERVAL)

    except Exception as e:
        señales.conexion_msg.emit(f"PyVISA: {e} (sin fuente, V=0 I=0)")


def _set_corriente_limite(i: float = FUENTE_I_LIMITE):
    """Fija el límite de corriente del CH1 (CURR)."""
    if _fuente_inst is None:
        return
    try:
        _fuente_inst.write("INST:SEL CH1")
        _fuente_inst.write(f"CURR {i:.3f}")
    except Exception:
        pass


def _set_voltaje(v: float):
    """Envía VOLT <v> al CH1 y reafirma el límite de 1 A. Seguro para llamar desde la GUI."""
    if _fuente_inst is None:
        return
    try:
        _fuente_inst.write("INST:SEL CH1")
        _fuente_inst.write(f"CURR {FUENTE_I_LIMITE:.3f}")
        _fuente_inst.write(f"VOLT {v:.3f}")
    except Exception:
        pass


def _set_output(on: bool):
    """Enciende o apaga la salida CH1 y recuerda el estado solicitado."""
    global _fuente_output_deseada
    _fuente_output_deseada = bool(on)
    if _fuente_inst is None:
        return
    try:
        _fuente_inst.write("INST:SEL CH1")
        _fuente_inst.write(f"CURR {FUENTE_I_LIMITE:.3f}")
        _fuente_inst.write("OUTP ON" if on else "OUTP OFF")
    except Exception:
        pass


def _run_escalones(v_ini: float, v_fin: float, paso: float, duracion: float):
    """
    Hilo background para prueba de escalones de voltaje.
    Recorre de v_ini a v_fin en pasos de 'paso' voltios,
    manteniendo cada escalón 'duracion' segundos.
    Captura un punto estable al final de cada escalón.
    """
    _escalon_stop.clear()

    import math as _math
    n_pasos = max(1, round((_math.fabs(v_fin - v_ini)) / paso) + 1)
    signo   = 1.0 if v_fin >= v_ini else -1.0
    voltajes = [round(v_ini + signo * paso * i, 3) for i in range(n_pasos)]
    # Asegurar que el último valor sea exactamente v_fin
    voltajes[-1] = v_fin

    total = len(voltajes)
    for idx, v in enumerate(voltajes, start=1):
        if _escalon_stop.is_set():
            señales.escalon_progreso.emit("Detenido por el usuario")
            _set_voltaje(0.0)
            return

        _set_voltaje(v)
        señales.escalon_progreso.emit(
            f"Escalón {idx}/{total} · {v:.1f} V"
        )

        # Esperar duracion segundos o hasta que se detenga, con cuenta regresiva
        t_fin = time.time() + duracion
        while time.time() < t_fin:
            if _escalon_stop.is_set():
                señales.escalon_progreso.emit("Detenido por el usuario")
                _set_voltaje(0.0)
                return
            restante = int(t_fin - time.time()) + 1
            señales.escalon_progreso.emit(
                f"Escalón {idx}/{total} · {v:.1f} V · {restante}s"
            )
            time.sleep(0.5)

        # Capturar punto al final del escalón
        _capturar_punto_estable()

    _set_voltaje(0.0)
    señales.escalon_progreso.emit(f"Completado · {total} escalones")


# ============================================================
#  CAPTURA DE PUNTO ÚNICO — PRUEBA 2
# ============================================================
def _run_captura_p2(duracion_s: float = 10.0, distancia_mm: float = 0.0):
    """
    Hilo background. Muestrea todas las variables durante duracion_s segundos,
    calcula el promedio y agrega un único punto a _p2_puntos (incluye distancia).
    Emite captura_p2_prog cada segundo y captura_p2_lista al terminar.
    """
    _captura_p2_stop.clear()
    muestras: dict[str, list[float]] = {
        "TL": [], "n": [], "I": [], "Pm": [], "eta": []
    }

    t_ini = time.time()
    while True:
        elapsed = time.time() - t_ini
        if elapsed >= duracion_s or _captura_p2_stop.is_set():
            break
        restante = int(duracion_s - elapsed) + 1
        señales.captura_p2_prog.emit(f"⏳  {restante} s restantes…")

        with _lock:
            if _TL_buf:
                muestras["TL"].append(_TL_buf[-1])
                muestras["n"].append(_n_buf[-1])
                muestras["I"].append(_I_buf[-1])
                muestras["Pm"].append(_Pm_buf[-1])
                muestras["eta"].append(_eta_buf[-1])
        time.sleep(0.25)

    if _captura_p2_stop.is_set():
        señales.captura_p2_prog.emit("Captura cancelada")
        return

    if muestras["TL"]:
        punto = {k: sum(v) / len(v) for k, v in muestras.items()}
        punto["d"] = float(distancia_mm)
        tl_teo = _lookup_tl_fem(punto["n"], punto["d"])
        punto["TL_teo"] = tl_teo
        with _lock:
            _p2_puntos.append(punto)
        if tl_teo is None:
            señales.captura_p2_prog.emit(
                f"✓  Punto #{len(_p2_puntos)} · d={punto['d']:.1f} mm · "
                f"n={punto['n']:.0f} RPM · TL exp={punto['TL']:.4f} N·m · "
                f"TL teó. no encontrado"
            )
        else:
            señales.captura_p2_prog.emit(
                f"✓  Punto #{len(_p2_puntos)} · d={punto['d']:.1f} mm · "
                f"n={punto['n']:.0f} RPM · TL exp={punto['TL']:.4f} · "
                f"TL teó.={tl_teo:.4f} N·m"
            )
    else:
        señales.captura_p2_prog.emit("Sin datos durante la captura")

    señales.captura_p2_lista.emit()


# ============================================================
#  MQTT
# ============================================================
def _on_connect(client, userdata, flags, rc, props):
    if rc == 0:
        client.subscribe(TOPIC_TELEMETRY)
        señales.conexion_msg.emit(f"MQTT conectado · HiveMQ Cloud")
    else:
        señales.conexion_msg.emit(f"Error MQTT código {rc}")


def _on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode("utf-8"))
        status = data.get("status", "")
        if status == "tara_ok":
            _tara_ok_event.set()
            señales.conexion_msg.emit("ESP32: tara confirmada ✓")
            return
        if status == "tara_error":
            señales.conexion_msg.emit("ESP32: ERROR en tara — HX711 sin respuesta")
            return
        _procesar(data)
    except Exception:
        pass


def _iniciar_mqtt_loop():
    global _mqtt_client
    c = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        client_id="Dashboard_FrenoMotorDC",
    )
    c.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    # HiveMQ Cloud tiene certificado CA válido; se usa el trust store del sistema.
    c.tls_set(cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLS)
    c.tls_insecure_set(False)
    c.on_connect = _on_connect
    c.on_message = _on_message
    _mqtt_client = c
    try:
        señales.conexion_msg.emit("Conectando a HiveMQ Cloud…")
        c.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
        c.loop_forever()
    except Exception as e:
        señales.conexion_msg.emit(f"Error MQTT: {e}")


def _publicar_cmd(cmd: str):
    if _mqtt_client and _mqtt_client.is_connected():
        _mqtt_client.publish(TOPIC_CMD, json.dumps({"cmd": cmd}))


# ============================================================
#  SERIAL
# ============================================================
def _buscar_puerto() -> str:
    for p in list_ports.comports():
        d = p.device
        if any(k in d for k in ("usbmodem", "usbserial", "COM", "ttyUSB", "ttyACM")):
            return d
    raise RuntimeError("No se encontró la ESP32 por USB.")


def _leer_serial_loop(ser: serial.Serial):
    while True:
        try:
            linea = ser.readline().decode("utf-8", errors="ignore").strip()
            if linea.startswith("DATA:"):
                try:
                    _procesar(json.loads(linea[5:]))
                except json.JSONDecodeError:
                    pass
        except Exception:
            time.sleep(0.3)


# ============================================================
#  EXPORTACION CSV
# ============================================================
def _exportar_csv(directorio: str, prueba: str) -> tuple[str, str]:
    """
    Genera dos archivos CSV en el directorio indicado:
      1. Datos completos (serie temporal).
      2. Puntos de operación estables capturados.
    Retorna las rutas generadas.
    """
    ts    = datetime.now().strftime("%Y%m%d_%H%M%S")
    base  = f"{prueba}_{ts}"

    # ── 1. Serie temporal completa
    ruta_raw = os.path.join(directorio, f"{base}_datos.csv")
    with _lock:
        regs = list(_registros)

    if regs:
        campos = list(regs[0].keys())
        with open(ruta_raw, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=campos)
            w.writeheader()
            w.writerows(regs)

    # ── 2. Puntos de operación
    ruta_pts = os.path.join(directorio, f"{base}_puntos.csv")
    with _lock:
        puntos = list(_puntos_op)

    if puntos:
        campos_p = list(puntos[0].keys())
        with open(ruta_pts, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=campos_p)
            w.writeheader()
            w.writerows(puntos)

    return ruta_raw, ruta_pts


def _capturar_punto_estable(ventana: int = 10) -> dict | None:
    """
    Promedia los últimos `ventana` registros y los guarda como
    un punto de operación estable.
    """
    with _lock:
        if len(_registros) < ventana:
            return None
        muestra = _registros[-ventana:]

    def prom(campo):
        vals = [r[campo] for r in muestra if isinstance(r[campo], (int, float))]
        return round(sum(vals) / len(vals), 6) if vals else 0.0

    campos_num = [
        "V_V", "I_A", "n_rpm", "omega_rads",
        "FL_N", "TL_Nm", "Pe_W", "Pm_W", "eta_pct",
    ]
    punto = {
        "punto_num":  len(_puntos_op) + 1,
        "t_captura":  round(time.time() - _inicio_tiempo, 1),
        "prueba":     _prueba_activa,
        **{c: prom(c) for c in campos_num},
    }

    with _lock:
        _puntos_op.append(punto)

    señales.punto_capturado.emit(len(_puntos_op))
    return punto


# ============================================================
#  PALETA  &  ESTILOS
# ============================================================
_BG      = "#0D1117"
_PANEL   = "#161B22"
_BORDER  = "#30363D"
_PRI     = "#E6EDF3"
_SEC     = "#8B949E"
_BLUE    = "#1F6FEB"
_BLUE_H  = "#388BFD"
_GREEN   = "#238636"
_GREEN_H = "#2EA043"
_ORANGE  = "#F0883E"
_CYAN    = "#00D4FF"
_RED_C   = "#FF6B35"
_YELLOW  = "#D29922"
_PURPLE  = "#BC8CFF"

STYLE_APP = f"""
QWidget {{
    background-color: {_BG};
    color: {_PRI};
    font-family: 'Segoe UI', 'Helvetica Neue', Arial, sans-serif;
}}
QPushButton {{
    background-color: {_BLUE};
    color: white;
    border: none;
    border-radius: 8px;
    padding: 10px 24px;
    font-size: 13px;
    font-weight: bold;
}}
QPushButton:hover   {{ background-color: {_BLUE_H}; }}
QPushButton:pressed {{ background-color: #1158C7; }}
QPushButton:disabled {{
    background-color: #21262D;
    color: {_SEC};
}}
QScrollArea {{ border: none; background: transparent; }}
QScrollBar:vertical {{
    background: {_PANEL};
    width: 6px;
    border-radius: 3px;
}}
QScrollBar::handle:vertical {{
    background: {_BORDER};
    border-radius: 3px;
}}
QFrame#sep {{
    background-color: {_BORDER};
    max-height: 1px;
    min-height: 1px;
}}
QFrame#card {{
    background-color: {_PANEL};
    border: 1px solid {_BORDER};
    border-radius: 14px;
}}
QFrame#panel_vars {{
    background-color: {_PANEL};
    border: 1px solid {_BORDER};
    border-radius: 10px;
}}
"""


def _sep() -> QFrame:
    f = QFrame(); f.setObjectName("sep")
    f.setFrameShape(QFrame.Shape.HLine)
    return f


def _card(width: int = 560) -> QFrame:
    c = QFrame(); c.setObjectName("card"); c.setFixedWidth(width)
    return c


# ============================================================
#  PANTALLA 0 — BIENVENIDA
# ============================================================
class _PantallaBienvenida(QWidget):

    def __init__(self, siguiente):
        super().__init__()
        root = QVBoxLayout(self)
        root.setAlignment(Qt.AlignmentFlag.AlignCenter)
        root.setContentsMargins(0, 0, 0, 0)

        card = _card(560)
        v = QVBoxLayout(card)
        v.setSpacing(14)
        v.setContentsMargins(56, 48, 56, 48)

        ico = QLabel("⚙")
        ico.setAlignment(Qt.AlignmentFlag.AlignCenter)
        ico.setStyleSheet(f"font-size: 64px; color: {_BLUE};")
        v.addWidget(ico)

        for txt, st in [
            ("Sistema de Monitoreo",
             f"font-size: 28px; font-weight: bold; color: {_PRI};"),
            ("Freno Magnético · Motor 57BLDC",
             f"font-size: 18px; font-weight: bold; color: {_BLUE};"),
        ]:
            l = QLabel(txt); l.setAlignment(Qt.AlignmentFlag.AlignCenter)
            l.setStyleSheet(st); v.addWidget(l)

        v.addWidget(_sep())

        for txt, st in [
            ("Proyecto de Diseño en Mecatrónica",
             f"font-size: 12px; color: {_SEC};"),
            ("Elaborado por:", f"font-size: 12px; color: {_SEC};"),
        ]:
            l = QLabel(txt); l.setAlignment(Qt.AlignmentFlag.AlignCenter)
            l.setStyleSheet(st); v.addWidget(l)

        for nombre in ("Juan Andrés Sanchez", "Sofía Vega", "Andrés Felipe Trujillo"):
            l = QLabel(f"· {nombre}")
            l.setAlignment(Qt.AlignmentFlag.AlignCenter)
            l.setStyleSheet(f"font-size: 14px; color: {_PRI};")
            v.addWidget(l)

        v.addSpacing(8); v.addWidget(_sep()); v.addSpacing(4)

        btn = QPushButton("Iniciar Dashboard")
        btn.setFixedHeight(46); btn.clicked.connect(siguiente)
        v.addWidget(btn)

        root.addWidget(card, alignment=Qt.AlignmentFlag.AlignCenter)


# ============================================================
#  PANTALLA 1 — CONFIGURACION + CALIBRACION
# ============================================================
class _PantallaConfig(QWidget):

    def __init__(self, siguiente):
        super().__init__()
        self._siguiente = siguiente
        self._modo = "serial"

        root = QVBoxLayout(self)
        root.setAlignment(Qt.AlignmentFlag.AlignCenter)
        root.setContentsMargins(0, 0, 0, 0)

        card = _card(620)
        v = QVBoxLayout(card)
        v.setSpacing(16)
        v.setContentsMargins(52, 40, 52, 40)

        tit = QLabel("Configuración del Sistema")
        tit.setAlignment(Qt.AlignmentFlag.AlignCenter)
        tit.setStyleSheet(f"font-size: 21px; font-weight: bold; color: {_PRI};")
        v.addWidget(tit); v.addWidget(_sep())

        lbl = QLabel("Medio de comunicación con la ESP32")
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet(f"font-size: 12px; color: {_SEC};")
        v.addWidget(lbl)

        row = QHBoxLayout(); row.setSpacing(12)
        self._btn_ser = QPushButton("🔌   USB Serial")
        self._btn_ser.setFixedHeight(46); self._btn_ser.setCheckable(True)
        self._btn_ser.setChecked(True)
        self._btn_ser.clicked.connect(lambda: self._set_modo("serial"))
        self._btn_mq  = QPushButton("🌐   MQTT · HiveMQ")
        self._btn_mq.setFixedHeight(46); self._btn_mq.setCheckable(True)
        self._btn_mq.clicked.connect(lambda: self._set_modo("mqtt"))
        row.addWidget(self._btn_ser); row.addWidget(self._btn_mq)
        v.addLayout(row); self._refrescar_estilo()

        self._lbl_desc = QLabel(
            "La ESP32 se conecta directamente al computador por USB."
        )
        self._lbl_desc.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_desc.setWordWrap(True)
        self._lbl_desc.setStyleSheet(f"font-size: 12px; color: {_SEC};")
        v.addWidget(self._lbl_desc)

        v.addWidget(_sep())

        self._lbl_estado = QLabel("Esperando calibración…")
        self._lbl_estado.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_estado.setStyleSheet(f"font-size: 12px; color: {_ORANGE};")
        v.addWidget(self._lbl_estado)

        self._btn_cal = QPushButton("⚙   Calibrar Sistema")
        self._btn_cal.setFixedHeight(50)
        self._btn_cal.setStyleSheet(f"""
            QPushButton {{
                background-color:{_GREEN}; color:white; border:none;
                border-radius:8px; font-size:14px; font-weight:bold;
            }}
            QPushButton:hover   {{ background-color:{_GREEN_H}; }}
            QPushButton:disabled {{ background-color:#1C6B2A; color:#7EE787; }}
        """)
        self._btn_cal.clicked.connect(self._calibrar)
        v.addWidget(self._btn_cal)

        self._btn_go = QPushButton("Continuar →")
        self._btn_go.setFixedHeight(44); self._btn_go.setEnabled(False)
        self._btn_go.clicked.connect(siguiente)
        v.addWidget(self._btn_go)

        root.addWidget(card, alignment=Qt.AlignmentFlag.AlignCenter)
        señales.conexion_msg.connect(lambda m: self._lbl_estado.setText(m))

    def _set_modo(self, modo):
        self._modo = modo
        self._btn_ser.setChecked(modo == "serial")
        self._btn_mq.setChecked(modo == "mqtt")
        self._refrescar_estilo()
        self._lbl_desc.setText(
            "La ESP32 se conecta directamente al computador por USB."
            if modo == "serial" else
            "La ESP32 se comunica vía WiFi con el broker HiveMQ Cloud."
        )

    def _refrescar_estilo(self):
        for btn, nombre in [(self._btn_ser, "serial"), (self._btn_mq, "mqtt")]:
            activo = self._modo == nombre
            s = (f"background-color:#0D419D; border:2px solid {_BLUE};"
                 if activo else
                 f"background-color:#21262D; color:{_SEC}; border:2px solid transparent;")
            btn.setStyleSheet(f"""
                QPushButton {{
                    color:{_PRI}; border-radius:8px; padding:10px 18px;
                    font-size:13px; font-weight:bold; {s}
                }}
                QPushButton:hover {{ background-color:#1F3A7A; color:white; }}
            """)

    def _calibrar(self):
        self._btn_cal.setEnabled(False); self._btn_cal.setText("Calibrando…")
        self._lbl_estado.setStyleSheet(f"font-size:12px; color:{_ORANGE};")
        self._lbl_estado.setText("Iniciando calibración…")
        if self._modo == "serial":
            threading.Thread(target=self._cal_serial, daemon=True).start()
        else:
            threading.Thread(target=self._cal_mqtt, daemon=True).start()

    def _cal_serial(self):
        global _serial_port, _inicio_tiempo
        try:
            puerto = _buscar_puerto()
            ser = serial.Serial(puerto, SERIAL_BAUD, timeout=0.5)
            _serial_port = ser
            señales.conexion_msg.emit(f"Puerto USB: {puerto}")

            # Pequeña pausa para que la ESP32 termine de arrancar
            time.sleep(2.0)

            # Enviar comando de calibración
            ser.write(b"C\n")
            señales.conexion_msg.emit("Calibración enviada — esperando ESP32…")
            _inicio_tiempo = time.time()

            # Esperar confirmación "Tara OK" en el serial (máx 30 s)
            deadline = time.time() + 30
            confirmado = False
            while time.time() < deadline:
                linea = ser.readline().decode("utf-8", errors="ignore").strip()
                restante = int(deadline - time.time()) + 1
                if linea:
                    señales.conexion_msg.emit(f"ESP32: {linea}")
                if "Tara OK" in linea:
                    confirmado = True
                    break
                señales.conexion_msg.emit(f"Calibrando galga… {restante}s")

            if confirmado:
                QTimer.singleShot(0, self._cal_ok)
            else:
                señales.conexion_msg.emit(
                    "Timeout: ESP32 no confirmó la tara — verifica conexión USB"
                )
                QTimer.singleShot(0, self._cal_timeout)
                return

            # Continuar leyendo datos de telemetría en este mismo hilo
            _leer_serial_loop(ser)

        except Exception as e:
            señales.conexion_msg.emit(f"Error serial: {e}")

    def _cal_mqtt(self):
        global _inicio_tiempo

        # 1. Arrancar el cliente MQTT
        threading.Thread(target=_iniciar_mqtt_loop, daemon=True).start()

        # 2. Esperar conexión al broker (máx 20 s)
        deadline = time.time() + 20
        while time.time() < deadline:
            if _mqtt_client and _mqtt_client.is_connected():
                break
            time.sleep(0.4)

        if not (_mqtt_client and _mqtt_client.is_connected()):
            señales.conexion_msg.emit("Error: no se pudo conectar al broker MQTT")
            QTimer.singleShot(0, self._cal_timeout)
            return

        # 3. Enviar comando de calibración
        _tara_ok_event.clear()
        _inicio_tiempo = time.time()
        _publicar_cmd("calibrate")
        señales.conexion_msg.emit("Calibración enviada — esperando ESP32…")

        # 4. Esperar confirmación tara_ok de la ESP32 (máx 30 s)
        for i in range(30):
            if _tara_ok_event.wait(timeout=1.0):
                # ESP32 confirmó tara exitosa
                QTimer.singleShot(0, self._cal_ok)
                return
            restante = 30 - i - 1
            señales.conexion_msg.emit(
                f"Calibrando galga… {restante}s"
            )

        # Timeout: la ESP32 no respondió
        señales.conexion_msg.emit(
            "Timeout: ESP32 no confirmó la tara — verifica la conexión"
        )
        QTimer.singleShot(0, self._cal_timeout)

    def _cal_ok(self):
        self._btn_cal.setText("✓   Sistema Calibrado")
        self._btn_cal.setEnabled(False)
        self._lbl_estado.setStyleSheet("font-size:12px; color:#7EE787;")
        self._lbl_estado.setText("Sistema listo para medir.")
        self._btn_go.setEnabled(True)

        # Iniciar hilo de lectura de fuente Keysight (no bloqueante si no está disponible)
        threading.Thread(target=_leer_fuente_loop, daemon=True).start()

    def _cal_timeout(self):
        """La ESP32 no confirmó la tara en el tiempo esperado."""
        self._btn_cal.setText("⚠  Reintentar Calibración")
        self._btn_cal.setEnabled(True)
        self._lbl_estado.setStyleSheet("font-size:12px; color:#FF7B72;")
        self._lbl_estado.setText(
            "Sin respuesta de la ESP32.\n"
            "Verifica WiFi y conexión MQTT, luego reintenta."
        )


# ============================================================
#  PANTALLA 2 — SELECCION DE PRUEBA
# ============================================================
class _PantallaPrueba(QWidget):

    def __init__(self, siguiente):
        super().__init__()
        self._siguiente = siguiente

        root = QVBoxLayout(self)
        root.setAlignment(Qt.AlignmentFlag.AlignCenter)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(24)

        tit = QLabel("Seleccione el Tipo de Ensayo")
        tit.setAlignment(Qt.AlignmentFlag.AlignCenter)
        tit.setStyleSheet(f"font-size: 22px; font-weight: bold; color: {_PRI};")
        root.addWidget(tit)

        cards_row = QHBoxLayout()
        cards_row.setSpacing(28)
        cards_row.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # Tarjeta Prueba 1
        self._c1 = self._hacer_card(
            numero="1",
            titulo="Velocidad en función del Voltaje",
            descripcion=(
                "Freno en posición de entrehierro máximo (12 mm).\n\n"
                "Se varía el voltaje de alimentación:\n"
                "V = {6, 8, 10, 12, 14, 16, 18, 20, 22, 24} V\n\n"
                "~30 s por nivel · Motor en régimen estable.\n\n"
                "Resultado: curva n(V) en carga mínima."
            ),
            color=_CYAN,
            prueba="prueba1",
        )
        self._c2 = self._hacer_card(
            numero="2",
            titulo="Caracterización con Freno Magnético",
            descripcion=(
                "Voltaje constante: 24 V.\n\n"
                "Se reduce el entrehierro progresivamente:\n"
                "g = {12, 11, 10, …, 2, 1} mm\n\n"
                "Esperar estabilización por posición.\n\n"
                "Resultado: curvas n(TL), I(TL), Pm(TL), η(TL)."
            ),
            color=_RED_C,
            prueba="prueba2",
        )
        cards_row.addWidget(self._c1)
        cards_row.addWidget(self._c2)
        root.addLayout(cards_row)

        self._btn_ir = QPushButton("Iniciar Ensayo →")
        self._btn_ir.setFixedSize(260, 48)
        self._btn_ir.setEnabled(False)
        self._btn_ir.clicked.connect(self._ir)
        root.addWidget(self._btn_ir, alignment=Qt.AlignmentFlag.AlignCenter)

        self._seleccion = None

    def _hacer_card(self, numero, titulo, descripcion, color, prueba) -> QFrame:
        c = QFrame(); c.setObjectName("card"); c.setFixedSize(320, 380)
        c.setCursor(Qt.CursorShape.PointingHandCursor)
        v = QVBoxLayout(c); v.setSpacing(12); v.setContentsMargins(28, 28, 28, 28)

        badge = QLabel(numero)
        badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        badge.setFixedSize(48, 48)
        badge.setStyleSheet(
            f"font-size:24px; font-weight:bold; color:white;"
            f" background-color:{color}; border-radius:24px;"
        )
        v.addWidget(badge, alignment=Qt.AlignmentFlag.AlignHCenter)

        t = QLabel(titulo); t.setAlignment(Qt.AlignmentFlag.AlignCenter)
        t.setWordWrap(True)
        t.setStyleSheet(f"font-size:14px; font-weight:bold; color:{_PRI};")
        v.addWidget(t)

        d = QLabel(descripcion); d.setWordWrap(True)
        d.setStyleSheet(f"font-size:12px; color:{_SEC}; line-height:1.5;")
        v.addWidget(d)

        v.addStretch()

        # Clic en la tarjeta → seleccionar
        c.mousePressEvent = lambda _ev, p=prueba: self._seleccionar(p)

        return c

    def _seleccionar(self, prueba: str):
        global _prueba_activa
        _prueba_activa = prueba
        self._seleccion = prueba

        colores = {"prueba1": _CYAN, "prueba2": _RED_C}
        for card, p in [(self._c1, "prueba1"), (self._c2, "prueba2")]:
            col = colores[p] if p == prueba else _BORDER
            card.setStyleSheet(
                f"QFrame#card {{ background-color:{_PANEL};"
                f" border:2px solid {col}; border-radius:14px; }}"
            )

        self._btn_ir.setEnabled(True)

    def _ir(self):
        self._siguiente(self._seleccion)


# ============================================================
#  PANTALLA 3 — GRAFICAS + PANEL DE VARIABLES + EXPORTACION
# ============================================================
class _PantallaGraficas(QWidget):

    def __init__(self, volver_cb=None):
        super().__init__()
        self._prueba        = "prueba1"
        self._volver_cb     = volver_cb
        self._ensayo_activo = False
        self._construir()

    def set_prueba(self, prueba: str):
        self._prueba = prueba
        self._ensayo_activo = False
        nombres = {
            "prueba1": "Prueba 1 · Velocidad vs Voltaje",
            "prueba2": "Prueba 2 · Caracterización con Freno",
        }
        self._lbl_titulo.setText(nombres.get(prueba, "Ensayo"))

        es_p1 = (prueba == "prueba1")
        self._panel_stack.setCurrentIndex(0 if es_p1 else 1)
        self._graf_stack.setCurrentIndex(0 if es_p1 else 1)

        if es_p1:
            # Mantener sincronizado el estado real de la fuente con el botón de P1.
            _set_output(self._btn_outp.isChecked())
        else:
            # REQUISITO P2: cada vez que se entra a la Prueba 2, la salida
            # debe comenzar realmente apagada y las curvas teóricas ocultas.
            self._btn_outp_p2.setChecked(False)
            _set_output(False)
            self._ensayo_activo = False
            self._spin_dist.setValue(_p2_distancia_mm)
            for curva in (self._p2_ct_w, self._p2_ct_i,
                          self._p2_ct_pm, self._p2_ct_n,
                          self._p2_ct_car_n, self._p2_ct_car_i,
                          self._p2_ct_car_pm, self._p2_ct_car_eta):
                curva.setData([], [])

    # ── construcción de la UI ─────────────────────────────────
    def _construir(self):
        root = QVBoxLayout(self)
        root.setSpacing(8)
        root.setContentsMargins(16, 10, 16, 10)

        # ── Header
        hdr = QHBoxLayout()
        self._lbl_titulo = QLabel("Ensayo")
        self._lbl_titulo.setStyleSheet(
            f"font-size:15px; font-weight:bold; color:{_PRI};"
        )
        hdr.addWidget(self._lbl_titulo); hdr.addStretch()
        self._lbl_conn = QLabel("—")
        self._lbl_conn.setStyleSheet(f"font-size:11px; color:{_SEC};")
        hdr.addWidget(self._lbl_conn)
        root.addLayout(hdr); root.addWidget(_sep())

        # ── Cuerpo principal (panel izquierdo + gráficas)
        body = QHBoxLayout(); body.setSpacing(12)

        # Panel izquierdo — QStackedWidget: 0=P1, 1=P2 captura, 2=P2 características
        self._panel_stack = QStackedWidget()
        self._panel_stack.setFixedWidth(232)
        self._panel_p1 = self._hacer_panel_p1()
        self._panel_p2 = self._hacer_panel_p2()
        self._panel_p2_carac = self._hacer_panel_p2_carac()
        self._panel_stack.addWidget(self._panel_p1)        # index 0
        self._panel_stack.addWidget(self._panel_p2)        # index 1
        self._panel_stack.addWidget(self._panel_p2_carac)  # index 2
        body.addWidget(self._panel_stack, stretch=0)

        # Panel derecho — 0=P1 | 1=P2 (2 gráficas) | 2=P2 (4 características)
        self._graf_stack = QStackedWidget()

        # ── Prueba 1: dos gráficas apiladas ────────────────────
        w_p1 = QWidget(); col_p1 = QVBoxLayout(w_p1); col_p1.setSpacing(8); col_p1.setContentsMargins(0,0,0,0)
        self._plot_top, self._curva_top, self._curva_top2 = self._make_plot_dual()
        self._plot_bot, self._curva_bot, self._curva_bot2 = self._make_plot_dual()
        _pen_teo_n = pg.mkPen(color="#FFFFFF", width=1.8, style=Qt.PenStyle.DashLine)
        _pen_teo_v = pg.mkPen(color="#FFD700", width=1.8, style=Qt.PenStyle.DashLine)
        self._curva_top_teo = self._plot_top.plot(pen=_pen_teo_n, name="n(t) Teórico")
        self._curva_bot_teo = self._plot_bot.plot(pen=_pen_teo_v, name="V(t) Teórico")
        col_p1.addWidget(self._plot_top, stretch=1)
        col_p1.addWidget(self._plot_bot, stretch=1)
        self._habilitar_click_coords(self._plot_top)
        self._habilitar_click_coords(self._plot_bot)
        self._graf_stack.addWidget(w_p1)   # index 0

        _pen_exp = lambda c: pg.mkPen(color=c, width=2.2)
        _pen_teo = lambda c: pg.mkPen(color=c, width=1.8, style=Qt.PenStyle.DashLine)
        _sym = dict(symbol="o", symbolSize=8)

        def _style_plot(pw, title, ylabel, xlabel):
            pw.setBackground(_BG)
            pw.showGrid(x=True, y=True, alpha=0.15)
            pw.setTitle(title, color="#C9D1D9", size="11pt")
            for ax in ("left", "bottom"):
                pw.getAxis(ax).setPen(pg.mkPen(color=_BORDER))
                pw.getAxis(ax).setTextPen(pg.mkPen(color=_SEC))
            pw.setLabel("left", ylabel, color=_SEC)
            pw.setLabel("bottom", xlabel, color=_SEC)
            self._habilitar_click_coords(pw)

        # ── Prueba 2 captura: 2 gráficas apiladas ──────────────
        w_p2 = QWidget(); col_p2 = QVBoxLayout(w_p2); col_p2.setSpacing(8); col_p2.setContentsMargins(0,0,0,0)

        self._p2_plot_carac = pg.PlotWidget()
        _style_plot(self._p2_plot_carac,
                    "Curva característica del motor",
                    "I / n / η / Pm", "TL (N·m)")
        self._p2_plot_carac.addLegend(offset=(8, 8))
        self._p2_c_car_n   = self._p2_plot_carac.plot(pen=_pen_exp(_CYAN),    name="n (RPM)",  **_sym)
        self._p2_c_car_i   = self._p2_plot_carac.plot(pen=_pen_exp(_ORANGE),  name="I (A)",    **_sym)
        self._p2_c_car_pm  = self._p2_plot_carac.plot(pen=_pen_exp(_PURPLE),  name="Pm (W)",   **_sym)
        self._p2_c_car_eta = self._p2_plot_carac.plot(pen=_pen_exp("#58A6FF"), name="η (%)",   **_sym)
        self._p2_ct_car_n   = self._p2_plot_carac.plot(pen=_pen_teo("#FFFFFF"), name="n teó.")
        self._p2_ct_car_i   = self._p2_plot_carac.plot(pen=_pen_teo("#FFD700"), name="I teó.")
        self._p2_ct_car_pm  = self._p2_plot_carac.plot(pen=_pen_teo("#C0A0FF"), name="Pm teó.")
        self._p2_ct_car_eta = self._p2_plot_carac.plot(pen=_pen_teo("#A0D0FF"), name="η teó.")

        self._p2_plot_dist = pg.PlotWidget()
        _style_plot(self._p2_plot_dist,
                    "Torque de carga vs Distancia imán–eje",
                    "TL (N·m)", "Distancia (mm)")
        self._p2_plot_dist.addLegend(offset=(8, 8))
        self._p2_c_dist = self._p2_plot_dist.plot(
            pen=_pen_exp(_RED_C), name="TL exp", **_sym)
        self._p2_ct_dist = self._p2_plot_dist.plot(
            pen=_pen_teo("#FFD700"), name="TL teó. FEM",
            symbol="t", symbolSize=9, symbolBrush="#FFD700")

        col_p2.addWidget(self._p2_plot_carac, stretch=1)
        col_p2.addWidget(self._p2_plot_dist,  stretch=1)
        self._graf_stack.addWidget(w_p2)   # index 1

        # ── Prueba 2 extra: cuatro gráficas 2×2 (vs TL) ───────
        w_p2c = QWidget(); grid_p2 = QGridLayout(w_p2c); grid_p2.setSpacing(8); grid_p2.setContentsMargins(0,0,0,0)

        def _make_p2_plot(title, ylabel, color_exp, color_teo):
            pw = pg.PlotWidget()
            _style_plot(pw, title, ylabel, "TL (N·m)")
            c_exp = pw.plot(pen=_pen_exp(color_exp), name="Experimental", **_sym)
            c_teo = pw.plot(pen=_pen_teo(color_teo),  name="Teórico")
            return pw, c_exp, c_teo

        self._p2_plot_w,  self._p2_c_w,  self._p2_ct_w  = _make_p2_plot("n vs TL",  "Velocidad (RPM)",      _CYAN,   "#FFFFFF")
        self._p2_plot_i,  self._p2_c_i,  self._p2_ct_i  = _make_p2_plot("I vs TL",  "Corriente (A)",        _ORANGE, "#FFD700")
        self._p2_plot_pm, self._p2_c_pm, self._p2_ct_pm = _make_p2_plot("Pm vs TL", "Pot. mec. (W)",        _PURPLE, "#C0A0FF")
        self._p2_plot_n,  self._p2_c_n,  self._p2_ct_n  = _make_p2_plot("η vs TL",  "Eficiencia (%)",       "#58A6FF","#A0D0FF")

        grid_p2.addWidget(self._p2_plot_w,  0, 0)
        grid_p2.addWidget(self._p2_plot_i,  0, 1)
        grid_p2.addWidget(self._p2_plot_pm, 1, 0)
        grid_p2.addWidget(self._p2_plot_n,  1, 1)
        self._graf_stack.addWidget(w_p2c)  # index 2

        body.addWidget(self._graf_stack, stretch=1)

        root.addLayout(body, stretch=1)

        # ── Timer
        tmr = QTimer(self); tmr.timeout.connect(self._tick); tmr.start(100)
        señales.conexion_msg.connect(lambda m: self._lbl_conn.setText(m))
        señales.escalon_progreso.connect(self._on_escalon_progreso)
        señales.captura_p2_prog.connect(self._on_captura_p2_prog)
        señales.captura_p2_lista.connect(self._on_captura_p2_lista)

    # ── Helpers de panel compartidos ─────────────────────────────
    def _sty_spin_ctrl(self) -> str:
        return f"""
            QDoubleSpinBox {{
                background:#21262D; color:{_PRI};
                border:1px solid {_BORDER}; border-radius:4px;
                font-size:11px; padding:1px 3px;
            }}
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{
                width:14px; background:#30363D; border:none;
            }}
        """

    def _seccion_vars(self, v: QVBoxLayout, lbl_dict_attr: str):
        """Añade la tabla de 9 variables al layout v, guarda labels en self.<lbl_dict_attr>."""
        lbl_h = QLabel("Variables actuales")
        lbl_h.setStyleSheet(f"font-size:12px; font-weight:bold; color:{_SEC};")
        v.addWidget(lbl_h); v.addWidget(_sep())
        _vars_def = [
            ("n",  "RPM Final",    "RPM",   _CYAN),
            ("ω",  "Vel. angular", "rad/s", _CYAN),
            ("V",  "Voltaje",      "V",     _YELLOW),
            ("I",  "Corriente",    "A",     _YELLOW),
            ("FL", "F. galga",     "N",     _GREEN_H),
            ("TL", "Torque",       "N·m",   _RED_C),
            ("Pe", "Pot. eléct.",  "W",     _ORANGE),
            ("Pm", "Pot. mec.",    "W",     _PURPLE),
            ("η",  "Eficiencia",   "%",     "#58A6FF"),
        ]
        d: dict[str, QLabel] = {}
        for sym, nombre, unidad, color in _vars_def:
            row = QHBoxLayout(); row.setSpacing(4)
            l_sym = QLabel(sym); l_sym.setFixedWidth(24)
            l_sym.setStyleSheet(f"font-size:13px; font-weight:bold; color:{color};")
            l_nom = QLabel(nombre); l_nom.setStyleSheet(f"font-size:10px; color:{_SEC};")
            l_nom.setFixedWidth(76)
            l_val = QLabel("—")
            l_val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            l_val.setStyleSheet(f"font-size:12px; font-weight:bold; color:{_PRI};")
            l_uni = QLabel(unidad); l_uni.setStyleSheet(f"font-size:10px; color:{_SEC};")
            l_uni.setFixedWidth(34)
            row.addWidget(l_sym); row.addWidget(l_nom)
            row.addWidget(l_val); row.addWidget(l_uni)
            v.addLayout(row); d[sym] = l_val
        setattr(self, lbl_dict_attr, d)
        v.addWidget(_sep())

    def _seccion_fuente(self, v: QVBoxLayout, spin_attr: str, outp_attr: str,
                         inicial_on: bool = True):
        """Añade control fuente CH1 al layout v."""
        lbl_f = QLabel("Control Fuente · CH1")
        lbl_f.setStyleSheet(f"font-size:11px; font-weight:bold; color:{_YELLOW};")
        v.addWidget(lbl_f)

        spin = QDoubleSpinBox()
        spin.setRange(0.0, 30.0); spin.setSingleStep(0.5)
        spin.setDecimals(1); spin.setSuffix(" V"); spin.setValue(0.0)
        spin.setFixedHeight(28)
        spin.setStyleSheet(f"""
            QDoubleSpinBox {{
                background:#21262D; color:{_PRI};
                border:1px solid {_BORDER}; border-radius:5px;
                font-size:12px; padding:1px 5px;
            }}
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{
                width:16px; background:#30363D; border:none;
            }}
        """)
        setattr(self, spin_attr, spin)

        btn_ap = QPushButton("Aplicar")
        btn_ap.setFixedHeight(28)
        btn_ap.setStyleSheet(f"""
            QPushButton {{
                background-color:{_YELLOW}; color:#0D1117;
                border:none; border-radius:5px;
                font-size:11px; font-weight:bold;
            }}
            QPushButton:hover {{ background-color:#E3C000; }}
        """)
        def _on_aplicar():
            _set_voltaje(getattr(self, spin_attr).value())
            # En P1 se conserva el comportamiento anterior.
            # En P2, "Aplicar" SOLO fija el voltaje: las curvas teóricas
            # aparecen exclusivamente cuando la salida pasa de OFF a ON.
            if spin_attr == "_spin_volt":
                self._ensayo_activo = True
                self._mostrar_teo_p1()
        btn_ap.clicked.connect(_on_aplicar)
        row_f = QHBoxLayout(); row_f.setSpacing(6)
        row_f.addWidget(spin, stretch=1); row_f.addWidget(btn_ap)
        v.addLayout(row_f)

        sty_on  = f"""QPushButton {{
            background-color:#1A4D2E; color:#39D353;
            border:1px solid #39D353; border-radius:5px;
            font-size:11px; font-weight:bold;
            padding-top:2px; padding-bottom:6px;
        }} QPushButton:hover {{ background-color:#245C38; }}"""
        sty_off = f"""QPushButton {{
            background-color:#3D1515; color:#FF7B72;
            border:1px solid #FF7B72; border-radius:5px;
            font-size:11px; font-weight:bold;
            padding-top:2px; padding-bottom:6px;
        }} QPushButton:hover {{ background-color:#5C1F1F; }}"""
        btn_out = QPushButton("⚡  Salida ON" if inicial_on else "○  Salida OFF")
        btn_out.setCheckable(True); btn_out.setChecked(inicial_on)
        btn_out.setFixedHeight(32)
        btn_out.setStyleSheet(sty_on if inicial_on else sty_off)

        def _toggle(checked, b=btn_out, s_on=sty_on, s_off=sty_off, attr=outp_attr):
            _set_output(checked)
            b.setText("⚡  Salida ON" if checked else "○  Salida OFF")
            b.setStyleSheet(s_on if checked else s_off)

            # REQUISITO P2: al pasar la salida de OFF -> ON se habilita
            # la medición y se dibujan inmediatamente TODAS las curvas
            # teóricas que ya hayan sido cargadas.
            if attr == "_btn_outp_p2" and checked:
                self._ensayo_activo = True
                self._mostrar_teo_p2()

        btn_out.toggled.connect(_toggle)
        setattr(self, outp_attr, btn_out)
        v.addWidget(btn_out)
        v.addWidget(_sep())

    @staticmethod
    def _bloque_csv_ui(nombre_var: str, slot, parent_layout: QVBoxLayout):
        """Botón CSV + label estado. Retorna (btn, lbl)."""
        sty = f"""
            QPushButton {{
                background-color:#21262D; color:{_PRI};
                border:1px solid {_BORDER}; border-radius:5px;
                font-size:11px; font-weight:bold;
                text-align:left; padding-left:10px;
                padding-top:2px; padding-bottom:6px;
            }}
            QPushButton:hover {{ background-color:#30363D; }}
        """
        btn = QPushButton(f"📂  {nombre_var}")
        btn.setFixedHeight(32); btn.setStyleSheet(sty)
        btn.clicked.connect(slot)
        lbl = QLabel("sin archivo")
        lbl.setStyleSheet(f"font-size:9px; color:{_SEC}; padding-left:12px;")
        lbl.setFixedHeight(14)
        parent_layout.addWidget(btn)
        parent_layout.addWidget(lbl)
        return btn, lbl

    @staticmethod
    def _btn_nav(texto: str, color_txt: str, color_bg: str,
                 color_border: str, color_hover: str) -> QPushButton:
        b = QPushButton(texto); b.setFixedHeight(32)
        b.setStyleSheet(f"""
            QPushButton {{
                background-color:{color_bg}; color:{color_txt};
                border:1px solid {color_border};
                border-radius:6px; font-size:12px; font-weight:bold;
            }}
            QPushButton:hover {{ background-color:{color_hover}; }}
        """)
        return b

    # ─────────────────────────────────────────────────────────────
    #  Panel Prueba 1: Variables · Fuente · Escalones · CSV P1
    # ─────────────────────────────────────────────────────────────
    def _hacer_panel_p1(self) -> QFrame:
        panel = QFrame(); panel.setObjectName("panel_p1")
        v = QVBoxLayout(panel); v.setSpacing(5); v.setContentsMargins(10, 10, 10, 10)

        self._seccion_vars(v, "_lbl_vars_p1")
        self._seccion_fuente(v, "_spin_volt",  "_btn_outp")

        # ── Prueba de Escalones ────────────────────────────────
        lbl_esc = QLabel("Prueba de Escalones")
        lbl_esc.setStyleSheet(f"font-size:11px; font-weight:bold; color:{_CYAN};")
        v.addWidget(lbl_esc)

        sty_s = self._sty_spin_ctrl()
        def _spin(lo, hi, val, step, dec, suf):
            s = QDoubleSpinBox()
            s.setRange(lo, hi); s.setValue(val); s.setSingleStep(step)
            s.setDecimals(dec); s.setSuffix(suf); s.setFixedHeight(24)
            s.setStyleSheet(sty_s); return s

        def _col(label, widget):
            c = QVBoxLayout(); c.setSpacing(1)
            l = QLabel(label); l.setStyleSheet(f"font-size:9px; color:{_SEC};")
            c.addWidget(l); c.addWidget(widget); return c

        self._spin_v_ini = _spin(0.0,  30.0,  6.0,  0.5, 1, " V")
        self._spin_v_fin = _spin(0.0,  30.0, 24.0,  0.5, 1, " V")
        self._spin_paso  = _spin(0.5,  10.0,  1.0,  0.5, 1, " V")
        self._spin_dur   = _spin(5.0, 300.0, 10.0,  5.0, 0, " s")

        g = QGridLayout(); g.setSpacing(4)
        g.addLayout(_col("V inicio", self._spin_v_ini), 0, 0)
        g.addLayout(_col("V final",  self._spin_v_fin), 0, 1)
        g.addLayout(_col("Paso V",   self._spin_paso),  1, 0)
        g.addLayout(_col("Duración", self._spin_dur),   1, 1)
        v.addLayout(g)

        self._btn_esc_style_ini = f"""QPushButton {{
            background-color:#0D419D; color:white;
            border:none; border-radius:5px;
            font-size:11px; font-weight:bold;
        }} QPushButton:hover {{ background-color:#1158C7; }}"""
        self._btn_esc_style_det = f"""QPushButton {{
            background-color:#3D1515; color:#FF7B72;
            border:1px solid #FF7B72; border-radius:5px;
            font-size:11px; font-weight:bold;
        }} QPushButton:hover {{ background-color:#5C1F1F; }}"""
        self._btn_esc = QPushButton("▶  Iniciar Escalones")
        self._btn_esc.setCheckable(True); self._btn_esc.setFixedHeight(30)
        self._btn_esc.setStyleSheet(self._btn_esc_style_ini)
        self._btn_esc.toggled.connect(self._toggle_escalones)
        v.addWidget(self._btn_esc)

        self._lbl_esc_estado = QLabel("Sin iniciar")
        self._lbl_esc_estado.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_esc_estado.setStyleSheet(f"font-size:10px; color:{_SEC};")
        v.addWidget(self._lbl_esc_estado)
        v.addWidget(_sep())

        # ── Curvas Teóricas P1 ─────────────────────────────────
        lbl_ct = QLabel("Curvas Teóricas · Prueba 1")
        lbl_ct.setStyleSheet("font-size:11px; font-weight:bold; color:#58A6FF;")
        v.addWidget(lbl_ct)
        self._btn_teo_n, self._lbl_teo_n = self._bloque_csv_ui(
            "n(t) teórico", self._importar_csv_teo_n, v)
        self._btn_teo_v, self._lbl_teo_v = self._bloque_csv_ui(
            "V(t) teórico", self._importar_csv_teo_v, v)
        v.addWidget(_sep())

        bv = self._btn_nav("←  Cambiar Prueba", _SEC, "#21262D", _BORDER, "#30363D")
        bv.clicked.connect(self._volver_menu); v.addWidget(bv)
        bf = self._btn_nav("⏹  Finalizar", "#FF7B72", "#3D1515", "#6A1C1C", "#5C1F1F")
        bf.clicked.connect(self._finalizar);  v.addWidget(bf)
        v.addStretch()
        return panel

    # ─────────────────────────────────────────────────────────────
    #  Panel Prueba 2: Variables · Fuente · Distancia · Captura
    # ─────────────────────────────────────────────────────────────
    def _hacer_panel_p2(self) -> QFrame:
        panel = QFrame(); panel.setObjectName("panel_p2")
        v = QVBoxLayout(panel); v.setSpacing(5); v.setContentsMargins(10, 10, 10, 10)

        self._seccion_vars(v, "_lbl_vars_p2")
        # P2 siempre inicia con la salida apagada.
        self._seccion_fuente(v, "_spin_volt_p2", "_btn_outp_p2", inicial_on=False)

        # ── Distancia imán–eje ────────────────────────────────
        lbl_d = QLabel("Distancia imán–eje")
        lbl_d.setStyleSheet(f"font-size:11px; font-weight:bold; color:{_YELLOW};")
        v.addWidget(lbl_d)

        self._spin_dist = QDoubleSpinBox()
        self._spin_dist.setRange(0.0, 50.0)
        self._spin_dist.setSingleStep(0.5)
        self._spin_dist.setDecimals(1)
        self._spin_dist.setSuffix(" mm")
        self._spin_dist.setValue(_p2_distancia_mm)
        self._spin_dist.setKeyboardTracking(False)
        self._spin_dist.setFixedHeight(28)
        self._spin_dist.setStyleSheet(self._sty_spin_ctrl())
        self._spin_dist.editingFinished.connect(self._guardar_distancia_p2)
        v.addWidget(self._spin_dist)
        v.addWidget(_sep())

        # ── Captura de punto ──────────────────────────────────
        lbl_cap = QLabel("Captura de Punto")
        lbl_cap.setStyleSheet(f"font-size:11px; font-weight:bold; color:{_GREEN_H};")
        v.addWidget(lbl_cap)

        sty_s = self._sty_spin_ctrl()
        self._spin_cap_dur = QDoubleSpinBox()
        self._spin_cap_dur.setRange(3.0, 120.0); self._spin_cap_dur.setValue(10.0)
        self._spin_cap_dur.setSingleStep(5.0); self._spin_cap_dur.setDecimals(0)
        self._spin_cap_dur.setSuffix(" s"); self._spin_cap_dur.setFixedHeight(24)
        self._spin_cap_dur.setStyleSheet(sty_s)
        row_dur = QHBoxLayout(); row_dur.setSpacing(4)
        lbl_dur = QLabel("Duración:"); lbl_dur.setStyleSheet(f"font-size:10px; color:{_SEC};")
        row_dur.addWidget(lbl_dur); row_dur.addWidget(self._spin_cap_dur)
        v.addLayout(row_dur)

        self._lbl_cap_contador = QLabel("Puntos: 0")
        self._lbl_cap_contador.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_cap_contador.setStyleSheet(
            f"font-size:12px; font-weight:bold; color:{_CYAN};")
        v.addWidget(self._lbl_cap_contador)

        self._btn_cap = QPushButton("📍  Capturar Punto")
        self._btn_cap.setFixedHeight(32)
        self._btn_cap.setStyleSheet(f"""
            QPushButton {{
                background-color:#1A4D2E; color:#39D353;
                border:1px solid #39D353; border-radius:5px;
                font-size:11px; font-weight:bold;
            }}
            QPushButton:hover {{ background-color:#245C38; }}
            QPushButton:disabled {{ background-color:#21262D; color:{_BORDER}; border-color:{_BORDER}; }}
        """)
        self._btn_cap.clicked.connect(self._capturar_punto_p2)
        v.addWidget(self._btn_cap)

        self._lbl_cap_prog = QLabel("Listo")
        self._lbl_cap_prog.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_cap_prog.setStyleSheet(f"font-size:10px; color:{_SEC};")
        v.addWidget(self._lbl_cap_prog)

        v.addWidget(_sep())

        b4 = self._btn_nav("📊  4 Características", _CYAN, "#0D419D", "#1F6FEB", "#1158C7")
        b4.clicked.connect(self._ir_caracteristicas); v.addWidget(b4)

        bc = self._btn_nav("🗑  Limpiar Gráficas", _SEC, "#21262D", _BORDER, "#30363D")
        bc.clicked.connect(self._limpiar_p2); v.addWidget(bc)

        bv = self._btn_nav("←  Cambiar Prueba", _SEC, "#21262D", _BORDER, "#30363D")
        bv.clicked.connect(self._volver_menu); v.addWidget(bv)
        bf = self._btn_nav("⏹  Finalizar", "#FF7B72", "#3D1515", "#6A1C1C", "#5C1F1F")
        bf.clicked.connect(self._finalizar);  v.addWidget(bf)
        v.addStretch()
        return panel

    # ─────────────────────────────────────────────────────────────
    #  Panel Prueba 2 extra: 4 curvas características
    # ─────────────────────────────────────────────────────────────
    def _hacer_panel_p2_carac(self) -> QFrame:
        panel = QFrame(); panel.setObjectName("panel_p2_carac")
        v = QVBoxLayout(panel); v.setSpacing(5); v.setContentsMargins(10, 10, 10, 10)

        self._seccion_vars(v, "_lbl_vars_p4")

        lbl_ct = QLabel("Curvas Teóricas · Prueba 2")
        lbl_ct.setStyleSheet("font-size:11px; font-weight:bold; color:#58A6FF;")
        v.addWidget(lbl_ct)
        self._btn_teo_p2_w,  self._lbl_teo_p2_w  = self._bloque_csv_ui(
            "n(TL) teórico",  self._importar_csv_p2_w,  v)
        self._btn_teo_p2_i,  self._lbl_teo_p2_i  = self._bloque_csv_ui(
            "I(TL) teórico",  self._importar_csv_p2_i,  v)
        self._btn_teo_p2_pm, self._lbl_teo_p2_pm = self._bloque_csv_ui(
            "Pm(TL) teórico", self._importar_csv_p2_pm, v)
        self._btn_teo_p2_n,  self._lbl_teo_p2_n  = self._bloque_csv_ui(
            "η(TL) teórico",  self._importar_csv_p2_n,  v)
        v.addWidget(_sep())

        self._lbl_cap_contador_c = QLabel("Puntos: 0")
        self._lbl_cap_contador_c.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_cap_contador_c.setStyleSheet(
            f"font-size:12px; font-weight:bold; color:{_CYAN};")
        v.addWidget(self._lbl_cap_contador_c)

        bb = self._btn_nav("←  Volver a captura", _SEC, "#21262D", _BORDER, "#30363D")
        bb.clicked.connect(self._volver_captura_p2); v.addWidget(bb)

        bc = self._btn_nav("🗑  Limpiar Gráficas", _SEC, "#21262D", _BORDER, "#30363D")
        bc.clicked.connect(self._limpiar_p2); v.addWidget(bc)

        bv = self._btn_nav("←  Cambiar Prueba", _SEC, "#21262D", _BORDER, "#30363D")
        bv.clicked.connect(self._volver_menu); v.addWidget(bv)
        bf = self._btn_nav("⏹  Finalizar", "#FF7B72", "#3D1515", "#6A1C1C", "#5C1F1F")
        bf.clicked.connect(self._finalizar);  v.addWidget(bf)
        v.addStretch()
        return panel

    def _ir_caracteristicas(self):
        """Pantalla extra P2: 4 gráficas características vs TL."""
        self._graf_stack.setCurrentIndex(2)
        self._panel_stack.setCurrentIndex(2)
        self._lbl_titulo.setText("Prueba 2 · Curvas características")
        self._actualizar_graficas_p2()
        if self._btn_outp_p2.isChecked():
            self._mostrar_teo_p2()

    def _volver_captura_p2(self):
        """Regresa a la pantalla de captura (2 gráficas)."""
        self._graf_stack.setCurrentIndex(1)
        self._panel_stack.setCurrentIndex(1)
        self._lbl_titulo.setText("Prueba 2 · Caracterización con Freno")

    # ── Captura Prueba 2 ──────────────────────────────────────
    def _guardar_distancia_p2(self) -> float:
        """Confirma el recuadro, guarda la distancia y la deja visible hasta que el usuario la cambie."""
        global _p2_distancia_mm
        self._spin_dist.interpretText()
        _p2_distancia_mm = float(self._spin_dist.value())
        self._spin_dist.setValue(_p2_distancia_mm)
        return _p2_distancia_mm

    def _capturar_punto_p2(self):
        """Inicia hilo de captura de 10 s para Prueba 2."""
        self._ensayo_activo = True        # activar actualización de labels
        self._btn_cap.setEnabled(False)
        dur = self._spin_cap_dur.value()
        dist = self._guardar_distancia_p2()
        self._lbl_cap_prog.setText(f"⏳  Iniciando {int(dur)} s…")
        threading.Thread(
            target=_run_captura_p2,
            args=(dur, dist),
            daemon=True,
        ).start()

    def _on_captura_p2_prog(self, texto: str):
        """Actualiza el label de progreso de captura."""
        self._lbl_cap_prog.setText(texto)

    def _on_captura_p2_lista(self):
        """Llamado cuando el hilo de captura termina — actualiza gráficos y habilita botón."""
        self._actualizar_graficas_p2()
        self._btn_cap.setEnabled(True)
        self._spin_dist.setValue(_p2_distancia_mm)
        with _lock:
            n_pts = len(_p2_puntos)
        self._lbl_cap_contador.setText(f"Puntos: {n_pts}")
        self._lbl_cap_contador_c.setText(f"Puntos: {n_pts}")

    def _actualizar_graficas_p2(self):
        """Redibuja las 2 gráficas de captura y las 4 características."""
        with _lock:
            puntos = list(_p2_puntos)
        if not puntos:
            return

        pts_d = sorted(puntos, key=lambda p: p.get("d", 0.0))
        self._p2_c_dist.setData(
            [p.get("d", 0.0) for p in pts_d],
            [p["TL"] for p in pts_d],
        )
        pts_teo = [p for p in pts_d if p.get("TL_teo") is not None]
        if pts_teo:
            self._p2_ct_dist.setData(
                [p["d"] for p in pts_teo],
                [p["TL_teo"] for p in pts_teo],
            )
        else:
            self._p2_ct_dist.setData([], [])

        pts_t = sorted(puntos, key=lambda p: p["TL"])
        TL  = [p["TL"]  for p in pts_t]
        n   = [p["n"]   for p in pts_t]
        I   = [p["I"]   for p in pts_t]
        Pm  = [p["Pm"]  for p in pts_t]
        eta = [p["eta"] for p in pts_t]

        self._p2_c_car_n.setData(TL, n)
        self._p2_c_car_i.setData(TL, I)
        self._p2_c_car_pm.setData(TL, Pm)
        self._p2_c_car_eta.setData(TL, eta)

        self._p2_c_w.setData(TL, n)
        self._p2_c_i.setData(TL, I)
        self._p2_c_pm.setData(TL, Pm)
        self._p2_c_n.setData(TL, eta)

    def _limpiar_p2(self):
        """Vacía puntos capturados y todas las curvas experimentales de Prueba 2."""
        _captura_p2_stop.set()
        with _lock:
            _p2_puntos.clear()
        _resetear_tiempo()
        for c in (
            self._p2_c_w, self._p2_c_i, self._p2_c_pm, self._p2_c_n,
            self._p2_c_car_n, self._p2_c_car_i, self._p2_c_car_pm, self._p2_c_car_eta,
            self._p2_c_dist, self._p2_ct_dist,
        ):
            c.setData([], [])
        self._lbl_cap_contador.setText("Puntos: 0")
        self._lbl_cap_contador_c.setText("Puntos: 0")
        self._lbl_cap_prog.setText("Listo")
        self._btn_cap.setEnabled(True)
        for pw in (self._p2_plot_dist, self._p2_plot_carac,
                   self._p2_plot_w, self._p2_plot_i, self._p2_plot_pm, self._p2_plot_n):
            lbl = getattr(pw, "_coord_label", None)
            mk  = getattr(pw, "_coord_marker", None)
            if lbl is not None:
                lbl.hide()
            if mk is not None:
                mk.setData([], [])

    def _habilitar_click_coords(self, pw: pg.PlotWidget):
        """Al hacer clic cerca de un punto, muestra sus coordenadas."""
        label = pg.TextItem(
            color=_PRI, anchor=(0.0, 1.15),
            fill=(13, 17, 23, 220), border=_CYAN,
        )
        label.setZValue(200)
        label.hide()
        pw.addItem(label)

        marker = pg.ScatterPlotItem(
            size=14, symbol="o",
            pen=pg.mkPen("#FFFFFF", width=2),
            brush=pg.mkBrush(0, 0, 0, 0),
        )
        marker.setZValue(199)
        marker._es_coord_ui = True  # type: ignore[attr-defined]
        pw.addItem(marker)
        pw._coord_label = label  # type: ignore[attr-defined]
        pw._coord_marker = marker  # type: ignore[attr-defined]

        def _on_click(ev):
            if ev.button() != Qt.MouseButton.LeftButton or ev.double():
                return
            scene_pos = ev.scenePos()
            if not pw.sceneBoundingRect().contains(scene_pos):
                return
            vb = pw.getViewBox()
            mejor = None  # (d2, x, y, nombre)
            radio2 = 20.0 * 20.0
            for item in pw.listDataItems():
                if getattr(item, "_es_coord_ui", False):
                    continue
                datos = item.getData()
                if not datos or datos[0] is None or len(datos[0]) == 0:
                    continue
                xs, ys = datos
                nombre = item.name() or ""
                for x, y in zip(xs, ys):
                    try:
                        xf, yf = float(x), float(y)
                    except (TypeError, ValueError):
                        continue
                    sp = vb.mapViewToScene(pg.Point(xf, yf))
                    dx, dy = sp.x() - scene_pos.x(), sp.y() - scene_pos.y()
                    d2 = dx * dx + dy * dy
                    if d2 < radio2 and (mejor is None or d2 < mejor[0]):
                        mejor = (d2, xf, yf, nombre)
            if mejor is None:
                label.hide()
                marker.setData([], [])
                return
            _, x, y, nombre = mejor
            xl = pw.getAxis("bottom").labelText or "X"
            yl = pw.getAxis("left").labelText or "Y"
            cab = f"{nombre}\n" if nombre else ""
            label.setText(f"{cab}{xl} = {x:.6g}\n{yl} = {y:.6g}")
            label.setPos(x, y)
            label.show()
            marker.setData([x], [y])

        pw.scene().sigMouseClicked.connect(_on_click)

    @staticmethod
    def _make_plot_dual():
        """Crea un PlotWidget con dos curvas (retorna widget, curva1, curva2)."""
        w = pg.PlotWidget()
        w.setBackground(_BG)
        w.showGrid(x=True, y=True, alpha=0.15)
        for ax in ("left", "bottom"):
            w.getAxis(ax).setPen(pg.mkPen(color=_BORDER))
            w.getAxis(ax).setTextPen(pg.mkPen(color=_SEC))
        c1 = w.plot(pen=pg.mkPen(color=_CYAN,  width=2.2))
        c2 = w.plot(pen=pg.mkPen(color=_RED_C, width=2.2))
        return w, c1, c2

    def _configurar_graficas(self, prueba: str):
        """Ajusta títulos y curvas según la prueba activa."""
        if prueba == "prueba1":
            # Gráfica top: n(t) [RPM]
            self._plot_top.setTitle("Velocidad (RPM) vs Tiempo",
                                    color="#C9D1D9", size="12pt")
            self._plot_top.setLabel("left",   "Velocidad (RPM)", color=_SEC)
            self._plot_top.setLabel("bottom", "Tiempo (s)",      color=_SEC)
            self._curva_top.setPen(pg.mkPen(color=_CYAN, width=2.2))
            self._curva_top2.setPen(pg.mkPen(color="transparent"))  # oculta
            # Gráfica bot: V(t)
            self._plot_bot.setTitle("Voltaje de Alimentación vs Tiempo",
                                    color="#C9D1D9", size="12pt")
            self._plot_bot.setLabel("left",   "Voltaje (V)",  color=_SEC)
            self._plot_bot.setLabel("bottom", "Tiempo (s)",   color=_SEC)
            self._curva_bot.setPen(pg.mkPen(color=_YELLOW, width=2.2))
            self._curva_bot2.setPen(pg.mkPen(color="transparent"))
        else:
            # Gráfica top: n(t) + TL(t) con dos ejes
            self._plot_top.setTitle("Velocidad Angular vs Tiempo",
                                    color="#C9D1D9", size="12pt")
            self._plot_top.setLabel("left",   "Velocidad (RPM)", color=_SEC)
            self._plot_top.setLabel("bottom", "Tiempo (s)",      color=_SEC)
            self._curva_top.setPen(pg.mkPen(color=_CYAN, width=2.2))
            self._curva_top2.setPen(pg.mkPen(color="transparent"))
            # Gráfica bot: TL(t) & η(t)
            self._plot_bot.setTitle("Torque de Carga y Eficiencia vs Tiempo",
                                    color="#C9D1D9", size="12pt")
            self._plot_bot.setLabel("left",   "Torque (N·m) / Ef. (%)", color=_SEC)
            self._plot_bot.setLabel("bottom", "Tiempo (s)",              color=_SEC)
            self._curva_bot.setPen(pg.mkPen(color=_RED_C,  width=2.2))
            self._curva_bot2.setPen(pg.mkPen(color=_PURPLE, width=1.6))

    # ── actualización en tiempo real ──────────────────────────
    def _tick(self):
        if not self._ensayo_activo:
            return
        with _lock:
            if not _t_buf:
                return
            t   = list(_t_buf)
            n   = list(_n_buf)
            V   = list(_V_buf)
            I   = list(_I_buf)
            FL  = list(_FL_buf)
            TL  = list(_TL_buf)
            Pe  = list(_Pe_buf)
            Pm  = list(_Pm_buf)
            eta = list(_eta_buf)
            w_r = list(_w_buf)

        # Actualizar curvas según prueba
        if self._prueba == "prueba1":
            self._curva_top.setData(t, n)
            self._curva_bot.setData(t, V)
        # Prueba 2: los gráficos solo se actualizan al capturar un punto

        # Panel de variables
        vals = {
            "n":  f"{n[-1]:.1f}",
            "ω":  f"{w_r[-1]:.2f}",
            "V":  f"{V[-1]:.3f}",
            "I":  f"{I[-1]:.4f}",
            "FL": f"{FL[-1]:.5f}",
            "TL": f"{TL[-1]:.6f}",
            "Pe": f"{Pe[-1]:.3f}",
            "Pm": f"{Pm[-1]:.5f}",
            "η":  f"{eta[-1]:.2f}",
        }
        for d in (self._lbl_vars_p1, self._lbl_vars_p2, self._lbl_vars_p4):
            for sym, lbl in d.items():
                lbl.setText(vals.get(sym, "—"))

    # ── exportación ───────────────────────────────────────────
    def _toggle_escalones(self, checked: bool):
        if checked:
            self._btn_esc.setText("⏹  Detener")
            self._btn_esc.setStyleSheet(self._btn_esc_style_det)

            v_ini = self._spin_v_ini.value()

            # 1. Limpiar curvas experimentales P1 (las teóricas se conservan)
            for c in (self._curva_top, self._curva_top2,
                      self._curva_bot, self._curva_bot2):
                c.setData([], [])

            # 2. Resetear tiempo y buffers → t=0 al inicio del ensayo
            _resetear_tiempo()

            # 3. Aplicar V_inicio de forma síncrona antes de arrancar el hilo
            _set_voltaje(v_ini)

            # 4. Habilitar graficación y mostrar teóricas
            self._ensayo_activo = True
            self._mostrar_teo_p1()

            # 5. Arrancar hilo de escalones (empezará desde v_ini)
            threading.Thread(
                target=_run_escalones,
                args=(v_ini,
                      self._spin_v_fin.value(),
                      self._spin_paso.value(),
                      self._spin_dur.value()),
                daemon=True,
            ).start()
        else:
            _escalon_stop.set()
            self._btn_esc.setText("▶  Iniciar Escalones")
            self._btn_esc.setStyleSheet(self._btn_esc_style_ini)

    def _on_escalon_progreso(self, texto: str):
        self._lbl_esc_estado.setText(texto)
        # Cuando termina, restablecer botón
        if texto.startswith("Completado") or texto.startswith("Detenido"):
            self._btn_esc.setChecked(False)
            self._btn_esc.setText("▶  Iniciar Escalones")
            self._btn_esc.setStyleSheet(self._btn_esc_style_ini)

    def _exportar(self):
        directorio = QFileDialog.getExistingDirectory(
            self, "Seleccionar carpeta de destino"
        )
        if not directorio:
            return

        threading.Thread(
            target=self._hacer_export,
            args=(directorio,),
            daemon=True,
        ).start()

    def _hacer_export(self, directorio: str):
        try:
            r1, r2 = _exportar_csv(directorio, _prueba_activa)
            QTimer.singleShot(
                0,
                lambda: QMessageBox.information(
                    self, "Exportación exitosa",
                    f"Datos guardados:\n\n{r1}\n\n{r2}",
                ),
            )
        except Exception as e:
            QTimer.singleShot(
                0,
                lambda: QMessageBox.critical(
                    self, "Error al exportar", str(e)
                ),
            )

    # ── Importar CSVs teóricos ────────────────────────────────
    @staticmethod
    def _leer_csv_xy(ruta: str) -> tuple[list[float], list[float]]:
        """
        Lee un CSV de dos columnas y acepta tanto ';' como ','.
        También ignora encabezados y tolera coma decimal cuando el
        separador del archivo es punto y coma.
        """
        x, y = [], []
        with open(ruta, "r", newline="", encoding="utf-8-sig") as f:
            sample = f.read(4096)
            f.seek(0)
            # Si existe ';' se prioriza explícitamente porque los CSV de P2
            # usan el formato: load torque;variable.
            delimiter = ";" if ";" in sample else ","

            reader = csv.reader(f, delimiter=delimiter)
            for row in reader:
                if len(row) < 2:
                    continue
                try:
                    sx = row[0].strip()
                    sy = row[1].strip()
                    if delimiter == ";":
                        sx = sx.replace(",", ".")
                        sy = sy.replace(",", ".")
                    x.append(float(sx))
                    y.append(float(sy))
                except (ValueError, TypeError):
                    # Encabezados o filas no numéricas.
                    continue

        if not x:
            raise ValueError(
                "El CSV no contiene dos columnas numéricas válidas. "
                "Formato esperado: eje_x;variable o eje_x,variable"
            )
        return x, y

    def _importar_csv_teo(self, tipo: str):
        """Lee un CSV (tiempo, valor) y guarda los datos; NO los grafica hasta iniciar ensayo."""
        titulo = "n(t) teórico — RPM" if tipo == "n" else "V(t) teórico — Voltaje"
        ruta, _ = QFileDialog.getOpenFileName(
            self, f"Importar {titulo}", "", "CSV (*.csv);;Todos (*)"
        )
        if not ruta:
            return
        try:
            t_teo, y_teo = self._leer_csv_xy(ruta)

            nombre = os.path.basename(ruta)
            _sty_ok = "font-size:9px; color:#7EE787; padding-left:12px;"
            if tipo == "n":
                self._teo_n_data = (t_teo, y_teo)   # guardado, no graficado aún
                self._lbl_teo_n.setText(f"✓ {nombre}")
                self._lbl_teo_n.setStyleSheet(_sty_ok)
                self._lbl_teo_n.setToolTip(nombre)
            else:
                self._teo_v_data = (t_teo, y_teo)
                self._lbl_teo_v.setText(f"✓ {nombre}")
                self._lbl_teo_v.setStyleSheet(_sty_ok)
                self._lbl_teo_v.setToolTip(nombre)

            señales.conexion_msg.emit(f"CSV teórico listo (se graficará al iniciar): {nombre}")
        except Exception as e:
            señales.conexion_msg.emit(f"Error al leer CSV teórico: {e}")

    def _importar_csv_teo_n(self):
        self._importar_csv_teo("n")

    def _importar_csv_teo_v(self):
        self._importar_csv_teo("V")

    def _mostrar_teo_p1(self):
        """Grafica las curvas teóricas de P1 si están cargadas."""
        if hasattr(self, "_teo_n_data") and self._teo_n_data:
            self._curva_top_teo.setData(*self._teo_n_data)
        if hasattr(self, "_teo_v_data") and self._teo_v_data:
            self._curva_bot_teo.setData(*self._teo_v_data)

    def _mostrar_teo_p2(self):
        """Grafica todas las curvas teóricas P2 que estén cargadas."""
        _map = [
            ("_teo_p2_w_data",  self._p2_ct_w,     self._p2_plot_w,     self._p2_ct_car_n),
            ("_teo_p2_i_data",  self._p2_ct_i,     self._p2_plot_i,     self._p2_ct_car_i),
            ("_teo_p2_pm_data", self._p2_ct_pm,    self._p2_plot_pm,    self._p2_ct_car_pm),
            ("_teo_p2_n_data",  self._p2_ct_n,     self._p2_plot_n,     self._p2_ct_car_eta),
        ]
        dibujadas = 0
        for attr, curva, plot, curva_comb in _map:
            if hasattr(self, attr) and getattr(self, attr):
                datos = getattr(self, attr)
                curva.setData(*datos)
                curva_comb.setData(*datos)
                plot.enableAutoRange()
                plot.autoRange()
                dibujadas += 1

        if dibujadas:
            señales.conexion_msg.emit(
                f"Prueba 2: {dibujadas} curva(s) teórica(s) mostrada(s)"
            )

    # ── Importar CSVs teóricos Prueba 2 (vs TL) ──────────────
    def _importar_csv_p2(self, data_attr: str, lbl_attr: str, titulo: str):
        """Guarda datos teóricos P2; se grafican cuando Salida cambia a ON."""
        ruta, _ = QFileDialog.getOpenFileName(
            self, f"Importar {titulo} teórico", "", "CSV (*.csv);;Todos (*)"
        )
        if not ruta:
            return
        try:
            x, y = self._leer_csv_xy(ruta)
            setattr(self, data_attr, (x, y))   # guardado, todavía oculto
            nombre = os.path.basename(ruta)
            lbl = getattr(self, lbl_attr)
            lbl.setText(f"✓ {nombre}")
            lbl.setStyleSheet("font-size:9px; color:#7EE787; padding-left:12px;")
            lbl.setToolTip(nombre)
            señales.conexion_msg.emit(
                f"CSV teórico listo ({len(x)} puntos; se graficará al poner Salida ON): {nombre}"
            )

            # Si por alguna razón el usuario carga un archivo con la salida
            # ya encendida, mostrarlo inmediatamente.
            if self._btn_outp_p2.isChecked():
                self._mostrar_teo_p2()
        except Exception as e:
            señales.conexion_msg.emit(f"Error CSV teórico: {e}")

    def _importar_csv_p2_w(self):
        self._importar_csv_p2("_teo_p2_w_data", "_lbl_teo_p2_w",  "n(TL)")

    def _importar_csv_p2_i(self):
        self._importar_csv_p2("_teo_p2_i_data", "_lbl_teo_p2_i",  "I(TL)")

    def _importar_csv_p2_pm(self):
        self._importar_csv_p2("_teo_p2_pm_data","_lbl_teo_p2_pm", "Pm(TL)")

    def _importar_csv_p2_n(self):
        self._importar_csv_p2("_teo_p2_n_data", "_lbl_teo_p2_n",  "η(TL)")

    def _volver_menu(self):
        """Detiene escalones activos y vuelve a la pantalla de selección de prueba."""
        _escalon_stop.set()
        _captura_p2_stop.set()
        self._ensayo_activo = False
        if self._volver_cb:
            self._volver_cb()

    def _finalizar(self):
        """Apaga la fuente, detiene todo y cierra la ventana."""
        _escalon_stop.set()
        _set_voltaje(0.0)
        _set_output(False)
        from PyQt6.QtWidgets import QApplication
        QApplication.quit()


# ============================================================
#  VENTANA PRINCIPAL
# ============================================================
class VentanaPrincipal(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dashboard — Freno Magnético Motor 57BLDC")
        self.resize(1380, 900)

        self._stack = QStackedWidget()
        self.setCentralWidget(self._stack)

        self._p0 = _PantallaBienvenida(lambda: self._ir(1))
        self._p1 = _PantallaConfig(lambda: self._ir(2))
        self._p2 = _PantallaPrueba(self._iniciar_ensayo)
        self._p3 = _PantallaGraficas(volver_cb=lambda: self._ir(2))

        for p in (self._p0, self._p1, self._p2, self._p3):
            self._stack.addWidget(p)

        self._stack.setCurrentIndex(0)

    def _ir(self, idx: int):
        self._stack.setCurrentIndex(idx)

    def _iniciar_ensayo(self, prueba: str):
        # Limpiar buffers para no mezclar datos de pruebas anteriores
        with _lock:
            for buf in (_t_buf, _n_buf, _w_buf, _FL_buf, _TL_buf,
                        _V_buf, _I_buf, _Pe_buf, _Pm_buf, _eta_buf):
                buf.clear()
            _registros.clear()
            _puntos_op.clear()

        self._p3.set_prueba(prueba)
        self._p3._configurar_graficas(prueba)
        self._ir(3)


# ============================================================
#  ENTRY POINT
# ============================================================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE_APP)
    win = VentanaPrincipal()
    win.show()
    if _fem_T:
        señales.conexion_msg.emit(
            f"FEM cargado · {len(_fem_T)} pts · "
            f"RPM {_fem_rpms[0]:.0f}–{_fem_rpms[-1]:.0f} · "
            f"gap {_fem_gaps[0]:.1f}–{_fem_gaps[-1]:.1f} mm"
        )
    else:
        señales.conexion_msg.emit(f"No se pudo cargar FEM: {_FEM_CSV}")
    sys.exit(app.exec())
