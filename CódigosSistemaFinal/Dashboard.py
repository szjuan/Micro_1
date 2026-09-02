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
#    3 · Gráficas en tiempo real + exportación
# ============================================================

import csv
import json
import math
import os
import ssl
import sys
import threading
import time
from datetime import datetime

import paho.mqtt.client as mqtt
import serial
from serial.tools import list_ports

from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QLabel,
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
VISA_ADDRESS    = "USB0::0x2A8D::0x3302::MY61004672::0::INSTR"
FUENTE_INTERVAL = 0.5   # segundos entre lecturas

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

# Prueba de escalones de voltaje
_escalon_stop = threading.Event()   # set() para detener la secuencia

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
    conexion_msg     = pyqtSignal(str)
    datos_nuevos     = pyqtSignal()
    punto_capturado  = pyqtSignal(int)   # emite el número de punto
    escalon_progreso = pyqtSignal(str)   # texto de estado del escalón


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
    global _V_fuente, _I_fuente, _fuente_inst
    try:
        import pyvisa  # type: ignore
        rm   = pyvisa.ResourceManager()
        inst = rm.open_resource(VISA_ADDRESS)
        inst.timeout           = 5000
        inst.write_termination = "\n"
        inst.read_termination  = "\n"

        # Seleccionar CH1, fijar voltaje a 0 y encender salida
        inst.write("INST:SEL CH1")
        inst.write("VOLT 0")
        inst.write("OUTP ON")

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


def _set_voltaje(v: float):
    """Envía VOLT <v> al CH1. Seguro para llamar desde la GUI."""
    if _fuente_inst is None:
        return
    try:
        _fuente_inst.write(f"VOLT {v:.3f}")
    except Exception:
        pass


def _set_output(on: bool):
    """Enciende o apaga la salida CH1."""
    if _fuente_inst is None:
        return
    try:
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
            time.sleep(2.0)
            deadline = time.time() + 20; enviado = False
            while time.time() < deadline:
                linea = ser.readline().decode("utf-8", errors="ignore").strip()
                if "1 = Tara automatica" in linea:
                    ser.write(b"1\n"); enviado = True; break
            if not enviado:
                ser.write(b"1\n")
            _inicio_tiempo = time.time()
            QTimer.singleShot(0, self._cal_ok)
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

    def __init__(self):
        super().__init__()
        self._prueba = "prueba1"
        self._construir()

    def set_prueba(self, prueba: str):
        self._prueba = prueba
        nombres = {
            "prueba1": "Prueba 1 · Velocidad vs Voltaje",
            "prueba2": "Prueba 2 · Caracterización con Freno",
        }
        self._lbl_titulo.setText(nombres.get(prueba, "Ensayo"))

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

        # Panel izquierdo — variables + controles
        self._panel_vars = self._hacer_panel_vars()
        body.addWidget(self._panel_vars, stretch=0)

        # Panel derecho — gráficas
        graficas_col = QVBoxLayout(); graficas_col.setSpacing(8)
        self._plot_top, self._curva_top, self._curva_top2 = self._make_plot_dual()
        self._plot_bot, self._curva_bot, self._curva_bot2 = self._make_plot_dual()
        graficas_col.addWidget(self._plot_top, stretch=1)
        graficas_col.addWidget(self._plot_bot, stretch=1)
        body.addLayout(graficas_col, stretch=1)

        root.addLayout(body, stretch=1)

        # ── Timer
        tmr = QTimer(self); tmr.timeout.connect(self._tick); tmr.start(100)
        señales.conexion_msg.connect(lambda m: self._lbl_conn.setText(m))
        señales.punto_capturado.connect(
            lambda n: self._lbl_puntos.setText(f"Puntos capturados: {n}")
        )
        señales.escalon_progreso.connect(self._on_escalon_progreso)

    def _hacer_panel_vars(self) -> QFrame:
        panel = QFrame(); panel.setObjectName("panel_vars")
        panel.setFixedWidth(230)
        v = QVBoxLayout(panel); v.setSpacing(6); v.setContentsMargins(14, 14, 14, 14)

        lbl_h = QLabel("Variables actuales")
        lbl_h.setStyleSheet(f"font-size:12px; font-weight:bold; color:{_SEC};")
        v.addWidget(lbl_h); v.addWidget(_sep())

        # Definición de variables con color y unidad
        self._vars_def = [
            ("n",   "RPM Final",    "RPM",   _CYAN),
            ("ω",   "Vel. angular", "rad/s", _CYAN),
            ("V",   "Voltaje",      "V",     _YELLOW),
            ("I",   "Corriente",    "A",     _YELLOW),
            ("FL",  "F. galga",     "N",     _GREEN_H),
            ("TL",  "Torque",       "N·m",   _RED_C),
            ("Pe",  "Pot. eléct.",  "W",     _ORANGE),
            ("Pm",  "Pot. mec.",    "W",     _PURPLE),
            ("η",   "Eficiencia",   "%",     "#58A6FF"),
        ]
        self._lbl_vars: dict[str, QLabel] = {}
        for sym, nombre, unidad, color in self._vars_def:
            row = QHBoxLayout(); row.setSpacing(4)
            l_sym = QLabel(sym)
            l_sym.setFixedWidth(26)
            l_sym.setStyleSheet(f"font-size:13px; font-weight:bold; color:{color};")
            l_nom = QLabel(nombre)
            l_nom.setStyleSheet(f"font-size:10px; color:{_SEC};")
            l_nom.setFixedWidth(76)
            l_val = QLabel("—")
            l_val.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            l_val.setStyleSheet(f"font-size:12px; font-weight:bold; color:{_PRI};")
            l_uni = QLabel(unidad)
            l_uni.setStyleSheet(f"font-size:10px; color:{_SEC};")
            l_uni.setFixedWidth(34)
            row.addWidget(l_sym); row.addWidget(l_nom)
            row.addWidget(l_val); row.addWidget(l_uni)
            v.addLayout(row)
            self._lbl_vars[sym] = l_val

        v.addWidget(_sep())

        # ── Control Fuente Keysight ─────────────────────────────
        lbl_fuente = QLabel("Control Fuente · CH1")
        lbl_fuente.setStyleSheet(f"font-size:11px; font-weight:bold; color:{_YELLOW};")
        v.addWidget(lbl_fuente)

        # Fila: spinbox voltaje + botón Aplicar
        row_v = QHBoxLayout(); row_v.setSpacing(6)
        self._spin_volt = QDoubleSpinBox()
        self._spin_volt.setRange(0.0, 30.0)
        self._spin_volt.setSingleStep(0.5)
        self._spin_volt.setDecimals(1)
        self._spin_volt.setSuffix(" V")
        self._spin_volt.setValue(0.0)
        self._spin_volt.setFixedHeight(32)
        self._spin_volt.setStyleSheet(f"""
            QDoubleSpinBox {{
                background:#21262D; color:{_PRI};
                border:1px solid {_BORDER}; border-radius:5px;
                font-size:13px; padding:2px 6px;
            }}
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{
                width:18px; background:#30363D; border:none;
            }}
        """)
        btn_aplicar = QPushButton("Aplicar")
        btn_aplicar.setFixedHeight(32)
        btn_aplicar.setStyleSheet(f"""
            QPushButton {{
                background-color:{_YELLOW}; color:#0D1117;
                border:none; border-radius:5px;
                font-size:11px; font-weight:bold;
            }}
            QPushButton:hover {{ background-color:#E3C000; }}
        """)
        btn_aplicar.clicked.connect(
            lambda: _set_voltaje(self._spin_volt.value())
        )
        row_v.addWidget(self._spin_volt, stretch=1)
        row_v.addWidget(btn_aplicar)
        v.addLayout(row_v)

        # Botón ON/OFF salida
        self._btn_outp = QPushButton("⚡  Salida ON")
        self._btn_outp.setCheckable(True)
        self._btn_outp.setChecked(True)
        self._btn_outp.setFixedHeight(32)
        self._btn_outp_style_on  = f"""
            QPushButton {{
                background-color:#1A4D2E; color:#39D353;
                border:1px solid #39D353; border-radius:5px;
                font-size:11px; font-weight:bold;
            }}
            QPushButton:hover {{ background-color:#245C38; }}
        """
        self._btn_outp_style_off = f"""
            QPushButton {{
                background-color:#3D1515; color:#FF7B72;
                border:1px solid #FF7B72; border-radius:5px;
                font-size:11px; font-weight:bold;
            }}
            QPushButton:hover {{ background-color:#5C1F1F; }}
        """
        self._btn_outp.setStyleSheet(self._btn_outp_style_on)
        self._btn_outp.toggled.connect(self._toggle_output)
        v.addWidget(self._btn_outp)

        v.addWidget(_sep())

        # ── Prueba de Escalones de Voltaje ─────────────────────
        lbl_esc = QLabel("Prueba de Escalones")
        lbl_esc.setStyleSheet(f"font-size:11px; font-weight:bold; color:{_CYAN};")
        v.addWidget(lbl_esc)

        def _spin_esc(lo, hi, val, step, dec, suf):
            s = QDoubleSpinBox()
            s.setRange(lo, hi); s.setValue(val)
            s.setSingleStep(step); s.setDecimals(dec)
            s.setSuffix(suf); s.setFixedHeight(28)
            s.setStyleSheet(f"""
                QDoubleSpinBox {{
                    background:#21262D; color:{_PRI};
                    border:1px solid {_BORDER}; border-radius:4px;
                    font-size:12px; padding:1px 4px;
                }}
                QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{
                    width:16px; background:#30363D; border:none;
                }}
            """)
            return s

        def _row_param(label, widget):
            r = QHBoxLayout(); r.setSpacing(4)
            l = QLabel(label)
            l.setStyleSheet(f"font-size:10px; color:{_SEC};")
            l.setFixedWidth(58)
            r.addWidget(l); r.addWidget(widget)
            return r

        self._spin_v_ini  = _spin_esc(0.0, 30.0, 6.0,  0.5, 1, " V")
        self._spin_v_fin  = _spin_esc(0.0, 30.0, 24.0, 0.5, 1, " V")
        self._spin_paso   = _spin_esc(0.5, 10.0, 2.0,  0.5, 1, " V")
        self._spin_dur    = _spin_esc(5.0, 300.0, 15.0, 5.0, 0, " s")

        v.addLayout(_row_param("V inicio:", self._spin_v_ini))
        v.addLayout(_row_param("V final:",  self._spin_v_fin))
        v.addLayout(_row_param("Paso V:",   self._spin_paso))
        v.addLayout(_row_param("Duración:", self._spin_dur))

        # Botón Iniciar / Detener
        self._btn_esc = QPushButton("▶  Iniciar Escalones")
        self._btn_esc.setCheckable(True)
        self._btn_esc.setFixedHeight(34)
        self._btn_esc_style_ini = f"""
            QPushButton {{
                background-color:#0D419D; color:white;
                border:none; border-radius:5px;
                font-size:11px; font-weight:bold;
            }}
            QPushButton:hover {{ background-color:#1158C7; }}
        """
        self._btn_esc_style_det = f"""
            QPushButton {{
                background-color:#3D1515; color:#FF7B72;
                border:1px solid #FF7B72; border-radius:5px;
                font-size:11px; font-weight:bold;
            }}
            QPushButton:hover {{ background-color:#5C1F1F; }}
        """
        self._btn_esc.setStyleSheet(self._btn_esc_style_ini)
        self._btn_esc.toggled.connect(self._toggle_escalones)
        v.addWidget(self._btn_esc)

        # Label de estado del escalón
        self._lbl_esc_estado = QLabel("Sin iniciar")
        self._lbl_esc_estado.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_esc_estado.setWordWrap(True)
        self._lbl_esc_estado.setStyleSheet(
            f"font-size:10px; color:{_SEC}; padding:2px;"
        )
        v.addWidget(self._lbl_esc_estado)

        v.addWidget(_sep())

        # Capturar punto
        btn_cap = QPushButton("📌  Capturar Punto")
        btn_cap.setFixedHeight(40)
        btn_cap.setStyleSheet(f"""
            QPushButton {{
                background-color:{_GREEN}; color:white; border:none;
                border-radius:6px; font-size:12px; font-weight:bold;
            }}
            QPushButton:hover {{ background-color:{_GREEN_H}; }}
        """)
        btn_cap.clicked.connect(
            lambda: threading.Thread(
                target=_capturar_punto_estable, daemon=True
            ).start()
        )
        v.addWidget(btn_cap)

        self._lbl_puntos = QLabel("Puntos capturados: 0")
        self._lbl_puntos.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_puntos.setStyleSheet(f"font-size:11px; color:{_SEC};")
        v.addWidget(self._lbl_puntos)

        v.addWidget(_sep())

        # Exportar CSV
        btn_exp = QPushButton("💾  Exportar CSV")
        btn_exp.setFixedHeight(38)
        btn_exp.setStyleSheet(f"""
            QPushButton {{
                background-color:#21262D; color:{_PRI}; border:1px solid {_BORDER};
                border-radius:6px; font-size:12px; font-weight:bold;
            }}
            QPushButton:hover {{ background-color:#30363D; }}
        """)
        btn_exp.clicked.connect(self._exportar)
        v.addWidget(btn_exp)

        # Finalizar ensayo
        btn_fin = QPushButton("⏹  Finalizar")
        btn_fin.setFixedHeight(38)
        btn_fin.setStyleSheet(f"""
            QPushButton {{
                background-color:#3D1515; color:#FF7B72; border:1px solid #6A1C1C;
                border-radius:6px; font-size:12px; font-weight:bold;
            }}
            QPushButton:hover {{ background-color:#5C1F1F; }}
        """)
        btn_fin.clicked.connect(self._finalizar)
        v.addWidget(btn_fin)

        v.addStretch()
        return panel

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
            self._plot_top.setTitle("Velocidad Angular vs Tiempo",
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
        self._curva_top.setData(t, n)
        if self._prueba == "prueba1":
            self._curva_bot.setData(t, V)
        else:
            self._curva_bot.setData(t, TL)
            self._curva_bot2.setData(t, eta)

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
        for sym, lbl in self._lbl_vars.items():
            lbl.setText(vals.get(sym, "—"))

    # ── exportación ───────────────────────────────────────────
    def _toggle_output(self, checked: bool):
        """Enciende/apaga salida CH1 y actualiza estilo del botón."""
        _set_output(checked)
        if checked:
            self._btn_outp.setText("⚡  Salida ON")
            self._btn_outp.setStyleSheet(self._btn_outp_style_on)
        else:
            self._btn_outp.setText("○  Salida OFF")
            self._btn_outp.setStyleSheet(self._btn_outp_style_off)

    def _toggle_escalones(self, checked: bool):
        if checked:
            self._btn_esc.setText("⏹  Detener")
            self._btn_esc.setStyleSheet(self._btn_esc_style_det)
            threading.Thread(
                target=_run_escalones,
                args=(
                    self._spin_v_ini.value(),
                    self._spin_v_fin.value(),
                    self._spin_paso.value(),
                    self._spin_dur.value(),
                ),
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

    def _finalizar(self):
        resp = QMessageBox.question(
            self,
            "Finalizar ensayo",
            "¿Desea exportar los datos antes de finalizar?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            | QMessageBox.StandardButton.Cancel,
        )
        if resp == QMessageBox.StandardButton.Cancel:
            return
        if resp == QMessageBox.StandardButton.Yes:
            self._exportar()


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
        self._p3 = _PantallaGraficas()

        for p in (self._p0, self._p1, self._p2, self._p3):
            self._stack.addWidget(p)

        self._stack.setCurrentIndex(0)

    def _ir(self, idx: int):
        self._stack.setCurrentIndex(idx)

    def _iniciar_ensayo(self, prueba: str):
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
    sys.exit(app.exec())
