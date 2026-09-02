# ============================================================
#  Dashboard — Sistema de Monitoreo
#  Freno Magnético · Motor DC
#
#  Autores:
#    Juan Andrés Sanchez
#    Sofía Vega
#    Andrés Felipe Trujillo
#
#  Pantallas:
#    0 · Bienvenida
#    1 · Selección de transporte + Calibración
#    2 · Gráficas en tiempo real
# ============================================================

import json
import ssl
import sys
import threading
import time

import paho.mqtt.client as mqtt
import serial
from serial.tools import list_ports

from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QFrame, QStackedWidget,
)
import pyqtgraph as pg


# ============================================================
#  CREDENCIALES  —  HiveMQ Cloud
# ============================================================
MQTT_BROKER     = "0e44beba4fc7422cb74bc8bbdcc67b2f.s1.eu.hivemq.cloud"
MQTT_PORT       = 8883
MQTT_USER       = "EspMicroUno"
MQTT_PASSWORD   = "FrenoMotor2026!"
TOPIC_TELEMETRY = "micro1/motor1/telemetry"
TOPIC_CMD       = "micro1/motor1/cmd"

# ============================================================
#  CONFIGURACION SERIAL
# ============================================================
SERIAL_BAUD = 115200

# ============================================================
#  PARAMETROS GRAFICAS
# ============================================================
MAX_POINTS           = 300
RPM_CHANGE_THRESHOLD = 5.0

# ============================================================
#  ESTADO COMPARTIDO (hilo de datos → GUI)
# ============================================================
_lock              = threading.Lock()
_tiempos           = []
_rpm_datos         = []
_torque_datos      = []
_inicio_tiempo     = time.time()
_ultima_rpm        = None
_ultimo_torque     = 0.0

_mqtt_client: mqtt.Client | None  = None
_serial_port: serial.Serial | None = None


class _Señales(QObject):
    """Señales Qt para comunicar hilos background con la GUI."""
    conexion_msg = pyqtSignal(str)


señales = _Señales()


# ============================================================
#  PROCESAMIENTO DE DATOS  (lógica idéntica a mqtt_graficas.py)
# ============================================================
def _procesar(data: dict):
    global _ultima_rpm, _ultimo_torque

    rpm   = float(data.get("rpm_final",  0))
    torq  = float(data.get("torque_Nm", 0))
    t     = time.time() - _inicio_tiempo

    if _ultima_rpm is None:
        _ultimo_torque = torq
        _ultima_rpm    = rpm
    elif abs(rpm - _ultima_rpm) >= RPM_CHANGE_THRESHOLD:
        _ultimo_torque = torq
        _ultima_rpm    = rpm

    with _lock:
        _tiempos.append(t)
        _rpm_datos.append(rpm)
        _torque_datos.append(_ultimo_torque)
        if len(_tiempos) > MAX_POINTS:
            _tiempos.pop(0)
            _rpm_datos.pop(0)
            _torque_datos.pop(0)


# ============================================================
#  MQTT
# ============================================================
def _on_connect(client, userdata, flags, rc, props):
    if rc == 0:
        client.subscribe(TOPIC_TELEMETRY)
        señales.conexion_msg.emit(f"MQTT · {MQTT_BROKER}")
    else:
        señales.conexion_msg.emit(f"Error MQTT código {rc}")


def _on_message(client, userdata, msg):
    try:
        _procesar(json.loads(msg.payload.decode("utf-8")))
    except Exception:
        pass


def _iniciar_mqtt_loop():
    global _mqtt_client
    c = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        client_id="Dashboard_FrenoMotorDC",
    )
    c.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    c.tls_set(tls_version=ssl.PROTOCOL_TLS_CLIENT)
    c.tls_insecure_set(True)
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
    """Bucle infinito de lectura — corre en hilo background."""
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
#  PALETA DE COLORES
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
_RED     = "#FF6B35"

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
    padding: 12px 28px;
    font-size: 14px;
    font-weight: bold;
}}
QPushButton:hover   {{ background-color: {_BLUE_H}; }}
QPushButton:pressed {{ background-color: #1158C7; }}
QPushButton:disabled {{
    background-color: #21262D;
    color: {_SEC};
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
"""


def _sep() -> QFrame:
    f = QFrame()
    f.setObjectName("sep")
    f.setFrameShape(QFrame.Shape.HLine)
    return f


def _card(width: int = 560) -> QFrame:
    c = QFrame()
    c.setObjectName("card")
    c.setFixedWidth(width)
    return c


# ============================================================
#  PANTALLA 0 — BIENVENIDA
# ============================================================
class _PantallaBienvenida(QWidget):

    def __init__(self, ir_a_config):
        super().__init__()
        root = QVBoxLayout(self)
        root.setAlignment(Qt.AlignmentFlag.AlignCenter)
        root.setContentsMargins(0, 0, 0, 0)

        card = _card(560)
        v = QVBoxLayout(card)
        v.setSpacing(16)
        v.setContentsMargins(56, 52, 56, 52)

        # Ícono
        ico = QLabel("⚙")
        ico.setAlignment(Qt.AlignmentFlag.AlignCenter)
        ico.setStyleSheet(f"font-size: 68px; color: {_BLUE};")
        v.addWidget(ico)

        # Título
        t1 = QLabel("Sistema de Monitoreo")
        t1.setAlignment(Qt.AlignmentFlag.AlignCenter)
        t1.setStyleSheet(
            f"font-size: 30px; font-weight: bold; color: {_PRI};"
        )
        v.addWidget(t1)

        t2 = QLabel("Freno Magnético · Motor DC")
        t2.setAlignment(Qt.AlignmentFlag.AlignCenter)
        t2.setStyleSheet(
            f"font-size: 19px; font-weight: bold; color: {_BLUE};"
        )
        v.addWidget(t2)

        v.addWidget(_sep())

        for txt, style in [
            (
                "Proyecto de Diseño en Mecatrónica",
                f"font-size: 13px; color: {_SEC}; margin-bottom: 4px;",
            ),
            ("Desarrollado por:", f"font-size: 13px; color: {_SEC};"),
        ]:
            l = QLabel(txt)
            l.setAlignment(Qt.AlignmentFlag.AlignCenter)
            l.setStyleSheet(style)
            v.addWidget(l)

        for nombre in (
            "Juan Andrés Sanchez",
            "Sofía Vega",
            "Andrés Felipe Trujillo",
        ):
            l = QLabel(f"· {nombre}")
            l.setAlignment(Qt.AlignmentFlag.AlignCenter)
            l.setStyleSheet(f"font-size: 15px; color: {_PRI};")
            v.addWidget(l)

        v.addSpacing(8)
        v.addWidget(_sep())
        v.addSpacing(4)

        btn = QPushButton("Iniciar Dashboard")
        btn.setFixedHeight(48)
        btn.clicked.connect(ir_a_config)
        v.addWidget(btn)

        root.addWidget(card, alignment=Qt.AlignmentFlag.AlignCenter)


# ============================================================
#  PANTALLA 1 — SELECCIÓN DE TRANSPORTE + CALIBRACIÓN
# ============================================================
class _PantallaConfig(QWidget):

    def __init__(self, ir_a_graficas):
        super().__init__()
        self._ir_a_graficas = ir_a_graficas
        self._modo = "serial"

        root = QVBoxLayout(self)
        root.setAlignment(Qt.AlignmentFlag.AlignCenter)
        root.setContentsMargins(0, 0, 0, 0)

        card = _card(600)
        v = QVBoxLayout(card)
        v.setSpacing(18)
        v.setContentsMargins(52, 44, 52, 44)

        # ── Título
        tit = QLabel("Configuración del Sistema")
        tit.setAlignment(Qt.AlignmentFlag.AlignCenter)
        tit.setStyleSheet(
            f"font-size: 22px; font-weight: bold; color: {_PRI};"
        )
        v.addWidget(tit)
        v.addWidget(_sep())

        # ── Selección de transporte
        lbl = QLabel("Seleccione el medio de comunicación")
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl.setStyleSheet(f"font-size: 13px; color: {_SEC};")
        v.addWidget(lbl)

        row = QHBoxLayout()
        row.setSpacing(14)

        self._btn_serial = QPushButton("🔌   USB Serial")
        self._btn_serial.setFixedHeight(48)
        self._btn_serial.setCheckable(True)
        self._btn_serial.setChecked(True)
        self._btn_serial.clicked.connect(lambda: self._set_modo("serial"))

        self._btn_mqtt = QPushButton("🌐   MQTT · HiveMQ")
        self._btn_mqtt.setFixedHeight(48)
        self._btn_mqtt.setCheckable(True)
        self._btn_mqtt.clicked.connect(lambda: self._set_modo("mqtt"))

        self._actualizar_estilo_botones()

        row.addWidget(self._btn_serial)
        row.addWidget(self._btn_mqtt)
        v.addLayout(row)

        # ── Descripción
        self._lbl_desc = QLabel(
            "La ESP32 se conecta directamente al computador por USB."
        )
        self._lbl_desc.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_desc.setWordWrap(True)
        self._lbl_desc.setStyleSheet(f"font-size: 13px; color: {_SEC};")
        v.addWidget(self._lbl_desc)

        v.addWidget(_sep())

        # ── Estado
        self._lbl_estado = QLabel("Esperando calibración del sistema…")
        self._lbl_estado.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_estado.setStyleSheet(f"font-size: 13px; color: {_ORANGE};")
        v.addWidget(self._lbl_estado)

        # ── Botón calibrar
        self._btn_cal = QPushButton("⚙   Calibrar Sistema")
        self._btn_cal.setFixedHeight(52)
        self._btn_cal.setStyleSheet(f"""
            QPushButton {{
                background-color: {_GREEN};
                color: white;
                border: none;
                border-radius: 8px;
                font-size: 15px;
                font-weight: bold;
            }}
            QPushButton:hover   {{ background-color: {_GREEN_H}; }}
            QPushButton:pressed {{ background-color: #1C6B2A; }}
            QPushButton:disabled {{
                background-color: #1C6B2A;
                color: #7EE787;
            }}
        """)
        self._btn_cal.clicked.connect(self._calibrar)
        v.addWidget(self._btn_cal)

        # ── Botón ir a gráficas
        self._btn_go = QPushButton("Ver Gráficas  →")
        self._btn_go.setFixedHeight(46)
        self._btn_go.setEnabled(False)
        self._btn_go.clicked.connect(ir_a_graficas)
        v.addWidget(self._btn_go)

        root.addWidget(card, alignment=Qt.AlignmentFlag.AlignCenter)

        señales.conexion_msg.connect(self._on_conexion)

    # ── helpers
    def _set_modo(self, modo: str):
        self._modo = modo
        self._btn_serial.setChecked(modo == "serial")
        self._btn_mqtt.setChecked(modo == "mqtt")
        self._actualizar_estilo_botones()
        self._lbl_desc.setText(
            "La ESP32 se conecta directamente al computador por USB."
            if modo == "serial"
            else "La ESP32 se comunica vía WiFi con el broker HiveMQ Cloud."
        )

    def _actualizar_estilo_botones(self):
        activo   = f"background-color: #0D419D; border: 2px solid {_BLUE};"
        inactivo = f"background-color: #21262D; color: {_SEC};"
        for btn, nombre in [
            (self._btn_serial, "serial"),
            (self._btn_mqtt,   "mqtt"),
        ]:
            s = activo if self._modo == nombre else inactivo
            btn.setStyleSheet(f"""
                QPushButton {{
                    color: {_PRI};
                    border-radius: 8px;
                    padding: 10px 20px;
                    font-size: 14px;
                    font-weight: bold;
                    {s}
                }}
                QPushButton:hover {{ background-color: #1F3A7A; border: 2px solid {_BLUE_H}; color: white; }}
            """)

    def _on_conexion(self, msg: str):
        self._lbl_estado.setText(msg)

    # ── calibración
    def _calibrar(self):
        self._btn_cal.setEnabled(False)
        self._btn_cal.setText("Calibrando…")
        self._lbl_estado.setStyleSheet(f"font-size: 13px; color: {_ORANGE};")
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

            # Esperar menú de tara (hasta 20 s) y enviar opción 1
            deadline = time.time() + 20
            enviado = False
            while time.time() < deadline:
                linea = ser.readline().decode("utf-8", errors="ignore").strip()
                if "1 = Tara automatica" in linea:
                    ser.write(b"1\n")
                    enviado = True
                    break
            if not enviado:
                ser.write(b"1\n")

            _inicio_tiempo = time.time()
            QTimer.singleShot(0, self._cal_ok)

            # Continúa leyendo datos en este mismo hilo
            _leer_serial_loop(ser)

        except Exception as e:
            señales.conexion_msg.emit(f"Error serial: {e}")

    def _cal_mqtt(self):
        global _inicio_tiempo
        threading.Thread(target=_iniciar_mqtt_loop, daemon=True).start()

        # Esperar conexión hasta 20 s
        deadline = time.time() + 20
        while time.time() < deadline:
            if _mqtt_client and _mqtt_client.is_connected():
                break
            time.sleep(0.4)

        _inicio_tiempo = time.time()
        _publicar_cmd("calibrate")
        señales.conexion_msg.emit("Comando de calibración enviado por MQTT")
        QTimer.singleShot(0, self._cal_ok)

    def _cal_ok(self):
        """Ejecuta en el hilo Qt — actualiza widgets."""
        self._btn_cal.setText("✓   Sistema Calibrado")
        self._btn_cal.setEnabled(False)
        self._lbl_estado.setStyleSheet("font-size: 13px; color: #7EE787;")
        self._lbl_estado.setText("Sistema listo — puede iniciar la medición.")
        self._btn_go.setEnabled(True)


# ============================================================
#  PANTALLA 2 — GRÁFICAS EN TIEMPO REAL
# ============================================================
class _PantallaGraficas(QWidget):

    def __init__(self):
        super().__init__()
        root = QVBoxLayout(self)
        root.setSpacing(10)
        root.setContentsMargins(24, 14, 24, 14)

        # ── Header
        hdr = QHBoxLayout()
        lbl_titulo = QLabel("Dashboard · Freno Magnético Motor DC")
        lbl_titulo.setStyleSheet(
            f"font-size: 16px; font-weight: bold; color: {_PRI};"
        )
        hdr.addWidget(lbl_titulo)
        hdr.addStretch()
        self._lbl_conn = QLabel("—")
        self._lbl_conn.setStyleSheet(f"font-size: 12px; color: {_SEC};")
        hdr.addWidget(self._lbl_conn)
        root.addLayout(hdr)
        root.addWidget(_sep())

        # ── Valores actuales (grande)
        vals = QHBoxLayout()
        vals.setSpacing(0)

        self._lbl_rpm_val = QLabel("—   RPM")
        self._lbl_rpm_val.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_rpm_val.setStyleSheet(
            f"font-size: 32px; font-weight: bold; color: {_CYAN};"
            " padding: 6px 0;"
        )
        vals.addWidget(self._lbl_rpm_val, stretch=1)

        div = QFrame()
        div.setFrameShape(QFrame.Shape.VLine)
        div.setStyleSheet(f"color: {_BORDER};")
        vals.addWidget(div)

        self._lbl_torq_val = QLabel("—   N·m")
        self._lbl_torq_val.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._lbl_torq_val.setStyleSheet(
            f"font-size: 32px; font-weight: bold; color: {_RED};"
            " padding: 6px 0;"
        )
        vals.addWidget(self._lbl_torq_val, stretch=1)
        root.addLayout(vals)
        root.addWidget(_sep())

        # ── Gráfica RPM
        self._plot_rpm = self._make_plot(
            "Velocidad Angular vs Tiempo",
            "Velocidad (RPM)",
            "Tiempo (s)",
        )
        self._curva_rpm = self._plot_rpm.plot(
            pen=pg.mkPen(color=_CYAN, width=2.5),
            fillLevel=0,
            brush=pg.mkBrush(0, 212, 255, 28),
        )
        root.addWidget(self._plot_rpm, stretch=1)

        # ── Gráfica Torque
        self._plot_torq = self._make_plot(
            "Torque de Carga vs Tiempo",
            "Torque (N·m)",
            "Tiempo (s)",
        )
        self._curva_torq = self._plot_torq.plot(
            pen=pg.mkPen(color=_RED, width=2.5),
            fillLevel=0,
            brush=pg.mkBrush(255, 107, 53, 28),
        )
        root.addWidget(self._plot_torq, stretch=1)

        # ── Timer refresco
        tmr = QTimer(self)
        tmr.timeout.connect(self._tick)
        tmr.start(100)

        señales.conexion_msg.connect(
            lambda m: self._lbl_conn.setText(m)
        )

    # ── helper para crear gráficas con el mismo estilo
    @staticmethod
    def _make_plot(titulo: str, eje_y: str, eje_x: str) -> pg.PlotWidget:
        w = pg.PlotWidget()
        w.setBackground(_BG)
        w.setTitle(titulo, color="#C9D1D9", size="13pt")
        w.setLabel("left",   eje_y, color=_SEC, **{"font-size": "11pt"})
        w.setLabel("bottom", eje_x, color=_SEC, **{"font-size": "11pt"})
        w.showGrid(x=True, y=True, alpha=0.18)
        for ax in ("left", "bottom"):
            w.getAxis(ax).setPen(pg.mkPen(color=_BORDER))
            w.getAxis(ax).setTextPen(pg.mkPen(color=_SEC))
        return w

    def _tick(self):
        with _lock:
            if not _tiempos:
                return
            x   = list(_tiempos)
            rpm = list(_rpm_datos)
            trq = list(_torque_datos)

        self._curva_rpm.setData(x, rpm)
        self._lbl_rpm_val.setText(f"{rpm[-1]:.1f}   RPM")

        self._curva_torq.setData(x, trq)
        self._lbl_torq_val.setText(f"{trq[-1]:.6f}   N·m")


# ============================================================
#  VENTANA PRINCIPAL
# ============================================================
class VentanaPrincipal(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dashboard — Freno Magnético Motor DC")
        self.resize(1280, 860)

        self._stack = QStackedWidget()
        self.setCentralWidget(self._stack)

        self._p0 = _PantallaBienvenida(self._a1)
        self._p1 = _PantallaConfig(self._a2)
        self._p2 = _PantallaGraficas()

        for p in (self._p0, self._p1, self._p2):
            self._stack.addWidget(p)

        self._stack.setCurrentIndex(0)

    def _a1(self): self._stack.setCurrentIndex(1)
    def _a2(self): self._stack.setCurrentIndex(2)


# ============================================================
#  ENTRY POINT
# ============================================================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE_APP)
    win = VentanaPrincipal()
    win.show()
    sys.exit(app.exec())
