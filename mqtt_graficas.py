import os

# ============================================================
# SOLUCION PyQt6 / COCOA EN TU INSTALACION ANACONDA
# Debe ir ANTES de importar PyQt6.
# ============================================================

PYQT_PLUGIN_PATH = (
    "/opt/anaconda3/lib/python3.12/"
    "site-packages/PyQt6/Qt6/plugins/platforms"
)

if os.path.isdir(PYQT_PLUGIN_PATH):
    os.environ.setdefault(
        "QT_QPA_PLATFORM_PLUGIN_PATH",
        PYQT_PLUGIN_PATH
    )


# ============================================================
# IMPORTS
# ============================================================

import argparse
import json
import sys
import threading
import time

import paho.mqtt.client as mqtt
import serial

from serial.tools import list_ports

from PyQt6.QtCore import QTimer

from PyQt6.QtWidgets import (
    QApplication,
    QLabel,
    QVBoxLayout,
    QWidget
)

import pyqtgraph as pg


# ============================================================
# CONFIGURACION MQTT
# ============================================================

BROKER = "172.20.10.2"

MQTT_PORT = 1883

TOPIC = "micro1/motor1/telemetry"


# ============================================================
# CONFIGURACION USB SERIAL
# ============================================================

SERIAL_BAUD = 115200


# ============================================================
# CONFIGURACION TARA POR USB
# ============================================================
#
# AUTO:
# Python envia "1" automaticamente cuando la ESP32
# muestra el menu de tara.
#
# MANUAL:
# Python selecciona opcion 2 y envia MANUAL_ZERO_RAW.
#
# ============================================================

SERIAL_TARE_MODE = "AUTO"

MANUAL_ZERO_RAW = -150000.0


# ============================================================
# ACTUALIZACION DEL TORQUE
# ============================================================
#
# El torque graficado solo cambia cuando RPM FINAL cambia
# al menos esta cantidad.
#
# ============================================================

RPM_CHANGE_THRESHOLD = 5.0


# ============================================================
# HISTORIAL DE LAS GRAFICAS
# ============================================================

MAX_POINTS = 300


# ============================================================
# VARIABLES COMPARTIDAS
# ============================================================

data_lock = threading.Lock()

tiempos = []

rpm_datos = []

torque_datos = []


inicio = time.time()


ultima_rpm_torque = None

ultimo_torque_valido = 0.0


estado_conexion = "Desconectado"


# ============================================================
# PROCESAR DATOS
#
# Esta funcion sirve tanto para MQTT como USB.
# ============================================================

def procesar_datos(data):

    global ultima_rpm_torque
    global ultimo_torque_valido


    # --------------------------------------------------------
    # VARIABLES PRINCIPALES
    # --------------------------------------------------------

    rpm_final = float(
        data["rpm_final"]
    )

    torque_medido = float(
        data["torque_Nm"]
    )


    # --------------------------------------------------------
    # TIEMPO
    # --------------------------------------------------------

    t = (
        time.time()
        -
        inicio
    )


    # ========================================================
    # ACTUALIZAR TORQUE SOLO SI CAMBIO RPM FINAL
    # ========================================================

    if ultima_rpm_torque is None:

        ultimo_torque_valido = (
            torque_medido
        )

        ultima_rpm_torque = (
            rpm_final
        )


    else:

        diferencia_rpm = abs(
            rpm_final
            -
            ultima_rpm_torque
        )


        if (
            diferencia_rpm
            >=
            RPM_CHANGE_THRESHOLD
        ):

            ultimo_torque_valido = (
                torque_medido
            )

            ultima_rpm_torque = (
                rpm_final
            )


            print(
                f"NUEVO PUNTO | "
                f"RPM = {rpm_final:.1f} | "
                f"Torque = "
                f"{torque_medido:.6f} N*m"
            )


    # ========================================================
    # GUARDAR PARA GRAFICAS
    # ========================================================

    with data_lock:

        tiempos.append(
            t
        )

        rpm_datos.append(
            rpm_final
        )

        torque_datos.append(
            ultimo_torque_valido
        )


        # Limitar historial
        if len(tiempos) > MAX_POINTS:

            tiempos.pop(0)

            rpm_datos.pop(0)

            torque_datos.pop(0)


# ============================================================
# MQTT
# ============================================================

def on_connect(
    client,
    userdata,
    flags,
    reason_code,
    properties
):

    global estado_conexion


    if reason_code == 0:

        estado_conexion = (
            "MQTT conectado"
        )

        print(
            f"MQTT conectado a "
            f"{BROKER}:{MQTT_PORT}"
        )

        print(
            f"Topic: {TOPIC}"
        )


        client.subscribe(
            TOPIC
        )


    else:

        estado_conexion = (
            f"MQTT error {reason_code}"
        )


def on_message(
    client,
    userdata,
    msg
):

    try:

        texto = (
            msg.payload.decode(
                "utf-8"
            )
        )


        data = json.loads(
            texto
        )


        procesar_datos(
            data
        )


    except Exception as error:

        print(
            "Error procesando MQTT:",
            error
        )


def iniciar_mqtt():

    global estado_conexion


    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        client_id="Python_Graficas_Micro1"
    )


    client.on_connect = (
        on_connect
    )

    client.on_message = (
        on_message
    )


    try:

        print(
            f"Conectando MQTT a "
            f"{BROKER}:{MQTT_PORT}..."
        )


        client.connect(
            BROKER,
            MQTT_PORT,
            60
        )


        client.loop_forever()


    except Exception as error:

        estado_conexion = (
            f"MQTT desconectado: {error}"
        )

        print(
            estado_conexion
        )


# ============================================================
# BUSCAR ESP32 POR USB
# ============================================================

def buscar_puerto_serial():

    candidatos = []


    for puerto in list_ports.comports():

        dispositivo = (
            puerto.device
        )


        if (
            "usbmodem" in dispositivo
            or
            "usbserial" in dispositivo
        ):

            candidatos.append(
                dispositivo
            )


    if not candidatos:

        raise RuntimeError(
            "No se encontro un puerto USB del ESP32."
        )


    print(
        "Puerto USB encontrado:",
        candidatos[0]
    )


    return candidatos[0]


# ============================================================
# COMUNICACION USB SERIAL
# ============================================================

def iniciar_serial(
    puerto_manual=None
):

    global estado_conexion


    # --------------------------------------------------------
    # ELEGIR PUERTO
    # --------------------------------------------------------

    if puerto_manual:

        puerto = (
            puerto_manual
        )

    else:

        puerto = (
            buscar_puerto_serial()
        )


    print(
        f"Abriendo {puerto} "
        f"a {SERIAL_BAUD} baud..."
    )


    ser = serial.Serial(
        puerto,
        SERIAL_BAUD,
        timeout=0.2
    )


    estado_conexion = (
        f"USB conectado: {puerto}"
    )


    # Al abrir el puerto algunas ESP32 se reinician.
    time.sleep(2.0)


    tara_enviada = False

    cero_enviado = False


    # ========================================================
    # BUCLE SERIAL
    # ========================================================

    while True:

        try:

            raw_line = (
                ser.readline()
            )


            if not raw_line:
                continue


            linea = (
                raw_line.decode(
                    "utf-8",
                    errors="ignore"
                ).strip()
            )


            if not linea:
                continue


            # =================================================
            # MENU DE TARA
            # =================================================

            if (
                not tara_enviada
                and
                "1 = Tara automatica"
                in linea
            ):

                if (
                    SERIAL_TARE_MODE.upper()
                    ==
                    "AUTO"
                ):

                    print(
                        "Realizando tara automatica..."
                    )

                    ser.write(
                        b"1\n"
                    )


                else:

                    print(
                        "Seleccionando CeroFinal manual..."
                    )

                    ser.write(
                        b"2\n"
                    )


                tara_enviada = True


            # =================================================
            # CERO MANUAL
            # =================================================

            if (
                SERIAL_TARE_MODE.upper()
                ==
                "MANUAL"
                and
                tara_enviada
                and
                not cero_enviado
                and
                "Introduzca CeroFinal"
                in linea
            ):

                comando = (
                    f"{MANUAL_ZERO_RAW}\n"
                )


                ser.write(
                    comando.encode(
                        "utf-8"
                    )
                )


                cero_enviado = True


            # =================================================
            # TELEMETRIA
            #
            # Ignorar todo excepto:
            #
            # DATA:{...JSON...}
            # =================================================

            if linea.startswith(
                "DATA:"
            ):

                json_text = (
                    linea[5:]
                )


                try:

                    data = json.loads(
                        json_text
                    )


                    procesar_datos(
                        data
                    )


                except json.JSONDecodeError:

                    print(
                        "JSON serial incompleto."
                    )


        except Exception as error:

            estado_conexion = (
                f"Error USB: {error}"
            )

            print(
                estado_conexion
            )

            time.sleep(1)


# ============================================================
# VENTANA PRINCIPAL
# ============================================================

class VentanaPrincipal(QWidget):

    def __init__(self):

        super().__init__()


        self.setWindowTitle(
            "Banco de Pruebas Motor BLDC"
        )


        self.resize(
            1100,
            800
        )


        layout = (
            QVBoxLayout()
        )


        # ====================================================
        # ESTADO CONEXION
        # ====================================================

        self.labelConexion = QLabel(
            "CONEXION: iniciando..."
        )


        self.labelConexion.setStyleSheet(
            "font-size: 16px; "
            "font-weight: bold;"
        )


        layout.addWidget(
            self.labelConexion
        )


        # ====================================================
        # RPM
        # ====================================================

        self.labelRPM = QLabel(
            "RPM FINAL: 0.0 RPM"
        )


        self.labelRPM.setStyleSheet(
            "font-size: 22px; "
            "font-weight: bold;"
        )


        layout.addWidget(
            self.labelRPM
        )


        self.plotRPM = (
            pg.PlotWidget()
        )


        self.plotRPM.setTitle(
            "Velocidad del Motor vs Tiempo"
        )


        self.plotRPM.setLabel(
            "left",
            "Velocidad",
            units="RPM"
        )


        self.plotRPM.setLabel(
            "bottom",
            "Tiempo",
            units="s"
        )


        self.plotRPM.showGrid(
            x=True,
            y=True
        )


        self.curvaRPM = (
            self.plotRPM.plot()
        )


        layout.addWidget(
            self.plotRPM
        )


        # ====================================================
        # TORQUE
        # ====================================================

        self.labelTorque = QLabel(
            "TORQUE VALIDADO: "
            "0.000000 N·m"
        )


        self.labelTorque.setStyleSheet(
            "font-size: 22px; "
            "font-weight: bold;"
        )


        layout.addWidget(
            self.labelTorque
        )


        self.plotTorque = (
            pg.PlotWidget()
        )


        self.plotTorque.setTitle(
            "Torque vs Tiempo"
        )


        self.plotTorque.setLabel(
            "left",
            "Torque",
            units="N·m"
        )


        self.plotTorque.setLabel(
            "bottom",
            "Tiempo",
            units="s"
        )


        self.plotTorque.showGrid(
            x=True,
            y=True
        )


        self.curvaTorque = (
            self.plotTorque.plot()
        )


        layout.addWidget(
            self.plotTorque
        )


        self.setLayout(
            layout
        )


        # ====================================================
        # TIMER GUI
        # ====================================================

        self.timer = QTimer(
            self
        )


        self.timer.timeout.connect(
            self.actualizar
        )


        self.timer.start(
            100
        )


    # ========================================================
    # ACTUALIZAR GRAFICAS
    # ========================================================

    def actualizar(self):

        self.labelConexion.setText(
            f"CONEXION: "
            f"{estado_conexion}"
        )


        with data_lock:

            if not tiempos:
                return


            x = (
                tiempos.copy()
            )

            rpm = (
                rpm_datos.copy()
            )

            torque = (
                torque_datos.copy()
            )


        # RPM
        self.curvaRPM.setData(
            x,
            rpm
        )


        self.labelRPM.setText(
            f"RPM FINAL: "
            f"{rpm[-1]:.1f} RPM"
        )


        # Torque
        self.curvaTorque.setData(
            x,
            torque
        )


        self.labelTorque.setText(
            f"TORQUE VALIDADO: "
            f"{torque[-1]:.6f} N·m"
        )


# ============================================================
# MAIN
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description=(
            "Visualizacion banco BLDC"
        )
    )


    parser.add_argument(
        "--transport",
        choices=[
            "serial",
            "mqtt"
        ],
        default="serial",
        help=(
            "Medio de comunicacion"
        )
    )


    parser.add_argument(
        "--port",
        default=None,
        help=(
            "Puerto USB manual, "
            "ejemplo /dev/cu.usbmodem1101"
        )
    )


    args = (
        parser.parse_args()
    )


    # ========================================================
    # SELECCIONAR MEDIO
    # ========================================================

    if (
        args.transport
        ==
        "serial"
    ):

        print(
            "MODO: USB SERIAL"
        )


        worker = lambda: (
            iniciar_serial(
                args.port
            )
        )


    else:

        print(
            "MODO: MQTT"
        )


        worker = (
            iniciar_mqtt
        )


    # ========================================================
    # HILO DE COMUNICACION
    # ========================================================

    hilo = threading.Thread(
        target=worker,
        daemon=True
    )


    hilo.start()


    # ========================================================
    # GUI
    # ========================================================

    app = QApplication(
        sys.argv
    )


    ventana = (
        VentanaPrincipal()
    )


    ventana.show()


    sys.exit(
        app.exec()
    )


# ============================================================
# EJECUTAR
# ============================================================

if __name__ == "__main__":

    main()