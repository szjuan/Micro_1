import json
import paho.mqtt.client as mqtt


# ============================================================
# CONFIGURACION MQTT
# ============================================================

BROKER = "192.168.20.24"
PORT = 1883

TOPIC = "micro1/motor1/telemetry"


# ============================================================
# CONEXION MQTT
# ============================================================

def on_connect(client, userdata, flags, reason_code, properties):

    if reason_code == 0:

        print()
        print("==============================================")
        print("MQTT CONECTADO CORRECTAMENTE")
        print("==============================================")
        print(f"Broker : {BROKER}:{PORT}")
        print(f"Topic  : {TOPIC}")
        print()

        client.subscribe(TOPIC)

    else:

        print("ERROR MQTT")
        print("Codigo:", reason_code)


# ============================================================
# RECIBIR DATOS
# ============================================================

def on_message(client, userdata, msg):

    try:

        # ----------------------------------------------------
        # DECODIFICAR JSON
        # ----------------------------------------------------

        mensaje = msg.payload.decode("utf-8")

        data = json.loads(mensaje)


        # ====================================================
        # INFORMACION GENERAL
        # ====================================================

        seq = data["seq"]

        time_ms = data["time_ms"]

        elapsed_ms = data["elapsed_ms"]


        # ====================================================
        # VELOCIDAD
        # ====================================================

        rpm_filtered = data["rpm_filtered"]

        rpm_count = data["rpm_count"]

        rpm_period = data["rpm_period"]

        rpm_final = data["rpm_final"]

        rpm_difference = data["rpm_difference_percent"]

        rpm_agreement = data["rpm_agreement"]

        signal_active = data["signal_active"]


        # ====================================================
        # GALGA
        # ====================================================

        raw = data["raw"]

        zero_raw = data["zero_raw"]

        mass_g = data["mass_g"]

        force_N = data["force_N"]

        torque_Nm = data["torque_Nm"]

        hx_ok = data["hx_ok"]


        # ====================================================
        # INFORMACION DE LA SEÑAL
        # ====================================================

        frequency_count = data["frequency_count_Hz"]

        frequency_period = data["frequency_period_Hz"]

        transitions_per_second = data["transitions_per_second"]

        edges = data["edges"]

        rising_edges = data["rising_edges"]

        falling_edges = data["falling_edges"]

        period_us = data["period_us"]

        high_time_us = data["high_time_us"]

        low_time_us = data["low_time_us"]

        duty_percent = data["duty_percent"]


        # ====================================================
        # MOSTRAR
        # ====================================================

        print()
        print("==============================================================")
        print(f"MUESTRA {seq}")
        print("==============================================================")

        print()

        print("--- TIEMPO ---")

        print(f"Tiempo ESP32       : {time_ms} ms")
        print(f"Intervalo          : {elapsed_ms} ms")


        print()
        print("--- VELOCIDAD ---")

        print(f"RPM filtradas      : {rpm_filtered:.2f} RPM")
        print(f"RPM por conteo     : {rpm_count:.2f} RPM")
        print(f"RPM por periodo    : {rpm_period:.2f} RPM")
        print(f"RPM FINAL          : {rpm_final:.2f} RPM")

        print(f"Diferencia         : {rpm_difference:.3f} %")

        print(
            f"Concordancia       : "
            f"{'SI' if rpm_agreement else 'NO'}"
        )

        print(
            f"Señal activa       : "
            f"{'SI' if signal_active else 'NO'}"
        )


        print()
        print("--- GALGA ---")

        print(f"RAW                : {raw:.2f}")

        print(f"CeroFinal          : {zero_raw:.2f}")

        print(f"Masa               : {mass_g:.4f} g")

        print(f"Fuerza             : {force_N:.6f} N")

        print(f"Torque             : {torque_Nm:.8f} N*m")

        print(
            f"HX711              : "
            f"{'OK' if hx_ok else 'ERROR'}"
        )


        print()
        print("--- SEÑAL DE VELOCIDAD ---")

        print(
            f"Frecuencia conteo  : "
            f"{frequency_count:.3f} Hz"
        )

        print(
            f"Frecuencia periodo : "
            f"{frequency_period:.3f} Hz"
        )

        print(
            f"Transiciones/s     : "
            f"{transitions_per_second:.3f}"
        )

        print(f"Transiciones       : {edges}")

        print(f"Flancos subida     : {rising_edges}")

        print(f"Flancos bajada     : {falling_edges}")

        print(f"Periodo            : {period_us} us")

        print(f"Tiempo HIGH        : {high_time_us} us")

        print(f"Tiempo LOW         : {low_time_us} us")

        print(f"Duty cycle         : {duty_percent:.3f} %")

        print()
        print("==============================================================")


    except json.JSONDecodeError:

        print("ERROR: JSON no valido")


    except KeyError as error:

        print("ERROR: Falta una variable:")
        print(error)


# ============================================================
# CREAR CLIENTE
# ============================================================

client = mqtt.Client(
    mqtt.CallbackAPIVersion.VERSION2,
    client_id="Python_Micro1"
)

client.on_connect = on_connect

client.on_message = on_message


# ============================================================
# CONECTAR
# ============================================================

print("Conectando a Mosquitto...")

client.connect(
    BROKER,
    PORT,
    keepalive=60
)


# ============================================================
# EJECUTAR
# ============================================================

client.loop_forever()