import paho.mqtt.client as mqtt

# ============================================================
# CONFIGURACION MQTT
# ============================================================

BROKER = "192.168.20.24"
PORT = 1883

TOPIC = "micro1/test"


# ============================================================
# CUANDO PYTHON SE CONECTA AL BROKER
# ============================================================

def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("MQTT conectado correctamente")
        print(f"Broker: {BROKER}:{PORT}")
        print(f"Suscrito a: {TOPIC}")
        print()

        client.subscribe(TOPIC)

    else:
        print("Error de conexion MQTT")
        print("Codigo:", reason_code)


# ============================================================
# CUANDO LLEGA UN MENSAJE
# ============================================================

def on_message(client, userdata, msg):
    mensaje = msg.payload.decode("utf-8")

    print("Mensaje recibido:")
    print("Topic:", msg.topic)
    print("Data :", mensaje)
    print("-----------------------------")


# ============================================================
# CREAR CLIENTE MQTT
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
# MANTENER PROGRAMA ESCUCHANDO
# ============================================================

client.loop_forever()