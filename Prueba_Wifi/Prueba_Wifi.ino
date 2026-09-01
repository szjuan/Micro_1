#include <WiFi.h>
#include <PubSubClient.h>

// ============================================================
// WIFI
// ============================================================

const char* WIFI_SSID = "Claro_6E09BE";
const char* WIFI_PASSWORD = "A9Y9A3W2Y3S4";

// ============================================================
// MQTT
// ============================================================

// IP del Mac donde esta corriendo Mosquitto
const char* MQTT_SERVER = "192.168.20.24";

const int MQTT_PORT = 1883;

const char* MQTT_TOPIC = "micro1/test";

// ============================================================
// OBJETOS
// ============================================================

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Contador de mensajes
unsigned long contador = 0;

// Tiempo del ultimo envio
unsigned long previousPublish = 0;


// ============================================================
// CONECTAR WIFI
// ============================================================

void conectarWiFi()
{
  Serial.println();
  Serial.println("======================================");
  Serial.println("        CONECTANDO AL WIFI");
  Serial.println("======================================");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println();
  Serial.println("WiFi conectado correctamente.");

  Serial.print("IP ESP32: ");
  Serial.println(WiFi.localIP());

  Serial.print("RSSI: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}


// ============================================================
// CONECTAR MQTT
// ============================================================

void conectarMQTT()
{
  while (!mqttClient.connected())
  {
    Serial.println();
    Serial.print("Conectando al broker MQTT ");
    Serial.print(MQTT_SERVER);
    Serial.print(":");
    Serial.println(MQTT_PORT);

    // Nombre del cliente MQTT
    if (mqttClient.connect("ESP32_Micro1"))
    {
      Serial.println("MQTT conectado correctamente.");
    }
    else
    {
      Serial.print("ERROR MQTT. Codigo = ");
      Serial.println(mqttClient.state());

      Serial.println("Reintentando en 2 segundos...");

      delay(2000);
    }
  }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(2000);

  Serial.println();
  Serial.println("======================================");
  Serial.println("     PRUEBA ESP32 + WIFI + MQTT");
  Serial.println("======================================");

  // WiFi
  conectarWiFi();

  // Configurar MQTT
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

  // MQTT
  conectarMQTT();

  Serial.println();
  Serial.println("Sistema listo.");
  Serial.println("Enviando mensaje cada segundo.");
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  // Si se pierde WiFi
  if (WiFi.status() != WL_CONNECTED)
  {
    conectarWiFi();
  }

  // Si se pierde MQTT
  if (!mqttClient.connected())
  {
    conectarMQTT();
  }

  mqttClient.loop();


  // ==========================================================
  // PUBLICAR CADA 1 SEGUNDO
  // ==========================================================

  if (millis() - previousPublish >= 1000)
  {
    previousPublish = millis();

    contador++;

    char mensaje[100];

    snprintf(
      mensaje,
      sizeof(mensaje),
      "Mensaje ESP32 numero %lu",
      contador
    );


    bool enviado =
      mqttClient.publish(
        MQTT_TOPIC,
        mensaje
      );


    Serial.print("Publicando: ");
    Serial.print(mensaje);

    if (enviado)
    {
      Serial.println(" -> OK");
    }
    else
    {
      Serial.println(" -> ERROR");
    }
  }
}