#include "HX711.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <math.h>

// ============================================================
//       BANCO DE PRUEBAS MOTOR BLDC
//
// RPM + HX711 + PALANCA + TORQUE
// + USB SERIAL + WIFI + MQTT
// ============================================================


// ============================================================
// MEDIOS DE TRANSMISION
// ============================================================

// El mismo JSON se manda por ambos medios.
const bool ENABLE_USB_DATA = true;
const bool ENABLE_MQTT     = true;


// ============================================================
// WIFI
// ============================================================

const char* WIFI_SSID     = "ipJAS";
const char* WIFI_PASSWORD = "coco123123";


// ============================================================
// MQTT  —  HiveMQ Cloud (TLS 8883)
// ============================================================
//
const char* MQTT_SERVER   = "0e44beba4fc7422cb74bc8bbdcc67b2f.s1.eu.hivemq.cloud";
const int   MQTT_PORT     = 8883;
const char* MQTT_USER     = "ESPMICROUNO";
const char* MQTT_PASSWORD = "espmicrouno";

const char* MQTT_TOPIC     = "micro1/motor1/telemetry";
const char* MQTT_TOPIC_CMD = "micro1/motor1/cmd";

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);

uint32_t sequence = 0;

uint32_t lastWiFiAttemptMs = 0;
uint32_t lastMQTTAttemptMs = 0;

// Flag: el Dashboard pidio re-tara via MQTT
volatile bool pendingCalibration = false;


// ============================================================
// PINES
// ============================================================

#define HX711_DOUT  1
#define HX711_SCK   2

#define SPEED_PIN   8

HX711 scale;


// ============================================================
// CALIBRACION GALGA
// ============================================================

// Masa [g] = pendiente * (RAW - CeroFinal)
const double MASS_SLOPE = 0.0006589883;

const double GRAVITY = 9.80665;

const int ZERO_SAMPLES = 30;

const int HX711_RUN_SAMPLES = 5;

double zeroRaw  = 0.0;
bool   calibrado = false;   // true solo después de una tara exitosa


// ============================================================
// GEOMETRIA DE LA PALANCA
// ============================================================
//
// Centro iman ---- 6 cm ---- PIVOTE ---- 4 cm ---- GALGA
//
// Equilibrio:
//
// F_iman * 0.06 = F_galga * 0.04
//
// F_iman = F_galga * 0.04 / 0.06
//
// Torque = F_iman * 0.06
//
// equivalente:
//
// Torque = F_galga * 0.04
//
// ============================================================

const double MAGNET_TO_PIVOT_M = 0.060;

const double PIVOT_TO_LOADCELL_M = 0.040;

// ============================================================
// TORQUE ADICIONAL DEL BRAZO
// ============================================================
//
// Torque constante producido por el brazo [N·m].
// Este valor se SUMA al torque calculado a partir de la galga.
//
//
const double TORQUE_BRAZO_NM = 0.02073;


// ============================================================
// CONFIGURACION VELOCIDAD
// ============================================================

const float TRANSITIONS_PER_REV = 12.0;
const float PULSES_PER_REV = 6.0;

const uint32_t REPORT_INTERVAL_MS = 500;

const uint32_t SIGNAL_TIMEOUT_US = 1000000;

const float FILTER_ALPHA = 0.25;

const float RPM_AGREEMENT_PERCENT = 2.0;


// ============================================================
// INTERRUPCIONES TACOMETRO
// ============================================================

volatile uint32_t edgeCount = 0;
volatile uint32_t risingCount = 0;
volatile uint32_t fallingCount = 0;

volatile uint32_t lastEdgeUs = 0;

volatile uint32_t lastRiseUs = 0;
volatile uint32_t lastFallUs = 0;

volatile uint32_t highTimeUs = 0;
volatile uint32_t lowTimeUs = 0;

volatile uint32_t risePeriodUs = 0;


// ============================================================
// VARIABLES RPM
// ============================================================

float filteredRPM = 0.0;
float finalRPM = 0.0;

bool filterInitialized = false;


// ============================================================
// ISR VELOCIDAD
// ============================================================

void IRAM_ATTR speedISR()
{
  uint32_t now = micros();

  bool state =
    digitalRead(SPEED_PIN);

  edgeCount++;

  lastEdgeUs = now;


  // ----------------------------------------------------------
  // FLANCO ASCENDENTE
  // ----------------------------------------------------------

  if (state == HIGH)
  {
    risingCount++;

    if (lastRiseUs != 0)
    {
      risePeriodUs =
        now - lastRiseUs;
    }

    if (lastFallUs != 0)
    {
      lowTimeUs =
        now - lastFallUs;
    }

    lastRiseUs = now;
  }


  // ----------------------------------------------------------
  // FLANCO DESCENDENTE
  // ----------------------------------------------------------

  else
  {
    fallingCount++;

    if (lastRiseUs != 0)
    {
      highTimeUs =
        now - lastRiseUs;
    }

    lastFallUs = now;
  }
}


// ============================================================
// HX711
// ============================================================

bool waitForHX711(unsigned long timeoutMs)
{
  unsigned long start = millis();

  while (!scale.is_ready())
  {
    if (
      millis() - start >= timeoutMs
    )
    {
      return false;
    }

    delay(5);
  }

  return true;
}


// ============================================================
// PROMEDIO RAW
// ============================================================

bool readAverageRaw(
  int samples,
  double &result
)
{
  double sum = 0.0;

  for (int i = 0; i < samples; i++)
  {
    if (!waitForHX711(2000))
    {
      return false;
    }

    long raw =
      scale.read();

    sum += raw;
  }

  result =
    sum / samples;

  return true;
}


// ============================================================
// LEER OPCION SERIAL
// ============================================================

char readOption()
{
  while (!Serial.available())
  {
    delay(10);
  }

  char option =
    Serial.read();

  delay(50);

  while (Serial.available())
  {
    Serial.read();
  }

  return option;
}


// ============================================================
// LEER NUMERO SERIAL
// ============================================================

double readSerialNumber()
{
  while (!Serial.available())
  {
    delay(10);
  }

  String line =
    Serial.readStringUntil('\n');

  line.trim();

  return line.toDouble();
}


// ============================================================
// TARA INICIAL
// ============================================================

void configureLoadCellZero()
{
  Serial.println();
  Serial.println("======================================================");
  Serial.println("        CONFIGURACION INICIAL DE LA GALGA");
  Serial.println("======================================================");
  Serial.println();

  Serial.print("Pendiente calibracion = ");
  Serial.print(MASS_SLOPE, 10);
  Serial.println(" g/count");

  Serial.println();

  Serial.println("La galga debe estar:");
  Serial.println(" - Instalada en el montaje.");
  Serial.println(" - Sin fuerza aplicada.");
  Serial.println(" - Motor detenido.");

  Serial.println();

  Serial.println("1 = Tara automatica");
  Serial.println("2 = Introducir CeroFinal manual");

  Serial.println();


  char option =
    readOption();


  // ==========================================================
  // TARA AUTOMATICA
  // ==========================================================

  if (option == '1')
  {
    Serial.println();
    Serial.println("TARA AUTOMATICA");
    Serial.println("No toque el montaje...");

    delay(1500);


    if (
      !readAverageRaw(
        ZERO_SAMPLES,
        zeroRaw
      )
    )
    {
      Serial.println(
        "ERROR: HX711 no responde."
      );

      while (1)
      {
        delay(1000);
      }
    }


    Serial.print("CeroFinal = ");
    Serial.println(zeroRaw, 2);
  }


  // ==========================================================
  // CERO MANUAL
  // ==========================================================

  else if (option == '2')
  {
    Serial.println();
    Serial.println(
      "Introduzca CeroFinal:"
    );

    zeroRaw =
      readSerialNumber();

    Serial.print("CeroFinal = ");
    Serial.println(zeroRaw, 2);
  }


  // ==========================================================
  // OPCION INVALIDA -> TARA AUTOMATICA
  // ==========================================================

  else
  {
    Serial.println(
      "Opcion invalida. Tara automatica..."
    );

    delay(1500);


    if (
      !readAverageRaw(
        ZERO_SAMPLES,
        zeroRaw
      )
    )
    {
      Serial.println(
        "ERROR HX711"
      );

      while (1)
      {
        delay(1000);
      }
    }


    Serial.print("CeroFinal = ");
    Serial.println(zeroRaw, 2);
  }




  Serial.println();

  Serial.println(
    "Geometria:"
  );

  Serial.println(
    "Iman -> pivote = 6 cm"
  );

  Serial.println(
    "Pivote -> galga = 4 cm"
  );

  Serial.println();

  Serial.println(
    "Torque = F_galga * 0.04"
  );

  Serial.println();
}


// ============================================================
// TARA AUTOMATICA (sin menu interactivo)
//
// Llamada desde setup() y desde el callback MQTT/Serial
// cuando el Dashboard envia el comando "calibrate".
// ============================================================

bool realizarTaraAutomatica()
{
  Serial.println();
  Serial.println(
    "Realizando tara automatica..."
  );

  Serial.println(
    "No toque el montaje."
  );

  delay(500);

  bool ok =
    readAverageRaw(
      ZERO_SAMPLES,
      zeroRaw
    );

  if (ok)
  {
      calibrado = true;   // habilitar medición en loop()

    Serial.print("CeroFinal = ");
    Serial.println(zeroRaw, 2);

    Serial.println("Tara OK. Iniciando medicion.");

    // Notificar al Dashboard que la tara terminó correctamente
    if (mqttClient.connected())
    {
      mqttClient.publish(
        MQTT_TOPIC,
        "{\"status\":\"tara_ok\"}"
      );
      Serial.println("MQTT: tara_ok publicado.");
    }
  }

  else
  {
    Serial.println(
      "ERROR: HX711 no respondio."
    );

    // Notificar al Dashboard que la tara falló
    if (mqttClient.connected())
    {
      mqttClient.publish(
        MQTT_TOPIC,
        "{\"status\":\"tara_error\"}"
      );
    }
  }

  Serial.println();

  return ok;
}


// ============================================================
// CALLBACK MQTT  —  comandos recibidos desde el Dashboard
// ============================================================

void mqttCallback(
  char* topic,
  byte* payload,
  unsigned int length
)
{
  char buf[256];

  unsigned int n =
    length < sizeof(buf) - 1
    ? length
    : sizeof(buf) - 1;

  memcpy(buf, payload, n);

  buf[n] = '\0';

  // Detectar comando "calibrate" en el JSON
  if (strstr(buf, "calibrate") != NULL)
  {
    pendingCalibration = true;

    Serial.println(
      "Comando MQTT: calibrate recibido."
    );
  }
}


// ============================================================
// INICIAR WIFI
//
// IMPORTANTE:
// NO bloquea el programa.
// Si no hay WiFi, USB sigue funcionando.
// ============================================================

void iniciarWiFi()
{
  if (!ENABLE_MQTT)
  {
    return;
  }

  Serial.println();
  Serial.println(
    "Iniciando WiFi..."
  );

  WiFi.mode(WIFI_STA);

  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );

  lastWiFiAttemptMs =
    millis();
}


// ============================================================
// MANTENER WIFI + MQTT
//
// No bloquea la adquisicion.
// ============================================================

void mantenerRed()
{
  if (!ENABLE_MQTT)
  {
    return;
  }


  // ==========================================================
  // WIFI
  // ==========================================================

  if (
    WiFi.status()
    != WL_CONNECTED
  )
  {
    if (
      millis() -
      lastWiFiAttemptMs
      >= 5000
    )
    {
      lastWiFiAttemptMs =
        millis();

      Serial.println(
        "Reintentando WiFi..."
      );

      WiFi.disconnect();

      WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
      );
    }

    return;
  }


  // ==========================================================
  // MQTT
  // ==========================================================

  if (!mqttClient.connected())
  {
    if (
      millis() -
      lastMQTTAttemptMs
      >= 3000
    )
    {
      lastMQTTAttemptMs =
        millis();

      Serial.print(
        "Conectando MQTT... "
      );


      if (
        mqttClient.connect(
          "ESP32_Micro1",
          MQTT_USER,
          MQTT_PASSWORD
        )
      )
      {
        Serial.println("OK");

        // Suscribirse al topic de comandos del Dashboard
        mqttClient.subscribe(
          MQTT_TOPIC_CMD
        );
      }

      else
      {
        Serial.print("ERROR ");
        Serial.println(
          mqttClient.state()
        );
      }
    }

    return;
  }


  mqttClient.loop();
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(2000);


  Serial.println();
  Serial.println("======================================================");
  Serial.println("       BANCO DE PRUEBAS MOTOR BLDC");
  Serial.println(" RPM + TORQUE + USB + WIFI + MQTT");
  Serial.println("======================================================");


  // ==========================================================
  // HX711
  // ==========================================================

  scale.begin(
    HX711_DOUT,
    HX711_SCK
  );


  Serial.println(
    "Buscando HX711..."
  );


  if (!waitForHX711(5000))
  {
    Serial.println(
      "ERROR: HX711 no encontrado."
    );

    while (1)
    {
      delay(1000);
    }
  }


  Serial.println(
    "HX711 encontrado correctamente."
  );


  // ==========================================================
  // TARA  —  controlada por el Dashboard (MQTT o Serial 'C')
  // ==========================================================
  // La tara se ejecuta únicamente cuando el Dashboard la solicita,
  // para garantizar que el montaje esté estabilizado antes de referenciar cero.
  Serial.println("ESP32 lista. Esperando comando de calibracion desde el Dashboard.");


  // ==========================================================
  // TACOMETRO
  // ==========================================================

  pinMode(
    SPEED_PIN,
    INPUT
  );


  attachInterrupt(
    digitalPinToInterrupt(SPEED_PIN),
    speedISR,
    CHANGE
  );


  Serial.println(
    "Tacometro listo."
  );


  // ==========================================================
  // MQTT
  // ==========================================================

  if (ENABLE_MQTT)
  {
    // Acepta el certificado TLS del broker sin verificar la CA raiz.
    // Suficiente para HiveMQ Cloud en un entorno de laboratorio.
    wifiClient.setInsecure();

    mqttClient.setServer(
      MQTT_SERVER,
      MQTT_PORT
    );

    mqttClient.setCallback(
      mqttCallback
    );

    mqttClient.setBufferSize(
      1600
    );

    iniciarWiFi();
  }


  Serial.println();
  Serial.println("======================================================");
  Serial.println("           INICIANDO MEDICION");
  Serial.println("======================================================");
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  static uint32_t previousReportMs =
    millis();


  // Mantener red sin bloquear
  mantenerRed();


  // ==========================================================
  // RE-CALIBRACION POR MQTT  (comando del Dashboard)
  // ==========================================================

  if (pendingCalibration)
  {
    pendingCalibration = false;
    realizarTaraAutomatica();
  }


  // ==========================================================
  // RE-CALIBRACION POR SERIAL  (envia 'C' desde cualquier terminal)
  // ==========================================================

  if (Serial.available())
  {
    char c = Serial.read();

    if (c == 'C' || c == 'c')
    {
      realizarTaraAutomatica();
    }
  }


  // ==========================================================
  // MEDICION CADA 500 ms  (solo si ya se realizó la tara)
  // ==========================================================

  if (!calibrado)
  {
    return;   // esperar calibración — WiFi y MQTT siguen activos
  }

  if (
    millis() - previousReportMs
    >= REPORT_INTERVAL_MS
  )
  {
    uint32_t nowMs =
      millis();


    uint32_t elapsedMs =
      nowMs -
      previousReportMs;


    previousReportMs =
      nowMs;


    float elapsedSeconds =
      elapsedMs /
      1000.0;


    // ========================================================
    // COPIAR DATOS DE INTERRUPCION
    // ========================================================

    noInterrupts();


    uint32_t edges =
      edgeCount;

    uint32_t rises =
      risingCount;

    uint32_t falls =
      fallingCount;

    uint32_t lastEdge =
      lastEdgeUs;

    uint32_t period =
      risePeriodUs;

    uint32_t highUs =
      highTimeUs;

    uint32_t lowUs =
      lowTimeUs;


    edgeCount = 0;
    risingCount = 0;
    fallingCount = 0;


    interrupts();


    // ========================================================
    // MOTOR ACTIVO
    // ========================================================

    bool signalActive = false;


    if (lastEdge != 0)
    {
      uint32_t dt =
        micros() -
        lastEdge;


      if (
        dt <
        SIGNAL_TIMEOUT_US
      )
      {
        signalActive =
          true;
      }
    }


    // ========================================================
    // VARIABLES VELOCIDAD
    // ========================================================

    float transitionsPerSecond = 0.0;

    float pulseFrequencyCount = 0.0;

    float pulseFrequencyPeriod = 0.0;

    float rpmCount = 0.0;

    float rpmPeriod = 0.0;

    float duty = 0.0;

    float rpmDifferencePercent = 0.0;

    bool rpmAgreement = false;


    // ========================================================
    // MOTOR GIRANDO
    // ========================================================

    if (signalActive)
    {
      transitionsPerSecond =
        edges /
        elapsedSeconds;


      pulseFrequencyCount =
        rises /
        elapsedSeconds;


      // RPM conteo
      rpmCount =
        (
          transitionsPerSecond *
          60.0
        )
        /
        TRANSITIONS_PER_REV;


      // RPM periodo
      if (period > 0)
      {
        pulseFrequencyPeriod =
          1000000.0 /
          period;


        rpmPeriod =
          (
            pulseFrequencyPeriod *
            60.0
          )
          /
          PULSES_PER_REV;
      }


      // Duty
      uint32_t pulseTime =
        highUs +
        lowUs;


      if (pulseTime > 0)
      {
        duty =
          100.0 *
          (
            (float)highUs /
            pulseTime
          );
      }


      // ======================================================
      // RPM FILTRADA
      // ======================================================

      if (!filterInitialized)
      {
        filteredRPM =
          rpmCount;

        filterInitialized =
          true;
      }

      else
      {
        filteredRPM =
          FILTER_ALPHA *
          rpmCount
          +
          (
            1.0 -
            FILTER_ALPHA
          )
          *
          filteredRPM;
      }


      // ======================================================
      // RPM FINAL
      //
      // Solo cambia cuando ambos metodos concuerdan.
      // ======================================================

      if (
        rpmCount > 0.0 &&
        rpmPeriod > 0.0
      )
      {
        float difference =
          fabs(
            rpmCount -
            rpmPeriod
          );


        rpmDifferencePercent =
          100.0 *
          difference /
          rpmCount;


        if (
          rpmDifferencePercent
          <=
          RPM_AGREEMENT_PERCENT
        )
        {
          rpmAgreement =
            true;


          finalRPM =
            (
              rpmCount +
              rpmPeriod
            )
            /
            2.0;
        }
      }
    }


    // ========================================================
    // MOTOR APAGADO
    // ========================================================

    else
    {
      transitionsPerSecond = 0.0;

      pulseFrequencyCount = 0.0;

      pulseFrequencyPeriod = 0.0;

      rpmCount = 0.0;

      rpmPeriod = 0.0;

      filteredRPM = 0.0;

      finalRPM = 0.0;

      filterInitialized = false;

      rpmAgreement = true;
    }


    // ========================================================
    // GALGA
    // ========================================================

    double raw = 0.0;


    bool hxOK =
      readAverageRaw(
        HX711_RUN_SAMPLES,
        raw
      );


    double mass_g = 0.0;

    double forceLoadCell_N = 0.0;

    double forceMagnet_N = 0.0;

    double torqueGalga_Nm = 0.0;

    double torque_Nm = 0.0;


    if (hxOK)
    {
      // ======================================================
      // MASA DIRECTA DE LA GALGA — SIN FILTRO KALMAN
      // ======================================================
      mass_g =
        MASS_SLOPE *
        (
          raw -
          zeroRaw
        );


      // Fuerza medida por la galga
      forceLoadCell_N =
        (
          mass_g /
          1000.0
        )
        *
        GRAVITY;


      // Fuerza equivalente en el iman
      forceMagnet_N =
        forceLoadCell_N *
        (
          PIVOT_TO_LOADCELL_M /
          MAGNET_TO_PIVOT_M
        );


      // Torque medido por la galga
      torqueGalga_Nm =
        forceMagnet_N *
        MAGNET_TO_PIVOT_M;


      // Torque total = torque de galga + torque constante del brazo
      torque_Nm =
        torqueGalga_Nm +
        TORQUE_BRAZO_NM;
    }


    // ========================================================
    // MOSTRAR SERIAL
    // ========================================================

    Serial.println();

    Serial.println(
      "======================================================"
    );


    Serial.print(
      "RPM conteo        : "
    );

    Serial.println(
      rpmCount,
      2
    );


    Serial.print(
      "RPM periodo       : "
    );

    Serial.println(
      rpmPeriod,
      2
    );


    Serial.print(
      "RPM filtrada      : "
    );

    Serial.println(
      filteredRPM,
      2
    );


    Serial.print(
      "RPM FINAL         : "
    );

    Serial.println(
      finalRPM,
      2
    );


    Serial.print(
      "Masa galga        : "
    );

    Serial.print(
      mass_g,
      4
    );

    Serial.println(" g");


    Serial.print(
      "Fuerza galga      : "
    );

    Serial.print(
      forceLoadCell_N,
      6
    );

    Serial.println(" N");


    Serial.print(
      "Torque galga      : "
    );

    Serial.print(
      torqueGalga_Nm,
      8
    );

    Serial.println(" N*m");


    Serial.print(
      "Torque brazo      : "
    );

    Serial.print(
      TORQUE_BRAZO_NM,
      8
    );

    Serial.println(" N*m");


    Serial.print(
      "Torque TOTAL      : "
    );

    Serial.print(
      torque_Nm,
      8
    );

    Serial.println(" N*m");


    // ========================================================
    // CREAR JSON
    // ========================================================

    sequence++;


    char mensaje[1400];


    snprintf(
      mensaje,
      sizeof(mensaje),

      "{"

      "\"seq\":%lu,"
      "\"time_ms\":%lu,"
      "\"elapsed_ms\":%lu,"

      "\"rpm_filtered\":%.2f,"
      "\"rpm_count\":%.2f,"
      "\"rpm_period\":%.2f,"
      "\"rpm_final\":%.2f,"
      "\"rpm_difference_percent\":%.3f,"

      "\"rpm_agreement\":%s,"
      "\"signal_active\":%s,"

      "\"raw\":%.2f,"
      "\"zero_raw\":%.2f,"

      "\"mass_unfiltered_g\":%.4f,"
      "\"mass_g\":%.4f,"

      "\"force_N\":%.6f,"
      "\"force_magnet_N\":%.6f,"
      "\"torque_galga_Nm\":%.8f,"
      "\"torque_brazo_Nm\":%.8f,"
      "\"torque_Nm\":%.8f,"

      "\"magnet_to_pivot_m\":%.3f,"
      "\"pivot_to_loadcell_m\":%.3f,"

      "\"hx_ok\":%s,"

      "\"frequency_count_Hz\":%.3f,"
      "\"frequency_period_Hz\":%.3f,"
      "\"transitions_per_second\":%.3f,"

      "\"edges\":%lu,"
      "\"rising_edges\":%lu,"
      "\"falling_edges\":%lu,"

      "\"period_us\":%lu,"
      "\"high_time_us\":%lu,"
      "\"low_time_us\":%lu,"

      "\"duty_percent\":%.3f"

      "}",


      (unsigned long)sequence,
      (unsigned long)nowMs,
      (unsigned long)elapsedMs,

      filteredRPM,
      rpmCount,
      rpmPeriod,
      finalRPM,
      rpmDifferencePercent,

      rpmAgreement
        ? "true"
        : "false",

      signalActive
        ? "true"
        : "false",

      raw,
      zeroRaw,

      mass_g,
      mass_g,

      forceLoadCell_N,
      forceMagnet_N,
      torqueGalga_Nm,
      TORQUE_BRAZO_NM,
      torque_Nm,

      MAGNET_TO_PIVOT_M,
      PIVOT_TO_LOADCELL_M,

      hxOK
        ? "true"
        : "false",

      pulseFrequencyCount,
      pulseFrequencyPeriod,
      transitionsPerSecond,

      (unsigned long)edges,
      (unsigned long)rises,
      (unsigned long)falls,

      (unsigned long)period,
      (unsigned long)highUs,
      (unsigned long)lowUs,

      duty
    );


    // ========================================================
    // TRANSMITIR POR USB
    // ========================================================

    if (ENABLE_USB_DATA)
    {
      // Python solamente lee las lineas que empiezan DATA:
      Serial.print("DATA:");
      Serial.println(mensaje);
    }


    // ========================================================
    // TRANSMITIR POR MQTT
    // ========================================================

    bool mqttEnviado = false;


    if (
      ENABLE_MQTT &&
      mqttClient.connected()
    )
    {
      mqttEnviado =
        mqttClient.publish(
          MQTT_TOPIC,
          mensaje
        );
    }


    Serial.print(
      "USB DATA          : "
    );

    Serial.println(
      ENABLE_USB_DATA
        ? "OK"
        : "DESACTIVADO"
    );


    Serial.print(
      "MQTT              : "
    );


    if (!ENABLE_MQTT)
    {
      Serial.println(
        "DESACTIVADO"
      );
    }

    else if (mqttEnviado)
    {
      Serial.println("OK");
    }

    else
    {
      Serial.println(
        "SIN CONEXION"
      );
    }


    Serial.println(
      "======================================================"
    );
  }
}