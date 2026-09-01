#include "HX711.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>

// ============================================================
//        BANCO DE PRUEBAS MOTOR BLDC
//
//   VELOCIDAD + GALGA + KALMAN
//   + SISTEMA DE PALANCA CON PIVOTE
//   + FUERZA + TORQUE + WiFi + MQTT
//
// ============================================================


// ============================================================
// WIFI
// ============================================================

const char* WIFI_SSID = "IPJAS";
const char* WIFI_PASSWORD = "coco123123";


// ============================================================
// MQTT
// ============================================================

const char* MQTT_SERVER = "172.20.10.2";
const int MQTT_PORT = 1883;

const char* MQTT_TOPIC = "micro1/motor1/telemetry";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long sequence = 0;


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

// Masa [g] = MASS_SLOPE * (RAW - CeroFinal)

const double MASS_SLOPE = 0.0006589883;

// Gravedad
const double GRAVITY = 9.80665;

// Número de muestras para tara
const int ZERO_SAMPLES = 30;

// Número de muestras durante funcionamiento
const int HX711_RUN_SAMPLES = 5;

// Cero
double zeroRaw = 0.0;


// ============================================================
// GEOMETRIA DEL SISTEMA DE PALANCA
// ============================================================
//
//        CENTRO IMAN
//             ↓
//             ●
//             |
//             |  6 cm
//             |
//          PIVOTE O
//             |
//             |  4 cm
//             |
//             ●
//           GALGA
//
//
// Equilibrio de momentos:
//
// F_iman * 0.06 = F_galga * 0.04
//
// F_iman = F_galga * 0.04 / 0.06
//
// Torque = F_iman * 0.06
//
//        = F_galga * 0.04
//
// ============================================================

// Centro del imán -> pivote
const double MAGNET_TO_PIVOT_M = 0.060;

// Pivote -> punto de aplicación sobre la galga
const double PIVOT_TO_LOADCELL_M = 0.040;


// ============================================================
// FILTRO KALMAN GALGA
// ============================================================

const double KALMAN_Q = 0.05;
const double KALMAN_R = 1.00;

double kalmanMassEstimate = 0.0;
double kalmanErrorEstimate = 1.0;
double kalmanGain = 0.0;

bool kalmanInitialized = false;


// ============================================================
// VELOCIDAD
// ============================================================

const float TRANSITIONS_PER_REV = 12.0;
const float PULSES_PER_REV = 6.0;

const uint32_t REPORT_INTERVAL_MS = 500;

const uint32_t SIGNAL_TIMEOUT_US = 1000000;

const float FILTER_ALPHA = 0.25;

const float RPM_AGREEMENT_PERCENT = 2.0;


// ============================================================
// VARIABLES INTERRUPCION
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
// RESET KALMAN
// ============================================================

void resetKalman()
{
  kalmanMassEstimate = 0.0;
  kalmanErrorEstimate = 1.0;
  kalmanGain = 0.0;

  kalmanInitialized = true;
}


// ============================================================
// FILTRO KALMAN
// ============================================================

double applyKalman(double measurement)
{
  if (!kalmanInitialized)
  {
    kalmanMassEstimate = measurement;
    kalmanErrorEstimate = 1.0;
    kalmanGain = 1.0;

    kalmanInitialized = true;

    return kalmanMassEstimate;
  }


  // Predicción
  double predictedEstimate =
    kalmanMassEstimate;

  double predictedError =
    kalmanErrorEstimate + KALMAN_Q;


  // Ganancia
  kalmanGain =
    predictedError /
    (predictedError + KALMAN_R);


  // Corrección
  kalmanMassEstimate =
    predictedEstimate +
    kalmanGain *
    (measurement - predictedEstimate);


  // Actualización del error
  kalmanErrorEstimate =
    (1.0 - kalmanGain) *
    predictedError;


  return kalmanMassEstimate;
}


// ============================================================
// ISR TACOMETRO
// ============================================================

void IRAM_ATTR speedISR()
{
  uint32_t now = micros();

  bool state =
    digitalRead(SPEED_PIN);

  edgeCount++;

  lastEdgeUs = now;


  // ==========================================================
  // FLANCO ASCENDENTE
  // ==========================================================

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


  // ==========================================================
  // FLANCO DESCENDENTE
  // ==========================================================

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
// WIFI
// ============================================================

void conectarWiFi()
{
  Serial.println();
  Serial.println("Conectando al WiFi...");

  WiFi.mode(WIFI_STA);

  WiFi.begin(
    WIFI_SSID,
    WIFI_PASSWORD
  );


  while (
    WiFi.status() != WL_CONNECTED
  )
  {
    delay(500);
    Serial.print(".");
  }


  Serial.println();
  Serial.println(
    "WiFi conectado correctamente."
  );


  Serial.print("IP ESP32: ");
  Serial.println(WiFi.localIP());


  Serial.print("RSSI: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}


// ============================================================
// MQTT
// ============================================================

void conectarMQTT()
{
  while (!mqttClient.connected())
  {
    Serial.print(
      "Conectando MQTT... "
    );


    if (
      mqttClient.connect(
        "ESP32_Micro1"
      )
    )
    {
      Serial.println("OK");
    }

    else
    {
      Serial.print("ERROR = ");

      Serial.println(
        mqttClient.state()
      );

      delay(2000);
    }
  }
}


// ============================================================
// ESPERAR HX711
// ============================================================

bool waitForHX711(
  unsigned long timeoutMs
)
{
  unsigned long start =
    millis();


  while (!scale.is_ready())
  {
    if (
      millis() - start
      >= timeoutMs
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


  for (
    int i = 0;
    i < samples;
    i++
  )
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
// TARA GALGA
// ============================================================

void configureLoadCellZero()
{
  Serial.println();
  Serial.println(
    "======================================================"
  );

  Serial.println(
    "        CONFIGURACION INICIAL DE LA GALGA"
  );

  Serial.println(
    "======================================================"
  );

  Serial.println();


  Serial.println(
    "Pendiente de calibracion:"
  );

  Serial.print(
    MASS_SLOPE,
    10
  );

  Serial.println(" g/count");


  Serial.println();
  Serial.println(
    "La galga debe estar:"
  );

  Serial.println(
    " - Instalada en el montaje final."
  );

  Serial.println(
    " - Sin fuerza aplicada."
  );

  Serial.println(
    " - Motor detenido."
  );

  Serial.println();


  Serial.println(
    "1 = Tara automatica"
  );

  Serial.println(
    "2 = Introducir CeroFinal manual"
  );

  Serial.println();


  char option =
    readOption();


  // ==========================================================
  // AUTOMATICA
  // ==========================================================

  if (option == '1')
  {
    Serial.println();
    Serial.println(
      "TARA AUTOMATICA"
    );

    Serial.println(
      "No toque el montaje..."
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


    Serial.print(
      "CeroFinal = "
    );

    Serial.println(
      zeroRaw,
      2
    );
  }


  // ==========================================================
  // MANUAL
  // ==========================================================

  else if (option == '2')
  {
    Serial.println();

    Serial.println(
      "Introduzca CeroFinal:"
    );


    zeroRaw =
      readSerialNumber();


    Serial.print(
      "CeroFinal = "
    );

    Serial.println(
      zeroRaw,
      2
    );
  }


  // ==========================================================
  // INVALIDA
  // ==========================================================

  else
  {
    Serial.println(
      "Opcion invalida."
    );

    Serial.println(
      "Tara automatica..."
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
  }


  resetKalman();


  Serial.println();
  Serial.println(
    "Geometria configurada:"
  );

  Serial.print(
    "Iman -> pivote = "
  );

  Serial.print(
    MAGNET_TO_PIVOT_M * 100.0,
    1
  );

  Serial.println(" cm");


  Serial.print(
    "Pivote -> galga = "
  );

  Serial.print(
    PIVOT_TO_LOADCELL_M * 100.0,
    1
  );

  Serial.println(" cm");


  Serial.println();
  Serial.println(
    "F_iman = F_galga * 0.04 / 0.06"
  );

  Serial.println(
    "Torque = F_iman * 0.06"
  );

  Serial.println(
    "Torque = F_galga * 0.04"
  );

  Serial.println();
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(2000);


  Serial.println();
  Serial.println(
    "======================================================"
  );

  Serial.println(
    "      BANCO DE PRUEBAS MOTOR BLDC"
  );

  Serial.println(
    "   GALGA + PIVOTE + KALMAN + RPM + MQTT"
  );

  Serial.println(
    "======================================================"
  );


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
  // TARA
  // ==========================================================

  configureLoadCellZero();


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
    "Sensor velocidad listo."
  );


  // ==========================================================
  // WIFI
  // ==========================================================

  conectarWiFi();


  // ==========================================================
  // MQTT
  // ==========================================================

  mqttClient.setServer(
    MQTT_SERVER,
    MQTT_PORT
  );


  mqttClient.setBufferSize(
    1600
  );


  conectarMQTT();


  Serial.println();
  Serial.println(
    "======================================================"
  );

  Serial.println(
    "           INICIANDO MEDICION"
  );

  Serial.println(
    "======================================================"
  );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  static uint32_t previousReportMs =
    millis();


  // ==========================================================
  // WIFI
  // ==========================================================

  if (
    WiFi.status()
    != WL_CONNECTED
  )
  {
    conectarWiFi();
  }


  // ==========================================================
  // MQTT
  // ==========================================================

  if (!mqttClient.connected())
  {
    conectarMQTT();
  }


  mqttClient.loop();


  // ==========================================================
  // MEDICION
  // ==========================================================

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
    // DATOS INTERRUPCION
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
    // DETECTAR MOTOR
    // ========================================================

    bool signalActive =
      false;


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

    float transitionsPerSecond =
      0.0;

    float pulseFrequencyCount =
      0.0;

    float pulseFrequencyPeriod =
      0.0;

    float rpmCount =
      0.0;

    float rpmPeriod =
      0.0;

    float duty =
      0.0;

    float rpmDifferencePercent =
      0.0;

    bool rpmAgreement =
      false;


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


      rpmCount =
        (
          transitionsPerSecond *
          60.0
        )
        /
        TRANSITIONS_PER_REV;


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
      transitionsPerSecond =
        0.0;

      pulseFrequencyCount =
        0.0;

      pulseFrequencyPeriod =
        0.0;

      rpmCount =
        0.0;

      rpmPeriod =
        0.0;

      filteredRPM =
        0.0;

      finalRPM =
        0.0;

      filterInitialized =
        false;

      rpmAgreement =
        true;
    }


    // ========================================================
    // GALGA
    // ========================================================

    double raw =
      0.0;


    bool hxOK =
      readAverageRaw(
        HX711_RUN_SAMPLES,
        raw
      );


    double massUnfiltered_g =
      0.0;

    double massFiltered_g =
      0.0;


    // Fuerza realmente medida por la galga
    double forceLoadCell_N =
      0.0;


    // Fuerza equivalente en el centro del imán
    double forceMagnet_N =
      0.0;


    // Torque calculado
    double torque_Nm =
      0.0;


    if (hxOK)
    {
      // ======================================================
      // MASA SIN FILTRO
      // ======================================================

      massUnfiltered_g =
        MASS_SLOPE *
        (
          raw -
          zeroRaw
        );


      // ======================================================
      // KALMAN
      // ======================================================

      massFiltered_g =
        applyKalman(
          massUnfiltered_g
        );


      // ======================================================
      // FUERZA GALGA
      // ======================================================

      forceLoadCell_N =
        (
          massFiltered_g /
          1000.0
        )
        *
        GRAVITY;


      // ======================================================
      // FUERZA EN EL CENTRO DEL IMAN
      //
      // F_iman * 0.06
      // =
      // F_galga * 0.04
      //
      // ======================================================

      forceMagnet_N =
        forceLoadCell_N *
        (
          PIVOT_TO_LOADCELL_M /
          MAGNET_TO_PIVOT_M
        );


      // ======================================================
      // TORQUE
      //
      // T = F_iman * 0.06
      //
      // ======================================================

      torque_Nm =
        forceMagnet_N *
        MAGNET_TO_PIVOT_M;
    }


    // ========================================================
    // SERIAL
    // ========================================================

    Serial.println();
    Serial.println(
      "======================================================"
    );


    Serial.println(
      "--- VELOCIDAD ---"
    );


    Serial.print(
      "RPM filtradas     : "
    );

    Serial.println(
      filteredRPM,
      1
    );


    Serial.print(
      "RPM por conteo    : "
    );

    Serial.println(
      rpmCount,
      1
    );


    Serial.print(
      "RPM por periodo   : "
    );

    Serial.println(
      rpmPeriod,
      1
    );


    Serial.print(
      "RPM FINAL         : "
    );

    Serial.println(
      finalRPM,
      1
    );


    Serial.print(
      "Diferencia        : "
    );

    Serial.print(
      rpmDifferencePercent,
      2
    );

    Serial.println(" %");


    Serial.println();
    Serial.println(
      "--- GALGA + PALANCA ---"
    );


    Serial.print(
      "RAW               : "
    );

    Serial.println(
      raw,
      2
    );


    Serial.print(
      "CeroFinal         : "
    );

    Serial.println(
      zeroRaw,
      2
    );


    Serial.print(
      "Masa SIN filtro   : "
    );

    Serial.print(
      massUnfiltered_g,
      4
    );

    Serial.println(" g");


    Serial.print(
      "Masa KALMAN       : "
    );

    Serial.print(
      massFiltered_g,
      4
    );

    Serial.println(" g");


    Serial.print(
      "Fuerza GALGA      : "
    );

    Serial.print(
      forceLoadCell_N,
      6
    );

    Serial.println(" N");


    Serial.print(
      "Fuerza IMAN       : "
    );

    Serial.print(
      forceMagnet_N,
      6
    );

    Serial.println(" N");


    Serial.print(
      "Torque            : "
    );

    Serial.print(
      torque_Nm,
      8
    );

    Serial.println(" N*m");


    Serial.println();
    Serial.println(
      "--- SENAL VELOCIDAD ---"
    );


    Serial.print(
      "Frecuencia conteo : "
    );

    Serial.print(
      pulseFrequencyCount,
      2
    );

    Serial.println(" Hz");


    Serial.print(
      "Frecuencia periodo: "
    );

    Serial.print(
      pulseFrequencyPeriod,
      2
    );

    Serial.println(" Hz");


    Serial.print(
      "Transiciones/s    : "
    );

    Serial.println(
      transitionsPerSecond,
      2
    );


    Serial.print(
      "Transiciones      : "
    );

    Serial.println(edges);


    Serial.print(
      "Flancos subida    : "
    );

    Serial.println(rises);


    Serial.print(
      "Flancos bajada    : "
    );

    Serial.println(falls);


    Serial.print(
      "Periodo           : "
    );

    Serial.print(period);

    Serial.println(" us");


    Serial.print(
      "Duty cycle        : "
    );

    Serial.print(
      duty,
      2
    );

    Serial.println(" %");


    // ========================================================
    // MQTT
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
      "\"kalman_gain\":%.5f,"

      "\"force_N\":%.6f,"
      "\"force_magnet_N\":%.6f,"
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


      sequence,
      nowMs,
      elapsedMs,

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

      massUnfiltered_g,
      massFiltered_g,
      kalmanGain,

      forceLoadCell_N,
      forceMagnet_N,
      torque_Nm,

      MAGNET_TO_PIVOT_M,
      PIVOT_TO_LOADCELL_M,

      hxOK
        ? "true"
        : "false",

      pulseFrequencyCount,
      pulseFrequencyPeriod,
      transitionsPerSecond,

      edges,
      rises,
      falls,

      period,
      highUs,
      lowUs,

      duty
    );


    bool enviado =
      mqttClient.publish(
        MQTT_TOPIC,
        mensaje
      );


    Serial.println();

    Serial.print(
      "MQTT              : "
    );


    if (enviado)
    {
      Serial.println("OK");
    }

    else
    {
      Serial.println("ERROR");
    }


    Serial.println(
      "======================================================"
    );
  }
}