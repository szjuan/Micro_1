#include "HX711.h"

// ============================================================
//        BANCO DE PRUEBAS MOTOR BLDC
//
//   VELOCIDAD + GALGA + FUERZA + TORQUE
//
// ESP32
//
// HX711:
//   DT  -> GPIO 1
//   SCK -> GPIO 2
//
// Velocidad ZS-X11H:
//   S / SC -> divisor -> GPIO 8
//
// ============================================================


// ============================================================
// PINES
// ============================================================

// HX711
#define HX711_DOUT  1
#define HX711_SCK   2

// Señal de velocidad
#define SPEED_PIN   8


HX711 scale;


// ============================================================
// CALIBRACION DE LA GALGA
// ============================================================

// Pendiente obtenida experimentalmente:
//
// Masa [g] = 0.0006589883 * (RAW - CeroFinal)
//
const double MASS_SLOPE = 0.0006589883;

// Gravedad
const double GRAVITY = 9.80665;

// Brazo de reacción:
// 20 mm = 0.020 m
const double TORQUE_ARM_M = 0.020;

// Número de muestras para hacer la tara inicial
const int ZERO_SAMPLES = 30;

// Número de muestras RAW durante funcionamiento
// Menor para no ralentizar demasiado el programa
const int HX711_RUN_SAMPLES = 5;


// Cero obtenido al inicio
double zeroRaw = 0.0;


// ============================================================
// CALIBRACION DE VELOCIDAD
// ============================================================

// Confirmado experimentalmente:
//
// 12 transiciones / revolución
//  6 pulsos completos / revolución
//
const float TRANSITIONS_PER_REV = 12.0;
const float PULSES_PER_REV      = 6.0;


// Mostrar resultados cada 500 ms
const uint32_t REPORT_INTERVAL_MS = 500;


// Si no hay transición durante 1 segundo,
// consideramos el motor detenido
const uint32_t SIGNAL_TIMEOUT_US = 1000000;


// Filtro exponencial
const float FILTER_ALPHA = 0.25;


// Para aceptar que conteo y periodo concuerdan:
//
// diferencia <= 2 %
const float RPM_AGREEMENT_PERCENT = 2.0;


// ============================================================
// VARIABLES DE INTERRUPCION DE VELOCIDAD
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
// VARIABLES GENERALES DE VELOCIDAD
// ============================================================

float filteredRPM = 0.0;
float finalRPM = 0.0;

bool filterInitialized = false;


// ============================================================
// INTERRUPCION DE VELOCIDAD
// ============================================================

void IRAM_ATTR speedISR()
{
  uint32_t now = micros();

  bool state = digitalRead(SPEED_PIN);

  edgeCount++;

  lastEdgeUs = now;


  // ----------------------------------------------------------
  // FLANCO ASCENDENTE
  // ----------------------------------------------------------

  if (state == HIGH)
  {
    risingCount++;

    // Periodo entre dos flancos ascendentes
    if (lastRiseUs != 0)
    {
      risePeriodUs = now - lastRiseUs;
    }

    // Tiempo LOW
    if (lastFallUs != 0)
    {
      lowTimeUs = now - lastFallUs;
    }

    lastRiseUs = now;
  }


  // ----------------------------------------------------------
  // FLANCO DESCENDENTE
  // ----------------------------------------------------------

  else
  {
    fallingCount++;

    // Tiempo HIGH
    if (lastRiseUs != 0)
    {
      highTimeUs = now - lastRiseUs;
    }

    lastFallUs = now;
  }
}


// ============================================================
// ESPERAR HX711
// ============================================================

bool waitForHX711(unsigned long timeoutMs)
{
  unsigned long start = millis();

  while (!scale.is_ready())
  {
    if (millis() - start >= timeoutMs)
    {
      return false;
    }

    delay(5);
  }

  return true;
}


// ============================================================
// LEER PROMEDIO RAW
// ============================================================

bool readAverageRaw(int samples, double &result)
{
  double sum = 0.0;

  for (int i = 0; i < samples; i++)
  {
    if (!waitForHX711(2000))
    {
      return false;
    }

    long raw = scale.read();

    sum += raw;
  }

  result = sum / samples;

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

  char option = Serial.read();

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

  String line = Serial.readStringUntil('\n');

  line.trim();

  return line.toDouble();
}


// ============================================================
// CONFIGURAR CERO DE GALGA
// ============================================================

void configureLoadCellZero()
{
  Serial.println();
  Serial.println("======================================================");
  Serial.println("        CONFIGURACION INICIAL DE LA GALGA");
  Serial.println("======================================================");
  Serial.println();

  Serial.println("Pendiente de calibracion guardada:");
  Serial.print(MASS_SLOPE, 10);
  Serial.println(" g/count");
  Serial.println();

  Serial.println("NO es necesario repetir la calibracion con masas.");
  Serial.println();

  Serial.println("Ahora necesitamos CeroFinal.");
  Serial.println();
  Serial.println("La galga debe estar:");
  Serial.println("  - SIN plataforma de calibracion.");
  Serial.println("  - Instalada en el montaje final.");
  Serial.println("  - SIN fuerza aplicada.");
  Serial.println("  - Con el motor detenido.");
  Serial.println();

  Serial.println("Seleccione:");
  Serial.println();
  Serial.println("1 = Hacer tara / cero automaticamente");
  Serial.println("2 = Introducir CeroFinal manualmente");
  Serial.println();

  char option = readOption();


  // ==========================================================
  // OPCION 1: TARA AUTOMATICA
  // ==========================================================

  if (option == '1')
  {
    Serial.println();
    Serial.println("TARA AUTOMATICA");
    Serial.println("------------------------------");
    Serial.println();

    Serial.println("No toque el montaje.");
    Serial.println("Midiendo cero...");
    Serial.println();

    delay(1500);


    if (!readAverageRaw(ZERO_SAMPLES, zeroRaw))
    {
      Serial.println("ERROR: HX711 no responde.");

      while (1)
      {
        delay(1000);
      }
    }


    Serial.print("CeroFinal = ");
    Serial.println(zeroRaw, 2);

    Serial.println();
    Serial.println("Tara realizada correctamente.");
  }


  // ==========================================================
  // OPCION 2: CERO MANUAL
  // ==========================================================

  else if (option == '2')
  {
    Serial.println();
    Serial.println("Introduzca el valor RAW de CeroFinal:");
    Serial.println();

    zeroRaw = readSerialNumber();

    Serial.println();

    Serial.print("CeroFinal introducido = ");
    Serial.println(zeroRaw, 2);
  }


  // ==========================================================
  // OPCION INVALIDA
  // ==========================================================

  else
  {
    Serial.println();
    Serial.println("Opcion invalida.");
    Serial.println("Se realizara tara automaticamente.");
    Serial.println();

    delay(1500);

    if (!readAverageRaw(ZERO_SAMPLES, zeroRaw))
    {
      Serial.println("ERROR: HX711 no responde.");

      while (1)
      {
        delay(1000);
      }
    }

    Serial.print("CeroFinal = ");
    Serial.println(zeroRaw, 2);
  }


  Serial.println();
  Serial.println("Ecuacion que se utilizara:");
  Serial.println();

  Serial.print("Masa[g] = ");
  Serial.print(MASS_SLOPE, 10);
  Serial.println(" * (RAW - CeroFinal)");

  Serial.println();

  Serial.println("Configuracion de galga terminada.");
  Serial.println("======================================================");
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
  Serial.println("      BANCO DE PRUEBAS MOTOR BLDC");
  Serial.println("      VELOCIDAD + FUERZA + TORQUE");
  Serial.println("======================================================");
  Serial.println();


  // ==========================================================
  // INICIALIZAR HX711
  // ==========================================================

  scale.begin(HX711_DOUT, HX711_SCK);


  Serial.println("Buscando HX711...");


  if (!waitForHX711(5000))
  {
    Serial.println();
    Serial.println("ERROR: HX711 no encontrado.");
    Serial.println();

    Serial.println("Revise:");
    Serial.println("DT  -> GPIO 1");
    Serial.println("SCK -> GPIO 2");
    Serial.println("VCC");
    Serial.println("GND");

    while (1)
    {
      delay(1000);
    }
  }


  Serial.println("HX711 encontrado correctamente.");


  // ==========================================================
  // CONFIGURAR CERO DE GALGA
  // ==========================================================

  configureLoadCellZero();


  // ==========================================================
  // CONFIGURAR TACOMETRO
  // ==========================================================

  Serial.println();
  Serial.println("Configurando sensor de velocidad...");

  pinMode(SPEED_PIN, INPUT);


  attachInterrupt(
    digitalPinToInterrupt(SPEED_PIN),
    speedISR,
    CHANGE
  );


  Serial.println("Sensor de velocidad listo.");
  Serial.println();

  Serial.println("Calibracion de velocidad:");
  Serial.println("12 transiciones / vuelta");
  Serial.println(" 6 pulsos / vuelta");

  Serial.println();

  Serial.println("======================================================");
  Serial.println("           INICIANDO MEDICION");
  Serial.println("======================================================");
  Serial.println();

  Serial.println("Puede iniciar el motor.");
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  static uint32_t previousReportMs = millis();


  // ==========================================================
  // REPORTE CADA 500 ms
  // ==========================================================

  if (millis() - previousReportMs >= REPORT_INTERVAL_MS)
  {
    uint32_t nowMs = millis();

    uint32_t elapsedMs =
      nowMs - previousReportMs;

    previousReportMs = nowMs;


    float elapsedSeconds =
      elapsedMs / 1000.0;


    // ========================================================
    // COPIAR DATOS DE INTERRUPCION
    // ========================================================

    noInterrupts();

    uint32_t edges = edgeCount;

    uint32_t rises = risingCount;

    uint32_t falls = fallingCount;

    uint32_t lastEdge = lastEdgeUs;

    uint32_t period = risePeriodUs;

    uint32_t highUs = highTimeUs;

    uint32_t lowUs = lowTimeUs;


    // Reiniciar contadores
    edgeCount = 0;

    risingCount = 0;

    fallingCount = 0;

    interrupts();


    // ========================================================
    // DETECTAR SI MOTOR ESTA GIRANDO
    // ========================================================

    bool signalActive = false;


    if (lastEdge != 0)
    {
      uint32_t timeSinceLastEdge =
        micros() - lastEdge;

      if (timeSinceLastEdge < SIGNAL_TIMEOUT_US)
      {
        signalActive = true;
      }
    }


    // ========================================================
    // VARIABLES DE VELOCIDAD
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
    // MOTOR EN MOVIMIENTO
    // ========================================================

    if (signalActive)
    {
      // Transiciones por segundo
      transitionsPerSecond =
        edges / elapsedSeconds;


      // Frecuencia mediante conteo de flancos ascendentes
      pulseFrequencyCount =
        rises / elapsedSeconds;


      // ------------------------------------------------------
      // RPM POR CONTEO
      // ------------------------------------------------------

      rpmCount =
        (
          transitionsPerSecond * 60.0
        )
        /
        TRANSITIONS_PER_REV;


      // ------------------------------------------------------
      // RPM POR PERIODO
      // ------------------------------------------------------

      if (period > 0)
      {
        pulseFrequencyPeriod =
          1000000.0 / period;

        rpmPeriod =
          (
            pulseFrequencyPeriod * 60.0
          )
          /
          PULSES_PER_REV;
      }


      // ------------------------------------------------------
      // DUTY CYCLE
      // ------------------------------------------------------

      uint32_t pulseTime =
        highUs + lowUs;

      if (pulseTime > 0)
      {
        duty =
          100.0 *
          ((float)highUs / pulseTime);
      }


      // ------------------------------------------------------
      // RPM FILTRADA
      // ------------------------------------------------------

      if (!filterInitialized)
      {
        filteredRPM = rpmCount;

        filterInitialized = true;
      }

      else
      {
        filteredRPM =
          FILTER_ALPHA * rpmCount
          +
          (1.0 - FILTER_ALPHA) * filteredRPM;
      }


      // ======================================================
      // COMPARAR CONTEO Y PERIODO
      // ======================================================

      if (
        rpmCount > 0.0 &&
        rpmPeriod > 0.0
      )
      {
        float difference =
          fabs(rpmCount - rpmPeriod);


        rpmDifferencePercent =
          100.0 *
          difference /
          rpmCount;


        // ----------------------------------------------------
        // VELOCIDAD FINAL
        //
        // SOLO se actualiza si los dos métodos concuerdan
        // ----------------------------------------------------

        if (
          rpmDifferencePercent <=
          RPM_AGREEMENT_PERCENT
        )
        {
          rpmAgreement = true;


          // Promedio de los dos métodos
          finalRPM =
            (rpmCount + rpmPeriod) / 2.0;
        }
      }
    }


    // ========================================================
    // MOTOR DETENIDO
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
    // LEER GALGA
    // ========================================================

    double raw = 0.0;

    bool hxOK =
      readAverageRaw(
        HX711_RUN_SAMPLES,
        raw
      );


    double mass_g = 0.0;

    double force_N = 0.0;

    double torque_Nm = 0.0;


    if (hxOK)
    {
      // ------------------------------------------------------
      // MASA
      // ------------------------------------------------------

      mass_g =
        MASS_SLOPE *
        (raw - zeroRaw);


      // ------------------------------------------------------
      // FUERZA
      //
      // gramos -> kg -> N
      // ------------------------------------------------------

      force_N =
        (mass_g / 1000.0) *
        GRAVITY;


      // ------------------------------------------------------
      // TORQUE
      //
      // T = F * r
      //
      // r = 0.020 m
      // ------------------------------------------------------

      torque_Nm =
        force_N *
        TORQUE_ARM_M;
    }


    // ========================================================
    // MOSTRAR RESULTADOS
    // ========================================================

    Serial.println();
    Serial.println("======================================================");


    // ========================================================
    // VELOCIDAD
    // ========================================================

    Serial.println("--- VELOCIDAD ---");


    Serial.print("RPM filtradas     : ");
    Serial.print(filteredRPM, 1);
    Serial.println(" RPM");


    Serial.print("RPM por conteo    : ");
    Serial.print(rpmCount, 1);
    Serial.println(" RPM");


    Serial.print("RPM por periodo   : ");
    Serial.print(rpmPeriod, 1);
    Serial.println(" RPM");


    Serial.print("RPM FINAL         : ");
    Serial.print(finalRPM, 1);
    Serial.println(" RPM");


    Serial.print("Diferencia        : ");
    Serial.print(rpmDifferencePercent, 2);
    Serial.println(" %");


    Serial.print("Concordancia      : ");

    if (!signalActive)
    {
      Serial.println("MOTOR DETENIDO");
    }

    else if (rpmAgreement)
    {
      Serial.println("OK - RPM FINAL ACTUALIZADA");
    }

    else
    {
      Serial.println("NO - RPM FINAL CONSERVADA");
    }


    // ========================================================
    // GALGA
    // ========================================================

    Serial.println();
    Serial.println("--- GALGA ---");


    if (hxOK)
    {
      Serial.print("RAW               : ");
      Serial.println(raw, 2);


      Serial.print("CeroFinal         : ");
      Serial.println(zeroRaw, 2);


      Serial.print("Masa              : ");
      Serial.print(mass_g, 2);
      Serial.println(" g");


      Serial.print("Fuerza            : ");
      Serial.print(force_N, 5);
      Serial.println(" N");


      Serial.print("TORQUE            : ");
      Serial.print(torque_Nm, 6);
      Serial.println(" N*m");
    }

    else
    {
      Serial.println("ERROR: HX711 no responde.");
    }


    // ========================================================
    // INFORMACION ADICIONAL DE VELOCIDAD
    // ========================================================

    Serial.println();
    Serial.println("--- SENAL VELOCIDAD ---");


    Serial.print("Frecuencia conteo : ");
    Serial.print(pulseFrequencyCount, 2);
    Serial.println(" Hz");


    Serial.print("Frecuencia periodo: ");
    Serial.print(pulseFrequencyPeriod, 2);
    Serial.println(" Hz");


    Serial.print("Transiciones      : ");
    Serial.println(edges);


    Serial.print("Flancos subida    : ");
    Serial.println(rises);


    Serial.print("Flancos bajada    : ");
    Serial.println(falls);


    Serial.print("Periodo           : ");
    Serial.print(period);
    Serial.println(" us");


    Serial.print("Duty cycle        : ");
    Serial.print(duty, 2);
    Serial.println(" %");


    Serial.println("======================================================");
  }
}