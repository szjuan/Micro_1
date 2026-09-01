// ============================================================
// TACOMETRO BLDC - ZS-X11H
// ESP32-S3
//
// Entrada:
//   Pin S / SC del ZS-X11H
//
// Conexion:
//   S ---- 10k ----+---- GPIO 8
//                  |
//                 20k
//                  |
//   GND ------------+---- GND ESP32
//
// CALIBRACION EXPERIMENTAL:
//
//   12 transiciones / revolucion
//    6 pulsos / revolucion
//
// ============================================================


// ============================================================
// CONFIGURACION
// ============================================================

const uint8_t SPEED_PIN = 8;

const float TRANSITIONS_PER_REV = 12.0;
const float PULSES_PER_REV      = 6.0;

// Reporte cada 500 ms
const uint32_t REPORT_INTERVAL_MS = 500;

// Sin pulsos durante 1 segundo = motor detenido
const uint32_t SIGNAL_TIMEOUT_US = 1000000;

// Filtro exponencial
const float FILTER_ALPHA = 0.25;


// ============================================================
// CRITERIO PARA VELOCIDAD FINAL
// ============================================================

// Conteo y periodo deben diferir como maximo 2 %
const float AGREEMENT_TOLERANCE_PERCENT = 2.0;


// ============================================================
// VARIABLES DE INTERRUPCION
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
// VARIABLES GENERALES
// ============================================================

float filteredRPM = 0.0;
bool filterInitialized = false;


// ------------------------------------------------------------
// NUEVA VARIABLE:
// Guarda la ultima velocidad confirmada
// ------------------------------------------------------------

float finalRPM = 0.0;


// ============================================================
// INTERRUPCION
// ============================================================

void IRAM_ATTR speedISR()
{
  uint32_t now = micros();

  bool state = digitalRead(SPEED_PIN);

  edgeCount++;
  lastEdgeUs = now;


  // ==========================================================
  // FLANCO ASCENDENTE
  // ==========================================================

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


  // ==========================================================
  // FLANCO DESCENDENTE
  // ==========================================================

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
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(2000);


  Serial.println();
  Serial.println("======================================================");
  Serial.println("            TACOMETRO BLDC - ZS-X11H");
  Serial.println("            ESP32-S3 - GPIO 8");
  Serial.println("======================================================");

  Serial.println();

  Serial.println("Calibracion:");
  Serial.println("  12 transiciones / vuelta");
  Serial.println("   6 pulsos / vuelta");

  Serial.println();

  Serial.print("Tolerancia concordancia: ");
  Serial.print(AGREEMENT_TOLERANCE_PERCENT);
  Serial.println(" %");

  Serial.println();

  Serial.println("La VELOCIDAD FINAL solo cambia cuando:");
  Serial.println("RPM conteo ~= RPM periodo");

  Serial.println();


  pinMode(SPEED_PIN, INPUT);


  Serial.print("Estado inicial GPIO8: ");

  if (digitalRead(SPEED_PIN))
  {
    Serial.println("HIGH");
  }
  else
  {
    Serial.println("LOW");
  }


  attachInterrupt(
    digitalPinToInterrupt(SPEED_PIN),
    speedISR,
    CHANGE
  );


  Serial.println();
  Serial.println("Interrupcion activada.");
  Serial.println("Esperando movimiento del motor...");
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  static uint32_t previousReportMs = millis();


  if (millis() - previousReportMs >= REPORT_INTERVAL_MS)
  {
    uint32_t nowMs = millis();

    uint32_t elapsedMs =
      nowMs - previousReportMs;

    previousReportMs = nowMs;


    float elapsedSeconds =
      elapsedMs / 1000.0;


    // ========================================================
    // COPIAR VARIABLES DE INTERRUPCION
    // ========================================================

    noInterrupts();

    uint32_t edges = edgeCount;
    uint32_t rises = risingCount;
    uint32_t falls = fallingCount;

    uint32_t lastEdge = lastEdgeUs;

    uint32_t period = risePeriodUs;

    uint32_t highUs = highTimeUs;
    uint32_t lowUs = lowTimeUs;


    edgeCount = 0;
    risingCount = 0;
    fallingCount = 0;

    interrupts();


    // ========================================================
    // DETECTAR SENAL
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
    // VARIABLES CALCULADAS
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
      // ------------------------------------------------------
      // TRANSICIONES / SEGUNDO
      // ------------------------------------------------------

      transitionsPerSecond =
        edges / elapsedSeconds;


      // ------------------------------------------------------
      // FRECUENCIA POR CONTEO
      // ------------------------------------------------------

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


      // ======================================================
      // FRECUENCIA Y RPM POR PERIODO
      // ======================================================

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


      // ======================================================
      // DUTY CYCLE
      // ======================================================

      uint32_t pulseTime =
        highUs + lowUs;


      if (pulseTime > 0)
      {
        duty =
          100.0 *
          ((float)highUs / pulseTime);
      }


      // ======================================================
      // RPM FILTRADAS
      // ======================================================

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
      // COMPARAR LOS DOS METODOS
      // ======================================================

      if (rpmCount > 0.0 && rpmPeriod > 0.0)
      {
        float difference =
          fabs(rpmCount - rpmPeriod);


        // Usamos el promedio como referencia
        float averageRPM =
          (rpmCount + rpmPeriod) / 2.0;


        rpmDifferencePercent =
          (
            difference /
            averageRPM
          )
          * 100.0;


        // ====================================================
        // ¿CONCUERDAN?
        // ====================================================

        if (
          rpmDifferencePercent
          <= AGREEMENT_TOLERANCE_PERCENT
        )
        {
          rpmAgreement = true;


          // ================================================
          // NUEVA VELOCIDAD FINAL CONFIRMADA
          // ================================================

          finalRPM =
            (
              rpmCount +
              rpmPeriod
            )
            /
            2.0;
        }

        else
        {
          rpmAgreement = false;


          // IMPORTANTE:
          //
          // NO modificamos finalRPM.
          //
          // Conservamos la ultima velocidad
          // que habia sido confirmada.
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

      duty = 0.0;

      filteredRPM = 0.0;

      filterInitialized = false;

      rpmDifferencePercent = 0.0;

      rpmAgreement = false;


      // Al detenerse realmente el motor:
      finalRPM = 0.0;


      // Evitar datos viejos
      period = 0;

      highUs = 0;

      lowUs = 0;
    }


    // ========================================================
    // MOSTRAR LOS CUATRO VALORES DE VELOCIDAD
    // ========================================================

    Serial.println();
    Serial.println("======================================================");
    Serial.println("                    VELOCIDAD");
    Serial.println("======================================================");


    Serial.print("RPM filtradas      : ");
    Serial.print(filteredRPM, 1);
    Serial.println(" RPM");


    Serial.print("RPM por conteo     : ");
    Serial.print(rpmCount, 1);
    Serial.println(" RPM");


    Serial.print("RPM por periodo    : ");
    Serial.print(rpmPeriod, 1);
    Serial.println(" RPM");


    Serial.println();


    // --------------------------------------------------------
    // VELOCIDAD FINAL
    // --------------------------------------------------------

    Serial.print(">>> VELOCIDAD FINAL: ");
    Serial.print(finalRPM, 1);
    Serial.println(" RPM <<<");


    Serial.println();


    // ========================================================
    // ESTADO DE LA VALIDACION
    // ========================================================

    if (!signalActive)
    {
      Serial.println(
        "ESTADO VELOCIDAD: MOTOR DETENIDO"
      );
    }

    else if (rpmAgreement)
    {
      Serial.println(
        "ESTADO VELOCIDAD: VALIDADA"
      );

      Serial.println(
        "Conteo y periodo CONCUERDAN."
      );
    }

    else
    {
      Serial.println(
        "ESTADO VELOCIDAD: ESPERANDO CONCORDANCIA"
      );

      Serial.println(
        "Se conserva la ultima VELOCIDAD FINAL valida."
      );
    }


    Serial.print("Diferencia entre metodos: ");
    Serial.print(rpmDifferencePercent, 2);
    Serial.println(" %");


    Serial.print("Tolerancia permitida     : ");
    Serial.print(AGREEMENT_TOLERANCE_PERCENT, 2);
    Serial.println(" %");


    // ========================================================
    // FRECUENCIA
    // ========================================================

    Serial.println();
    Serial.println("--- FRECUENCIA ---");


    Serial.print("Frecuencia conteo : ");
    Serial.print(pulseFrequencyCount, 2);
    Serial.println(" Hz");


    Serial.print("Frecuencia periodo: ");
    Serial.print(pulseFrequencyPeriod, 2);
    Serial.println(" Hz");


    Serial.print("Transiciones/s    : ");
    Serial.print(transitionsPerSecond, 2);
    Serial.println(" trans/s");


    // ========================================================
    // SENAL
    // ========================================================

    Serial.println();
    Serial.println("--- SENAL S / SC ---");


    Serial.print("GPIO8 actual      : ");

    if (digitalRead(SPEED_PIN))
    {
      Serial.println("HIGH");
    }
    else
    {
      Serial.println("LOW");
    }


    Serial.print("Flancos subida    : ");
    Serial.println(rises);


    Serial.print("Flancos bajada    : ");
    Serial.println(falls);


    Serial.print("Transiciones      : ");
    Serial.println(edges);


    Serial.print("Periodo           : ");
    Serial.print(period);
    Serial.println(" us");


    Serial.print("Tiempo HIGH       : ");
    Serial.print(highUs);
    Serial.println(" us");


    Serial.print("Tiempo LOW        : ");
    Serial.print(lowUs);
    Serial.println(" us");


    Serial.print("Duty cycle        : ");
    Serial.print(duty, 2);
    Serial.println(" %");


    // ========================================================
    // DIAGNOSTICO GENERAL
    // ========================================================

    Serial.println();
    Serial.println("--- DIAGNOSTICO ---");


    if (!signalActive)
    {
      Serial.println(
        "ESTADO: MOTOR DETENIDO / SIN SENAL"
      );
    }

    else
    {
      Serial.println(
        "ESTADO: SENAL DE VELOCIDAD DETECTADA"
      );


      int edgeDifference =
        abs(
          (int)rises -
          (int)falls
        );


      if (edgeDifference <= 2)
      {
        Serial.println(
          "OK: flancos consistentes."
        );
      }

      else
      {
        Serial.println(
          "AVISO: diferencia entre flancos."
        );
      }


      if (
        duty >= 40.0 &&
        duty <= 60.0
      )
      {
        Serial.println(
          "OK: duty cercano al 50%."
        );
      }

      else
      {
        Serial.println(
          "AVISO: duty alejado del 50%."
        );
      }
    }


    Serial.println("======================================================");
  }
}