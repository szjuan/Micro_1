#include "HX711.h"

// =====================================================
// PINES HX711
// =====================================================
#define HX711_DOUT  1
#define HX711_SCK   2

HX711 scale;

// Número de lecturas que se promedian en cada punto
const int N_SAMPLES = 30;

// Máximo número de puntos de calibración
const int MAX_POINTS = 10;

double rawValues[MAX_POINTS];
double massValues[MAX_POINTS];

int nPoints = 0;


// =====================================================
// FUNCIÓN: ESPERAR A QUE EL HX711 ESTÉ LISTO
// =====================================================
bool waitForHX711(unsigned long timeoutMs)
{
  unsigned long start = millis();

  while (!scale.is_ready())
  {
    if (millis() - start >= timeoutMs)
    {
      return false;
    }

    delay(10);
  }

  return true;
}


// =====================================================
// FUNCIÓN: PROMEDIO DE LECTURAS RAW
// =====================================================
double readAverageRaw(int samples)
{
  double sum = 0;

  for (int i = 0; i < samples; i++)
  {
    // Esperar hasta que haya una lectura disponible
    if (!waitForHX711(2000))
    {
      Serial.println("ERROR: HX711 dejo de responder.");
      return 0;
    }

    long lectura = scale.read();

    // Mostrar cada lectura individual
    Serial.print("Lectura ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(samples);
    Serial.print(" = ");
    Serial.println(lectura);

    sum += lectura;

    delay(50);
  }

  return sum / samples;
}


// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);

  delay(2000);

  Serial.println();
  Serial.println("==========================================");
  Serial.println("     CALIBRACION DE CELDA DE CARGA");
  Serial.println("==========================================");
  Serial.println();

  // Inicializar HX711
  scale.begin(HX711_DOUT, HX711_SCK);

  Serial.println("Esperando HX711...");

  // Esperar hasta 5 segundos
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
  Serial.println();

  // ---------------------------------------------------
  // PASO 1:
  // Plataforma instalada y sin masas adicionales
  // ---------------------------------------------------

  Serial.println("==========================================");
  Serial.println("PASO 1 - CERO DE CALIBRACION");
  Serial.println("==========================================");
  Serial.println();

  Serial.println("1. Atornille la plataforma a la celda.");
  Serial.println("2. NO coloque ninguna masa sobre ella.");
  Serial.println("3. Espere a que la estructura este quieta.");
  Serial.println("4. Envie cualquier caracter para continuar.");
  Serial.println();

  while (!Serial.available())
  {
    delay(10);
  }

  // Limpiar buffer serial
  while (Serial.available())
  {
    Serial.read();
  }

  Serial.println();
  Serial.println("Midiendo lectura RAW de referencia...");
  Serial.println();

  double zeroRaw = readAverageRaw(N_SAMPLES);

  Serial.println();
  Serial.println("------------------------------------------");

  Serial.print("Lectura RAW promedio con plataforma = ");
  Serial.println(zeroRaw, 2);

  Serial.println("------------------------------------------");
  Serial.println();

  Serial.println("La plataforma se considera ahora:");
  Serial.println("MASA = 0 g");
  Serial.println();

  // Guardar automáticamente punto de cero
  rawValues[0] = zeroRaw;
  massValues[0] = 0.0;

  nPoints = 1;

  // ---------------------------------------------------
  // PASO 2:
  // Introducción de masas conocidas
  // ---------------------------------------------------

  Serial.println("==========================================");
  Serial.println("AGREGAR MASAS DE CALIBRACION");
  Serial.println("==========================================");
  Serial.println();

  Serial.println("Coloque una masa conocida sobre la plataforma.");
  Serial.println();
  Serial.println("Luego escriba su masa EN GRAMOS.");
  Serial.println();

  Serial.println("Ejemplos:");
  Serial.println("20");
  Serial.println("50");
  Serial.println("100");
  Serial.println("200");

  Serial.println();
  Serial.println("Para terminar escriba:");
  Serial.println("-1");

  Serial.println();
}


// =====================================================
// LOOP
// =====================================================
void loop()
{
  if (Serial.available())
  {
    // Leer masa introducida
    double mass = Serial.parseFloat();

    // Limpiar buffer serial
    while (Serial.available())
    {
      Serial.read();
    }


    // =================================================
    // FINALIZAR CALIBRACIÓN
    // =================================================
    if (mass < 0)
    {
      // Punto 0 + mínimo dos masas conocidas
      if (nPoints < 3)
      {
        Serial.println();
        Serial.println("ERROR:");
        Serial.println("Se necesitan al menos 2 masas conocidas.");
        Serial.println();
        Serial.println("Ingrese otra masa.");
        return;
      }


      // ------------------------------------------------
      // REGRESIÓN LINEAL
      //
      // masa = a * raw + b
      // ------------------------------------------------

      double sumX = 0;
      double sumY = 0;
      double sumXY = 0;
      double sumX2 = 0;

      for (int i = 0; i < nPoints; i++)
      {
        double x = rawValues[i];
        double y = massValues[i];

        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumX2 += x * x;
      }

      double N = nPoints;

      double denominator =
        N * sumX2 - sumX * sumX;

      // Comprobar que la regresión sea posible
      if (denominator == 0)
      {
        Serial.println();
        Serial.println("ERROR DE CALIBRACION.");
        Serial.println("Las lecturas RAW no estan cambiando.");
        Serial.println();
        Serial.println("Revise que:");
        Serial.println("- La celda se deforme al aplicar peso.");
        Serial.println("- A+ y A- esten bien conectados.");
        Serial.println("- Las masas utilizadas sean diferentes.");

        while (1)
        {
          delay(1000);
        }
      }


      double a =
        (N * sumXY - sumX * sumY) /
        denominator;

      double b =
        (sumY - a * sumX) / N;


      // ------------------------------------------------
      // CUENTAS POR GRAMO
      // ------------------------------------------------

      double countsPerGram = 1.0 / a;


      // ------------------------------------------------
      // MOSTRAR RESULTADOS
      // ------------------------------------------------

      Serial.println();
      Serial.println("==========================================");
      Serial.println("       RESULTADO DE CALIBRACION");
      Serial.println("==========================================");
      Serial.println();

      Serial.print("Pendiente a = ");
      Serial.println(a, 10);

      Serial.print("Intercepto b = ");
      Serial.println(b, 6);

      Serial.println();

      Serial.print("Cuentas por gramo = ");
      Serial.println(countsPerGram, 4);

      Serial.println();

      Serial.println("Ecuacion de calibracion:");
      Serial.println();

      Serial.print("Masa [g] = ");
      Serial.print(a, 10);
      Serial.print(" * RAW + ");
      Serial.println(b, 6);


      Serial.println();
      Serial.println("------------------------------------------");
      Serial.println("PUNTOS DE CALIBRACION");
      Serial.println("------------------------------------------");

      Serial.println("RAW promedio , Masa [g]");

      for (int i = 0; i < nPoints; i++)
      {
        Serial.print(rawValues[i], 2);
        Serial.print(" , ");
        Serial.println(massValues[i], 2);
      }


      Serial.println();
      Serial.println("==========================================");
      Serial.println("       MONTAJE FINAL");
      Serial.println("==========================================");
      Serial.println();

      Serial.println("Si posteriormente quita la plataforma:");
      Serial.println();

      Serial.println("1. Haga una nueva lectura de cero.");
      Serial.println("2. NO cambie la pendiente a.");
      Serial.println();
      Serial.println("Use:");

      Serial.println();

      Serial.print("Masa [g] = ");
      Serial.print(a, 10);
      Serial.println(" * (RAW - CeroFinal)");

      Serial.println();
      Serial.println("==========================================");

      // Detener programa después de terminar
      while (1)
      {
        delay(1000);
      }
    }


    // =================================================
    // REGISTRAR NUEVO PUNTO
    // =================================================

    if (nPoints >= MAX_POINTS)
    {
      Serial.println();
      Serial.println("Numero maximo de puntos alcanzado.");
      Serial.println("Escriba -1 para calcular la calibracion.");
      return;
    }


    Serial.println();
    Serial.println("==========================================");

    Serial.print("Masa indicada = ");
    Serial.print(mass, 2);
    Serial.println(" g");

    Serial.println("==========================================");
    Serial.println();

    Serial.println("Mantenga la masa quieta.");
    Serial.println("Esperando estabilizacion...");

    delay(2000);

    Serial.println();
    Serial.println("Midiendo...");
    Serial.println();


    double raw = readAverageRaw(N_SAMPLES);


    rawValues[nPoints] = raw;
    massValues[nPoints] = mass;

    nPoints++;


    Serial.println();
    Serial.println("------------------------------------------");

    Serial.print("Lectura RAW promedio = ");
    Serial.println(raw, 2);

    Serial.print("Masa = ");
    Serial.print(mass, 2);
    Serial.println(" g");

    Serial.println("------------------------------------------");

    Serial.println();
    Serial.println("Punto guardado correctamente.");
    Serial.println();

    Serial.println("Coloque la siguiente masa.");
    Serial.println("Escriba su masa en gramos.");
    Serial.println();

    Serial.println("O escriba -1 para terminar.");
    Serial.println();
  }
}