# Resumen del Sistema — Banco de Pruebas Freno Magnético Motor DC

## Arquitectura General

El sistema tiene tres bloques físicos conectados en cadena:

```
ESP32 → (WiFi / USB) → HiveMQ Cloud → Python Dashboard
```

La ESP32 es el nodo de adquisición. El computador con Python es el nodo de
procesamiento y visualización. La comunicación puede ser directa por USB Serial
o inalámbrica vía MQTT sobre TLS a través de un broker HiveMQ Cloud.

---

## ESP32 (`ESP32.ino`)

### Sensores y periféricos conectados
- **HX711 + celda de carga:** mide la fuerza en la galga extensométrica (pines GPIO 1 y 2).
- **Tacómetro óptico/hall:** genera pulsos por revolución (pin GPIO 8), capturado con interrupciones de hardware.

### Flujo al encender
1. Inicializa comunicación serial a 115 200 baud.
2. Verifica que el HX711 responda (timeout 5 s).
3. Realiza **tara automática**: promedia 30 lecturas RAW del HX711 en reposo y guarda el valor `zeroRaw`.
4. Configura la interrupción del tacómetro (flancos de subida y bajada).
5. Si `ENABLE_MQTT = true`: activa TLS (`setInsecure()`), registra el callback MQTT, conecta al broker HiveMQ Cloud (puerto 8883, usuario/contraseña) y lanza la reconexión WiFi no bloqueante.

### Ciclo de medición (cada 500 ms)
- Copia atómica de contadores de interrupción con `noInterrupts()`.
- Calcula RPM por **dos métodos**:
  - Conteo de flancos en el intervalo.
  - Período entre flancos ascendentes consecutivos.
- Aplica filtro **EMA** (α = 0.25) sobre la RPM.
- Solo actualiza la **RPM Final** cuando ambos métodos concuerdan dentro del 2 %.
- Lee 5 muestras del HX711 y las promedia.
- Aplica **filtro de Kalman escalar** (Q = 0.05, R = 1.00) sobre la masa.
- Calcula fuerza y torque usando la geometría de palanca:
  - Imán → pivote = 6 cm | Pivote → galga = 4 cm
  - `Torque [N·m] = F_galga [N] × 0.04 m`
- Empaqueta todos los valores en un **JSON de ~30 campos**.
- Publica el JSON simultáneamente:
  - Por **USB Serial** con el prefijo `DATA:`.
  - Por **MQTT** al topic `micro1/motor1/telemetry`.

### Re-calibración en caliente
- Mensaje MQTT en `micro1/motor1/cmd` con `{"cmd":"calibrate"}` → activa el flag `pendingCalibration`.
- Carácter `'C'` recibido por Serial → mismo efecto.
- Al inicio del siguiente `loop()` se ejecuta `realizarTaraAutomatica()`: promedia 30 nuevas lecturas RAW y reinicia el filtro de Kalman. El motor puede seguir girando.

### JSON transmitido (campos principales)
| Campo | Descripción |
|---|---|
| `seq` | Número de secuencia |
| `time_ms` | Timestamp desde boot |
| `rpm_final` | RPM validada (método dual) |
| `rpm_filtered` | RPM filtrada (EMA) |
| `rpm_count` / `rpm_period` | RPM por cada método |
| `mass_g` | Masa Kalman [g] |
| `force_N` | Fuerza en galga [N] |
| `torque_Nm` | Torque de carga [N·m] |
| `raw` / `zero_raw` | Lecturas crudas HX711 |
| `kalman_gain` | Ganancia actual del filtro |
| `duty_percent` | Duty cycle de la señal del tacómetro |
| `hx_ok` | Estado del HX711 |

---

## Dashboard Python (`Dashboard.py`)

Aplicación de escritorio en **PyQt6** con tres pantallas y tema oscuro de ingeniería.

### Pantalla 0 — Bienvenida
Muestra el nombre del proyecto, materia y autores del equipo. El botón **"Iniciar Dashboard"** avanza a la siguiente pantalla.

### Pantalla 1 — Configuración y Calibración
El usuario elige el transporte de datos:

**Modo USB Serial**
- Abre el puerto COM de la ESP32.
- Espera hasta 20 s a que aparezca el menú de tara en el serial.
- Envía `"1\n"` → la ESP32 realiza la tara automática.
- Arranca un hilo background que lee continuamente las líneas `DATA:{json}`.

**Modo MQTT · HiveMQ**
- Lanza un hilo que conecta al broker HiveMQ Cloud (TLS 8883).
- Publica `{"cmd":"calibrate"}` al topic `micro1/motor1/cmd`.
- La ESP32 recibe el comando y ejecuta la tara.
- El hilo permanece en `loop_forever()` recibiendo mensajes del topic `micro1/motor1/telemetry`.

Cuando la calibración termina, se habilita el botón **"Ver Gráficas →"**.

### Pantalla 2 — Gráficas en Tiempo Real
- Dos gráficas `pyqtgraph` con fondo oscuro (`#0D1117`), refresco cada 100 ms, historial de 300 puntos.
- **Velocidad Angular vs Tiempo** — curva cian con área rellena.
- **Torque de Carga vs Tiempo** — curva naranja-rojo con área rellena.
- Valores numéricos actuales en grande sobre cada gráfica.
- Estado de conexión visible en el header.

### Lógica de filtrado del torque
El torque graficado **no se actualiza con cada muestra**. Solo cambia cuando la RPM Final varía en más de 5 RPM respecto al último punto registrado. Esto elimina el ruido de la galga en régimen estacionario y produce una curva limpia y representativa.

### Hilos de ejecución
| Hilo | Función |
|---|---|
| Hilo principal Qt | GUI + timer de refresco |
| Hilo background Serial o MQTT | Lectura/recepción de datos |

Los buffers compartidos están protegidos con `threading.Lock`.

---

## Topics MQTT

| Topic | Dirección | Contenido |
|---|---|---|
| `micro1/motor1/telemetry` | ESP32 → Dashboard | JSON con todas las variables (30 campos, cada 500 ms) |
| `micro1/motor1/cmd` | Dashboard → ESP32 | Comandos: `{"cmd":"calibrate"}` |

## Broker
- **Proveedor:** HiveMQ Cloud (plan gratuito)
- **Host:** `0e44beba4fc7422cb74bc8bbdcc67b2f.s1.eu.hivemq.cloud`
- **Puerto:** 8883
- **Seguridad:** TLS + autenticación usuario/contraseña
