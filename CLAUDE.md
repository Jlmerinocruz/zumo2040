# Contexto del proyecto para Claude Code

Este repo es el código del equipo para **TE2004B Sistemas Embebidos Avanzados** (Tec de Monterrey, Campus Monterrey). El proyecto del semestre es el **reto John Deere "AgroZumo"**: un tractor autónomo a escala hecho con un Pololu Zumo 2040 que se localiza con una cámara en el techo y con los sensores de reflectancia sobre un piso en niveles de gris. Ahora toca la **entrega final**, que agrega cosas extra a lo que ya funciona (ver "Entrega final" abajo).

Idioma: todo en **español** (código comentado, README, reportes, mensajes de commit).

## Reglas de trabajo

- **Código limpio:** nada de código muerto, variables sin usar ni líneas redundantes que no cambien el resultado. Si algo ya no se usa, se borra.
- No modificar `zumo-2040-robot/` (submódulo oficial de Pololu).
- No hay placa conectada a esta máquina: no se puede cargar firmware ni ver el serial. Para validar lógica de control usar `SecondDelivery/sim/` (compila **el mismo** `follower.c` que el Zumo) y `alpha/alpha_test.c`. Lo que requiera hardware se deja como pendiente explícito para que el equipo lo pruebe en el laboratorio.
- `wifi_config.h` no se sube (está en `.gitignore`); solo `wifi_config.example.h`.
- Cada proyecto de firmware se abre/compila por separado (Pico SDK 2.3.1, extensión Raspberry Pi Pico de VS Code); no la raíz del repo.

## Hardware

- **Zumo 2040** (RP2040), 4 pilas AA. No tiene salida de 5 V: solo 3V3, VSW (pilas) y VBST (~8.3 V). El USB solo alimenta la lógica.
- **PCB del equipo `pcb_agustin/`** (KiCad): Raspberry Pi Pico 2 W (RP2350, CYW43439) + dos módulos MCP2515/TJA1050 ("RASP" para la Pico y "ZUMO" para el Zumo) + **micrófono en GP26/ADC0** de la Pico. Fabricada, soldada y validada (CAN Pico↔Zumo 10/10 en ambas direcciones, también solo con pilas).
  - Se alimenta solo del conector VSW del Zumo (pin 1 cuadrado = VSW, pin 2 = GND; es la única tierra común) a través de dos 1N4001 → VCC_CAN ≈ 4.4 V con pilas (el TJA1050 pide 4.75–5.25 V; funciona pero está al límite). **El switch del Zumo debe estar en ON.**
  - El diseño original compartía SCK/MOSI/MISO entre dos maestros SPI; se corrigió con 4 cortes y un puente GP18 → pin 6 del módulo RASP (`pcb_agustin/cortes_pcb.png`).
  - HL_CAN1 y HL_CAN2 tienen H/L invertidos entre sí (guiarse por la serigrafía). J1 (120 Ω) en los dos módulos → ≈60 Ω entre H y L.
  - Ideas para rev B: nets SPI separadas, transceptor de 3.3 V (SN65HVD230) o regulador de 5 V, Schottky en vez de 1N4001, GND en el header del Zumo, pull-ups en CS.

### Pines

| MCP2515 | Zumo 2040 | Pico 2 W |
|---|---|---|
| MISO (SO) | GP0 | GP16 |
| CS | GP1 | GP17 |
| SCK | GP2 | GP18 |
| MOSI (SI) | GP3 | GP19 |
| INT | GP28 | GP20 |

Restricciones del Zumo que no se ven en el código:
- **Botón C no se puede usar**: está en GP0 (MISO).
- GP0–GP3 comparten SPI0 con la OLED y los LEDs RGB; **GP6 (reloj de los LEDs RGB) se mantiene en bajo** porque GP3 es también datos de los LEDs.
- Cargar firmware con **una sola placa conectada por USB**; si no, picotool puede tomar la Pico 2 W y dejarla en BOOTSEL.

### Sensores de reflectancia (medidos)

Espaciado no uniforme, simétrico; posición lateral sᵢ respecto al centro:
S1 GP22 −0.045 m · S2 GP21 −0.010 m · S3 GP20 0 · S4 GP19 +0.010 m · S5 GP18 +0.045 m.
En el terreno radial, el gris solo da la distancia r a la esquina blanca, no la dirección. α = 0° es el robot apuntando hacia la esquina **negra**.

## Arquitectura (lo que ya funciona)

```
Cámara del techo → PC del lab (ArUco) → WiFi "embedded", UDP broadcast :5000, JSON ~8 Hz
  → Pico 2 W (PicoLPBridge: filtra el marcador id 4) → SPI → MCP2515 → CAN 500 kbps
  → MCP2515 → Zumo (lp_can.c → follower.c → motores)
```

JSON de la cámara: `{"timestamp", "board_detected", "markers":[{"id","x","y","angle"}]}` en metros y grados. A veces llega `"markers": []`. Aparecen marcadores de otros equipos; el nuestro es el **id 4**.

### Base de datos CAN

| ID | Nombre | Emisor → Receptor | DLC | Bytes 0–1 | Bytes 2–3 | Bytes 4–5 |
|---|---|---|---|---|---|---|
| 0x100 | LP_POSE | Pico 2 W → Zumo | 6 | x [mm] int16 | y [mm] int16 | ángulo [0.1°] int16 |

CAN 2.0A (11 bits), **big-endian**, cristal de 8 MHz. Ejemplo real: x=0.176, y=0.079, ángulo=−69.0 → `00 B0 00 4F FD 4E`. Si se agregan mensajes nuevos, documentarlos en esta tabla y en el README.

## Historia de entregas

| Carpeta | Entrega | Resultado |
|---|---|---|
| `Blink/`, `MotorTests/` | Prácticas iniciales | — |
| `CANbus/` | Práctica CAN con MCP2515 (IDs 0x500 y 0x200) | OK |
| (Local Positioning, 23-oct) | Avanzar 50 cm en y solo con la cámara | 49.6 cm con cinta (cámara 503 mm) |
| `ZumoReflectanceStage1/` | QE GrayLevel1: tiempos de descarga de los 5 sensores | OK |
| `SecondDelivery/` | Vuelta completa a la pista en serpentina con 14 waypoints | Vueltas de 102.2, 93.5 y 89.0 s |

Detalle de `SecondDelivery/` (ver su `README.md`, la fuente más completa):
- `follower.c`: arma 7 rectas + 5 arcos de 180° + 2 de 90° (R ≈ 7 cm) a partir de los waypoints; compensa el retardo de cámara; ω = v/R + k_θ·(rumbo ref − θ), con rumbo ref = tangente − atan(k_e·e_y) y k_e ajustada para ζ constante.
- Constantes actuales en `ZumoWaypointFollower/main.c`: `MOTOR_K 7000`, `MOTOR_DEAD 850`, `WHEEL_BASE 0.085`, `CAM_LATENCY 0.12`, `V_STRAIGHT 0.08`, `V_ARC 0.06` m/s, `POSE_TIMEOUT_MS 800`.
- Botón A = vuelta desde P14 mirando a +y; botón B = calibración (fuera de vuelta) o paro (durante la vuelta).
- `alpha/`: parte E, método para estimar α con los sensores de reflectancia (pendiente cargar mediciones reales t_i(r)).
- `analysis/plot_route.py` (con `uv run`): mapa de waypoints y ruta deseada vs. medida a partir del log serial (líneas `L,...`).
- `docs/`: diagramas en Mermaid, reporte (`.docx`/`.pdf`) y su generador (`generador/build.js`, `figs.py`).
- `logs/`: salidas seriales reales de las pruebas.

Pendientes que quedaron de la segunda entrega: subir velocidades poco a poco, medir VCC_CAN con pilas (quieto y con motores) y las mediciones reales de la parte E.

## Entrega final

Requisitos completos: `docs/Final_Evidence_TE2004B_AD26.pdf`. El reporte final documenta 8 subsistemas (motores, encoders, CAN, reflectancia, wireless, multinúcleo/hilos, PCB, micrófono) y estas pruebas: ≥2 unitarias; integración de radio, α y multinúcleo; sistema de vuelta con LP, tonos con ruido y una función innovadora.

### Lo que falta en el código

| # | Qué | Estado actual | Prueba del PDF |
|---|---|---|---|
| 1 | Movimiento en lazo abierto (comandos + tiempo) | `MotorTests/` solo mueve un motor 1.5 s | Diagrama 1 |
| 2 | Lazo cerrado con encoders (p. ej. distancia/cuadrado) | No hay código de encoders en el repo | Diagrama 2, unitaria sugerida |
| 3 | Estimación de radio en el Zumo | Solo en PC (`alpha_radius`) con 3 radios; Stage1 imprime tiempos crudos | Integración 1 |
| 4 | Estimación de α en el Zumo | Solo en PC (`alpha/`); faltan radios de calibración reales | Integración 2 |
| 5 | Multinúcleo / hilos con sincronización | Nada; todo corre en un núcleo | Integración 3 |
| 6 | Micrófono: muestreo, filtrado de ruido, reconocer tonos y mandar la acción por CAN al Zumo | Nada (micrófono en GP26/ADC0 de la Pico 2 W) | Sistema 2 |
| 7 | Función innovadora (sugerida: waypoints en la zona gris con reflectancia + LP) | Nada; requiere aprobación del profesor antes | Sistema 3 |
| 8 | Pruebas unitarias con criterio pass/fail | `alpha_test.c` y `sim/` existen pero no son pruebas unitarias formales | Unitarias (≥2) |
| 9 | Firmware integrado del Zumo (modos: vuelta LP, radio/α, comandos por tono) | Cada función es un proyecto aparte | Sistema 1–3 |

Ya cumplido: CAN (0x100), wireless (PicoLPBridge) y vuelta con LP y waypoints (Sistema 1).

### Prioridades acordadas por el equipo (25-nov)

1. **Multinúcleo, optimización y limpieza** del código existente (Pico 2 W y Zumo), con sincronización explícita entre núcleos.
2. **Vuelta más rápida y con menos error.** Línea base medida en `logs/` (líneas `L,...`, columna `ey_mm`):

   | Log | Tiempo | \|e_y\| máx | RMS |
   |---|---|---|---|
   | vuelta1 | 102.2 s | 38 mm | 15.3 mm |
   | vuelta2 | 93.5 s | 39 mm | 16.7 mm |
   | vuelta4 | 89.0 s | 92 mm | 21.3 mm |

   Los errores grandes se concentran en el arco P3–P4, la esquina de P10 y el inicio/fin (P13–P14). Toda mejora se compara contra esta tabla, primero en `sim/` y luego en pista.
3. **Función innovadora (por ahora):** una GUI en la PC para elegir a cuál de los 14 waypoints debe ir el Zumo. Lo que hace al llegar está por definirse. El Zumo solo escucha CAN, así que el destino tiene que llegarle por la Pico 2 W (por ejemplo, UDP desde la GUI → Pico → un ID CAN nuevo que se documenta en la base de datos CAN).

**Micrófono (#6): pospuesto** hasta que el profesor defina esa parte. No implementar todavía.

Contexto que puede venir al caso:
- También se asignó el reto del **módulo de Procesamiento Digital de Señales (RetoTE2004B Módulo DSP)**, que es el de los tonos del micrófono.
- Los reportes del curso piden: portada (clave y nombre de la clase, título, equipo, integrantes, profesor, fecha), objetivo, resultados con capturas del monitor serial y video, problemas encontrados y cómo se resolvieron, reflexión y bibliografía. Citas en APA 7; se prefieren fuentes oficiales (John Deere, Pololu, datasheets de Microchip/NXP/Raspberry Pi) que el equipo pueda verificar.
