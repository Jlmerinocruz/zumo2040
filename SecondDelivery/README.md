# Second Delivery: vuelta a la pista con waypoints (TE2004B)

Reto: el Zumo 2040 da una vuelta completa a la pista en serpentina siguiendo 14 waypoints. Su pose (x, y, θ) viene de la cámara del techo: llega por WiFi a la Pico 2 W y de ahí por CAN al Zumo. También incluye un método para medir el ángulo α con los sensores de reflectancia (parte E).

| Carpeta | Qué es | Corre en |
|---|---|---|
| `PicoLPBridge/` | Firmware de la Pico 2 W: recibe la pose por WiFi (UDP) y la manda por CAN | Pico 2 W (en la PCB) |
| `ZumoPartA50cm/` | Parte A: avanzar 50 cm en y con la cámara, ya con la PCB | Zumo 2040 |
| `ZumoWaypointFollower/` | Firmware del Zumo: recibe la pose por CAN y sigue la ruta | Zumo 2040 |
| `common/` | Driver del MCP2515 que comparten la Pico 2 W y el Zumo (solo cambian los pines) | Pico 2 W y Zumo 2040 |
| `sim/` | Simulador del seguidor y prueba unitaria de `goto_wp.c`; compilan **el mismo** código del Zumo | PC |
| `gui/` | Función innovadora: GUI para mandar al Zumo a cualquiera de los 14 waypoints | PC (Python) |
| `analysis/` | Mapa de waypoints, gráfica de ruta deseada vs medida y análisis por tramo de los logs | PC (Python) |
| `alpha/` | Parte E: método y código para α, con su prueba | PC (portable al Zumo) |
| `docs/diagramas.md` | Diagramas de contexto, despliegue y actividades; requisitos; justificación de waypoints | — |

## Prueba desde cero (con la PCB)

### Conexiones físicas
- **Conector VSW** de la PCB: pin 1 (cuadrado) a VSW del Zumo, pin 2 a GND. Es la única tierra común y la única alimentación de la PCB: la Pico (VSYS) y los dos TJA1050 (sockets VCC_CAN1/2) salen de VSW pasando por dos 1N4001 (~4.4 V con pilas). **El switch del Zumo debe estar en ON.**
- **Tira de 5 pines al Zumo:** 1 CS→GP1, 2 MISO→GP0, 3 MOSI→GP3, 4 SCK→GP2, 5 INT→GP28.
- **Módulo RASP:** pad cuadrado = VCC. **Módulo ZUMO:** pad cuadrado = INT; va en la misma orientación que el RASP. Cable de 5 V del transceptor a VCC_CAN1 (RASP) y VCC_CAN2 (ZUMO).
- **Bus CAN:** H con H y L con L. Las borneras HL_CAN1 y HL_CAN2 tienen el orden **invertido**: guiarse por la serigrafía. Jumper J1 (120 Ω) en **los dos** módulos; entre H y L, sin energía, debe medir ≈ 60 Ω.
- La PCB lleva el arreglo del bus SPI compartido (4 cortes y un puente de GP18 al pin 6 del módulo RASP). Ver `../pcb_agustin/cortes_pcb.png`.

### Orden de carga (una placa por USB a la vez)
1. **Pico 2 W:** abrir `PicoLPBridge/` en el IDE. Si no existe `wifi_config.h`, copiar `wifi_config.example.h` con ese nombre y poner la red del laboratorio. Compilar y cargar con *Run*. Si la Pico tiene MicroPython, entrar a BOOTSEL (mantener BOOTSEL al conectar el USB).
   - LED parpadeando rápido = conectando al WiFi; fijo = conectado; parpadea con cada pose = el Zumo la confirmó por CAN; apagado = llegan poses pero nadie da ACK.
   - En el monitor serial imprime cada 2 s cuántas poses llegaron por UDP y cuántas salieron por CAN.
2. **Zumo, parte A:** abrir `ZumoPartA50cm/`, compilar y cargar (si tiene MicroPython: mantener B y presionar Reset). Con la cámara viendo el marcador debe imprimir `pose: x=… y=…`. Colocar en el waypoint inicial mirando hacia +y, presionar A: avanza 500 mm y se detiene. Medir con cinta y grabar el video.
3. **Zumo, vuelta completa:** cargar `ZumoWaypointFollower/` y seguir la sección *En el Zumo* de abajo (primero calibración con B, luego vueltas con A).

Si el Zumo imprime `MCP2515 no responde`, revisar el switch en ON (sin él la PCB no tiene energía) y la tira de 5 pines. Si se detiene con `ALTO: sin pose`, la línea `# CAN: EFLG=… TEC=… REC=…` dice si falló el bus (TEC/REC altos, EFLG ≠ 0) o la cámara (todo en 0).

## Función innovadora: ir a un waypoint desde la GUI

En la PC se elige en un mapa a cuál de los 14 waypoints debe ir el Zumo. El Zumo va **siguiendo la pista** en el sentido de la vuelta (nunca cruza carriles) desde el tramo donde esté hasta el punto elegido, con el mismo `follower.c` de la vuelta. Si el destino ya quedó atrás, da la vuelta completa para llegar a él.

```
GUI (PC) --UDP 5002 "GOTO n"/"STOP"--> Pico 2 W --CAN 0x110 WP_GOTO--> Zumo
GUI (PC) <--UDP "ESTADO e n"--------- Pico 2 W <--CAN 0x111 WP_STATUS-- Zumo
```

| ID | Nombre | Emisor → Receptor | DLC | Byte 0 | Byte 1 |
|---|---|---|---|---|---|
| 0x110 | WP_GOTO | Pico 2 W → Zumo | 1 | destino 1–14, 0 = alto | — |
| 0x111 | WP_STATUS | Zumo → Pico 2 W | 2 | estado: 0 en espera, 1 en camino, 2 llegó, 3 rechazado (sin pose o a más de 10 cm de la pista), 4 detenido (alto, botón B o sin pose) | destino 1–14 |

- `ZumoWaypointFollower/goto_wp.c`: ubica al Zumo en su tramo y arma la ruta hasta el destino (C portable).
- Multinúcleo: en el Zumo el núcleo 1 recibe el WP_GOTO y lo pasa al núcleo 0 por una cola; el WP_STATUS va del núcleo 0 al 1 por otra cola y lo envía el núcleo 1 (dueño único del SPI). En la Pico, el núcleo 0 recibe el UDP y encola el comando; el núcleo 1 lo manda por CAN, lee el WP_STATUS y devuelve el texto por una cola para que el núcleo 0 lo mande por UDP.
- `ZumoWaypointFollower/main.c`, `on_arrival()`: **lo que hace al llegar está pendiente de definir**; por ahora imprime `# LLEGO a Pn` y parpadea el LED amarillo.
- Un `GOTO` nuevo o `STOP` interrumpe cualquier recorrido, también la vuelta del botón A. Con un `GOTO` nuevo vuelve a planear desde donde quedó.

### Uso
1. PC conectada al WiFi del laboratorio (Python 3 con tkinter; no requiere instalar nada más).
2. Ver la IP de la Pico 2 W en su monitor serial (`Conectado, IP: …`).
3. `python gui/elegir_waypoint.py --pico <IP de la Pico>` (sin `--pico` manda broadcast). Clic en un punto = ir; **Alto** = detener. La GUI también se suscribe a la cámara y dibuja al Zumo en vivo.
4. Prueba unitaria en la PC: `cd sim && make test` (6 casos con PASS/FAIL, incluye 42 trayectos simulados en lazo cerrado que deben detenerse a menos de 40 mm del destino).

### Mapa de calor del suelo (cámara + reflectancia)

Junta dos sensores distintos: la cámara del techo dice **dónde** está el Zumo y sus 5 sensores de reflectancia dicen **qué tan oscuro** es el piso ahí. Durante cualquier recorrido (vuelta con A o GOTO) el Zumo lee los sensores 10 veces por segundo y la GUI los pinta sobre la pista, de amarillo (claro) a morado (oscuro). Con hojas de colores sobre la pista se ven como manchas en el mapa.

```
Zumo (line_sensors_read, núcleo 0) --cola--> núcleo 1 --CAN 0x112 FLOOR--> Pico 2 W --UDP "PISO b1..b5"--> GUI
```

| ID | Nombre | Emisor → Receptor | DLC | Bytes 0–4 |
|---|---|---|---|---|
| 0x112 | FLOOR | Zumo → Pico 2 W | 5 | tiempo de descarga de S1..S5 en unidades de 4 µs (uint8, 255 = 1020 µs o más; S1 = izquierda) |

- Es un frame de 5 bytes cada 100 ms y solo durante un recorrido (≈0.2 % del bus a 500 kbps). La lectura usa el PIO de la librería de Pololu y tarda 1 ms como máximo.
- La GUI manda `HOLA` a la Pico cada 2 s para recibir los `PISO` aunque la vuelta se arranque con el botón A.
- Cada lectura se pinta con la pose de la cámara que llega 0.12 s después (`FLOOR_DELAY_S`), 4 cm adelante del marcador (`SENSOR_AHEAD_M`, aproximado: medirlo) y con la posición lateral de cada sensor.
- La escala es automática por sensor, porque con el mismo piso S1 y S5 tardan casi el doble que S3. Va del percentil 5 al 95 de lo visto (no del mínimo al máximo): en la prueba del 26/11 las pocas lecturas de S1 y S5 sobre la línea negra estiraban la escala y todas las parcelas salían del mismo color. **Borrar mapa** reinicia la escala.
- En la pista de la vuelta (sobre la foto aérea de cultivos), S2–S4 van sobre la línea negra y S1/S5 sobre las parcelas.

## Cómo funciona el seguidor (`follower.c`)

1. **La ruta se arma a partir de los waypoints.** Entre dos waypoints con el mismo rumbo hay una recta. Entre dos con rumbo distinto hay un arco: su centro y su radio se calculan con los dos puntos medidos, y el sentido de giro con un producto cruz. Así se obtienen 7 rectas, 5 arcos de 180° y 2 de 90°, todos de R ≈ 7 cm.
2. **En cada pose nueva:**
   - se compensa el retardo de la cámara;
   - se proyecta la pose sobre el tramo actual (avance t y error lateral e_y);
   - si t ≥ 1, se pasa al siguiente tramo, en orden y sin saltarse waypoints.
3. **Giro:** ω = v/R (solo en arcos) + k_θ·(rumbo ref − θ), donde el rumbo de referencia es la tangente − atan(k_e·e_y). k_e se ajusta con la velocidad para mantener el amortiguamiento ζ constante. Si fuera fija, el Zumo serpentea al acelerar.
4. **Velocidad:** `v_recta` en las rectas, con frenado para llegar a `v_arco` justo al entrar a cada vuelta.
5. **Entre poses** (`follower_tick`, cada 20 ms): la pose se estima integrando (v, ω) y el control se recalcula. Por el WiFi, en ~40 % de la vuelta pasan más de 250 ms sin pose; antes el Zumo seguía a ciegas con el último comando.
6. **Ganancias medidas con la cámara:** el Zumo no hace lo que se le manda. En las vueltas del 17/11 avanzaba ~1.3 veces la v mandada en rectas y giraba solo ~0.3 veces la ω mandada en los arcos, por eso se abría 15–25 mm hacia afuera en **todos** los arcos. Se compara lo que avanzó y giró (cámara) contra lo que se mandó a las orugas (desplazado por el retardo) y se ajustan g_v y g_ω con un cociente de sumas con olvido. Los intervalos entre poses se juntan hasta tener ~1 cm o ~0.1 rad mandados, porque las poses llegan en grupos y un intervalo suelto no cuadra con su comando. g_ω solo se ajusta en los arcos y no baja de 0.25. La diferencia entre orugas se limita a 0.4 m/s (`diff_max`): con las orugas en ±0.6 m/s, en la vuelta 6 del 25/11 el Zumo pivoteaba en zigzag en los arcos. El comando a las orugas es v/g_v y ω/g_ω. Al terminar la vuelta se imprimen los valores medidos. `ADAPT_GAINS` en `main.c` lo apaga.
7. Se detiene al cerrar la vuelta en P14. Si pasan 800 ms sin pose se pausa, y si pasan 3 s termina.

## Reparto entre núcleos (multinúcleo)

Las dos placas usan sus dos núcleos. Cada recurso tiene un solo dueño y los núcleos solo se comunican por colas `queue_t` del Pico SDK (FIFO protegidas por un spinlock de hardware, que nunca bloquean al otro núcleo).

| Placa | Núcleo 0 | Núcleo 1 | Entre núcleos |
|---|---|---|---|
| Zumo 2040 (`ZumoWaypointFollower/`) | Lazo de control (`follower.c`), motores, botones, calibración, pausa/paro por pérdida de pose | MCP2515 por SPI0 (la interrupción en INT/GP28 anota la hora de llegada de cada pose), revisión del modo CAN y toda la salida serial | Cola de poses 1→0 (el control usa solo la más reciente); cola de telemetría 0→1 (muestras `L,...` y texto) |
| Pico 2 W (`PicoLPBridge/`) | WiFi/lwIP, parseo del JSON, LED, watchdog y resumen cada 2 s | MCP2515: envío por CAN con su espera de ACK e impresión de cada pose | Cola de poses 0→1; contadores con un solo escritor cada uno |

Detalles:
- **Zumo:** un `printf` por USB o una espera del SPI ya no pueden retrasar el comando a los motores. El programa corre desde RAM (`copy_to_ram`): leer el botón B desconecta un instante la flash (comparte pin con su CS) y el otro núcleo no puede estar ejecutando desde ella en ese momento.
- **Zumo:** al terminar o detener una vuelta imprime la evidencia para la prueba de integración multinúcleo: `# nucleo 0: … calculo max … us, edad de la pose al usarla prom … ms / max … ms` y `# colas: … poses reemplazadas …, … mensajes de log perdidos`.
- **Pico 2 W:** el chip WiFi y lwIP solo se usan desde el núcleo 0. El watchdog (3 s) solo se alimenta si el núcleo 1 también avanza, así que se reinicia si cualquiera de los dos se cuelga.
- **MCP2515:** lectura y escritura de cada mensaje en una sola transacción SPI (`READ RX BUFFER` / `LOAD TX BUFFER`) y rollover de RXB0 a RXB1, para no perder poses cuando llegan en ráfaga.

## Uso

### Simulador (PC)
```bash
cd sim && make
./sim_follower 0.15 0.10 > vuelta.csv   # v_recta v_arco (cámara 9.5 Hz / 120 ms)
./sim_follower 0.15 0.10 8 0.2          # v_recta v_arco Hz_cámara retardo_s
K_TH=5 ZETA=0.7 ./sim_follower 0.15 0.10            # probar otras ganancias
ADAPT=0 TICK=0 LAT=0.12 ./sim_follower 0.08 0.06    # como el firmware del 17/11
IDEAL=1 ./sim_follower 0.15 0.10                    # planta ideal (modelo anterior)
PARES=1 HOLD=0.02 DZ=0.12 WB_REAL=0.17 GAIN=1.0 ./sim_follower 0.12 0.08   # como la prueba del 25/11
```

### Gráficas (PC, con `uv`)
```bash
uv run analysis/plot_route.py                  # mapa de waypoints numerados
uv run analysis/plot_route.py sim/vuelta.csv   # ruta deseada vs simulada
uv run analysis/plot_route.py log_vuelta1.txt  # ruta deseada vs real (log serial del Zumo)
uv run analysis/analizar_vueltas.py logs/vuelta4.txt   # error, huecos y real/mandado por tramo
```

### En el Zumo
1. Abrir `ZumoWaypointFollower/` solo en el IDE, compilar y cargar con *Run*.
2. Al arrancar imprime la tabla de tramos. Después muestra la pose que recibe por CAN.
3. **Botón B, la primera vez:** calibración con la cámara. Avanza recto a dos velocidades (dejar ~1 m libre al frente) y luego gira en su lugar. Imprime `MOTOR_K`, `MOTOR_DEAD` (zona muerta del motor), `WHEEL_BASE`, las poses por segundo y el retardo efectivo. Copiar esos valores a `main.c`.
4. Colocar el Zumo sobre **P14** mirando hacia +y y presionar **A**. Al terminar imprime `# FIN en P14. Tiempo de vuelta: …`.
5. **B durante la vuelta = paro.** El **botón C no se usa**: comparte GP0 con el MISO del MCP2515. GP6 (reloj de los LEDs RGB) se mantiene en bajo, porque GP3 es a la vez MOSI y datos de los LEDs.
6. Cargar el código con **una sola placa conectada por USB**. Si no, picotool puede tomar la Pico 2 W y dejarla en BOOTSEL (problema 3 del reporte Wireless).
7. Guardar toda la salida serial en un `.txt` para la gráfica. Solo se usan las líneas `L,...`.

## Resultados del simulador

El modelo de planta se ajustó para que el firmware del 17/11 se equivoque igual que en pista (`analysis/analizar_vueltas.py` sobre `logs/vuelta4.txt`):
- orugas que van 1.3 veces lo mandado, con la izquierda 5 % más lenta, y respuesta de 80 ms;
- separación efectiva de orugas de 0.30 m (la geométrica es 0.085 m): así el Zumo se abre 15–25 mm en los arcos, como en pista;
- cámara a 9.5 Hz con 120 ms de retardo, ruido σ = 5 mm y 0.5°, y un hueco de 450 ms cada 0.8 s (las ráfagas del WiFi).

Promedio de 5 corridas con ruido distinto (`SEED=1..5`); el máximo es el peor de las 5.

| v_recta / v_arco | Firmware del 17/11 | Con `follower_tick` + ganancias medidas |
|---|---|---|
| 0.08 / 0.06 m/s | 75.6 s, máx 47 mm, RMS 16.2 mm | 84.5 s, máx 12 mm, RMS 3.1 mm |
| 0.12 / 0.08 m/s | 55.8 s, máx 69 mm, RMS 28.3 mm | 59.0 s, máx 15 mm, RMS 3.5 mm |
| 0.15 / 0.10 m/s | 46.8 s, máx 103 mm, RMS 40.9 mm | 47.6 s, máx 20 mm, RMS 4.6 mm |
| 0.20 / 0.12 m/s | 38.3 s, máx 122 mm, RMS 58.4 mm | 37.4 s, máx 30 mm, RMS 6.3 mm |

A la misma velocidad mandada, el firmware anterior da vueltas más cortas porque en rectas el Zumo iba más rápido de lo que se mandaba; el nuevo va a lo que se manda.

Ninguno de los cambios depende de conocer la planta. A 0.15 / 0.10 m/s, con separación efectiva de 0.20 o 0.41 m, con orugas exactas (ganancia 1.0) o con la planta ideal, el RMS queda entre 4.1 y 4.9 mm. Con 200 ms de retardo de cámara (compensando 160 ms) queda en ~9 mm. El retardo a compensar (`CAM_LATENCY`) funciona entre 0.12 y 0.20 s, y lo mejor fue 0.16 s (cámara + medio tiempo de respuesta del motor). k_θ = 4 y ζ = 0.8 siguen siendo buenos: con 3 o 5 cambia poco.

**Prueba en pista del 25/11 (0.12 / 0.08 m/s):** las rectas salieron bien (|e_y| < 35 mm), pero en los arcos las orugas se saturaban y el rumbo oscilaba. Había dos causas, y las dos se reprodujeron en el simulador:
- las poses llegaban de dos en dos cada 200 ms, así que g_v y g_ω se estimaban a la mitad;
- en las rectas, las correcciones caían en la zona muerta del giro, y eso bajaba g_ω al mínimo.

Se corrigió así: los intervalos se juntan, g_ω se ajusta solo en arcos con un mínimo de 0.25, y `main.c` espera 20 ms después de cada pose para usar solo la más reciente del grupo. Con poses en pares y una zona muerta de giro de 0.12 m/s (`PARES=1 HOLD=0.02 DZ=0.12 WB_REAL=0.17 GAIN=1.0`), a 0.12 / 0.08 m/s el simulador da RMS 14.8 mm. Con el control del 17/11 da 37.4 mm, y con la versión que falló en pista, 33.5 mm.

**Vueltas 6 y 7 del 25/11 (0.12 / 0.08 m/s), ya con ese arreglo:**
- La vuelta 6 salió completa en 62.8 s, pero los arcos P3–P4 y P7–P8 iban en zigzag, con las orugas en ±0.6 m/s. Por eso se agregó `diff_max`.
- La vuelta 7 salió completa en 63.0 s, sin zigzag (g_v = 1.19, g_ω = 0.26). Lo peor fue la primera recta: e_y crece de 5 a 32 mm porque g_ω todavía no se ajustaba. Ahora g_ω arranca en 0.27 (`FOLLOWER_GW0`). En el resto de los tramos, |e_y| ≤ 18 mm. La línea base era la vuelta 4: 89.0 s, con máximo de 92 mm.

Lo que no modela el simulador: que la cámara deje de ver el marcador por imagen movida (pasó a 0.20 m/s mandados, que en realidad eran ~0.26 m/s), el arrastre extra de las orugas al girar (en pista los arcos tardan más) y la caída de las pilas.

## Pendiente (requiere hardware o datos)

- [x] Formato CAN de la Pico 2 W (reporte Wireless): `LP_POSE`, ID 0x100, DLC 6, **big-endian**, x/y en mm y ángulo en **décimas** de grado. Se probó `lp_decode()` con los mensajes reales del reporte.
- [x] PCB fabricada, corregida (bus SPI compartido) y validada: SPI de cada micro en loopback y CAN Pico↔Zumo 10/10 en las dos direcciones, también solo con pilas.
- [ ] Parte A: fotos de las dos caras de la PCB y de los cortes, Zumo con PCB y marcador (foto) y video de los 50 cm (`ZumoPartA50cm/`).
- [ ] Probar la versión multinúcleo en el laboratorio (Pico 2 W y Zumo): que la vuelta salga igual que antes y guardar las líneas `# nucleo 0` y `# colas` para el reporte.
- [ ] Medir VCC_CAN con pilas, con el Zumo quieto y con motores andando (para el reporte).
- [ ] Calibración con el botón B → `MOTOR_K`, `MOTOR_DEAD`, `WHEEL_BASE` y `CAM_LATENCY`.
- [ ] Vueltas en pista: subir velocidades poco a poco (parte D), 3 tiempos, video y gráfica medida.
- [ ] Probar en pista `follower_tick` + ganancias medidas, primero a 0.12 / 0.08 m/s y luego a 0.15 / 0.10. Comparar con `analysis/analizar_vueltas.py` contra la línea base (vuelta 4: 89.0 s, máx 92 mm, RMS 21.3 mm) y anotar los `g_v`, `g_w` que imprime al final.
- [ ] Mapa de calor del suelo: vuelta con hojas de colores sobre la pista, captura de la GUI y video. Medir `SENSOR_AHEAD_M` (distancia del centro del marcador a la fila de sensores) y revisar que S1 quede a la izquierda en el mapa.
- [ ] GUI de waypoints en pista: comprobar UDP 5002 con la Pico, que el Zumo llegue a varios puntos y definir `on_arrival()`.
- [ ] Parte E: medir t_i(r) con α = 0 en 5–7 radios (por ejemplo 0.25, 0.5, 0.75, 1.0, 1.25, 1.5 y 1.75 m) y cargarlos en la tabla de `alpha_test.c`. Opcional: validar con α = 45° y 90° en r = 1.5 m.
