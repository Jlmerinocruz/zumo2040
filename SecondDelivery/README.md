# Second Delivery: vuelta a la pista con waypoints (TE2004B)

Reto: el Zumo 2040 da una vuelta completa a la pista en serpentina siguiendo 14 waypoints. Su pose (x, y, θ) viene de la cámara del techo: llega por WiFi a la Pico 2 W y de ahí por CAN al Zumo. También incluye un método para medir el ángulo α con los sensores de reflectancia (parte E).

| Carpeta | Qué es | Corre en |
|---|---|---|
| `PicoLPBridge/` | Firmware de la Pico 2 W: recibe la pose por WiFi (UDP) y la manda por CAN | Pico 2 W (en la PCB) |
| `ZumoPartA50cm/` | Parte A: avanzar 50 cm en y con la cámara, ya con la PCB | Zumo 2040 |
| `ZumoWaypointFollower/` | Firmware del Zumo: recibe la pose por CAN y sigue la ruta | Zumo 2040 |
| `common/` | Driver del MCP2515 que comparten la Pico 2 W y el Zumo (solo cambian los pines) | Pico 2 W y Zumo 2040 |
| `sim/` | Simulador del seguidor; compila **el mismo** `follower.c` | PC |
| `analysis/` | Mapa de waypoints y gráfica de ruta deseada vs medida | PC (Python) |
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

## Cómo funciona el seguidor (`follower.c`)

1. **La ruta se arma a partir de los waypoints.** Entre dos waypoints con el mismo rumbo hay una recta. Entre dos con rumbo distinto hay un arco: su centro y su radio se calculan con los dos puntos medidos, y el sentido de giro con un producto cruz. Así se obtienen 7 rectas, 5 arcos de 180° y 2 de 90°, todos de R ≈ 7 cm.
2. **En cada pose nueva:**
   - se compensa el retardo de la cámara;
   - se proyecta la pose sobre el tramo actual (avance t y error lateral e_y);
   - si t ≥ 1, se pasa al siguiente tramo, en orden y sin saltarse waypoints.
3. **Giro:** ω = v/R (solo en arcos) + k_θ·(rumbo ref − θ), donde el rumbo de referencia es la tangente − atan(k_e·e_y). k_e se ajusta con la velocidad para mantener el amortiguamiento ζ constante. Si fuera fija, el Zumo serpentea al acelerar.
4. **Velocidad:** `v_recta` en las rectas, con frenado para llegar a `v_arco` justo al entrar a cada vuelta.
5. Se detiene al cerrar la vuelta en P14, o si pasan 300 ms sin pose.

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
./sim_follower                    # v_recta=0.30, v_arco=0.15, cámara 15 Hz / 80 ms
./sim_follower 0.40 0.20 10 0.15  # v_recta v_arco Hz_cámara retardo_s
ZETA=0.7 K_TH=5 ./sim_follower 0.40 0.20 > vuelta.csv   # probar otras ganancias
```

### Gráficas (PC, con `uv`)
```bash
uv run analysis/plot_route.py                  # mapa de waypoints numerados
uv run analysis/plot_route.py sim/vuelta.csv   # ruta deseada vs simulada
uv run analysis/plot_route.py log_vuelta1.txt  # ruta deseada vs real (log serial del Zumo)
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

Modelo usado:
- cámara a 8 Hz, que es lo que midió el reporte de Wireless (una pose cada ~125 ms);
- motores con retardo de 80 ms y oruga izquierda 5 % más lenta;
- ruido de cámara σ = 5 mm y 0.5°.

| v_recta / v_arco | Retardo de cámara | Tiempo de vuelta | Error lateral máx. | RMS |
|---|---|---|---|---|
| 0.30 / 0.15 m/s | 100 ms | 28.0 s | 35 mm | 13 mm |
| 0.40 / 0.20 m/s | 100 ms | 22.1 s | 41 mm | 20 mm |
| 0.30 / 0.15 m/s | 200 ms | 29.5 s | 55 mm | 22 mm |
| 0.40 / 0.20 m/s | 200 ms | 23.2 s | 72 mm | 34 mm |

Ganancias: k_θ = 4 y ζ = 0.8. Con k_θ = 5 el control va mejor si el retardo es de 100 ms, pero oscila si es de 200 ms. **El retardo real todavía no se conoce.** La calibración del botón B lo imprime: con ese valor, ajustar `CAM_LATENCY` en `main.c` y, si es ≤ 100 ms, subir k_θ a 5.

## Pendiente (requiere hardware o datos)

- [x] Formato CAN de la Pico 2 W (reporte Wireless): `LP_POSE`, ID 0x100, DLC 6, **big-endian**, x/y en mm y ángulo en **décimas** de grado. Se probó `lp_decode()` con los mensajes reales del reporte.
- [x] PCB fabricada, corregida (bus SPI compartido) y validada: SPI de cada micro en loopback y CAN Pico↔Zumo 10/10 en las dos direcciones, también solo con pilas.
- [ ] Parte A: fotos de las dos caras de la PCB y de los cortes, Zumo con PCB y marcador (foto) y video de los 50 cm (`ZumoPartA50cm/`).
- [ ] Probar la versión multinúcleo en el laboratorio (Pico 2 W y Zumo): que la vuelta salga igual que antes y guardar las líneas `# nucleo 0` y `# colas` para el reporte.
- [ ] Medir VCC_CAN con pilas, con el Zumo quieto y con motores andando (para el reporte).
- [ ] Calibración con el botón B → `MOTOR_K`, `MOTOR_DEAD`, `WHEEL_BASE` y `CAM_LATENCY`.
- [ ] Vueltas en pista: subir velocidades poco a poco (parte D), 3 tiempos, video y gráfica medida.
- [ ] Parte E: medir t_i(r) con α = 0 en 5–7 radios (por ejemplo 0.25, 0.5, 0.75, 1.0, 1.25, 1.5 y 1.75 m) y cargarlos en la tabla de `alpha_test.c`. Opcional: validar con α = 45° y 90° en r = 1.5 m.
