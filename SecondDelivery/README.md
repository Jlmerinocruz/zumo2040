# Second Delivery: vuelta a la pista con waypoints (TE2004B)

Reto: el Zumo 2040 da una vuelta completa a la pista en serpentina siguiendo 14 waypoints. Su pose (x, y, θ) viene de la cámara del techo: llega por WiFi a la Pico 2 W y de ahí por CAN al Zumo. También incluye un método para medir el ángulo α con los sensores de reflectancia (parte E).

| Carpeta | Qué es | Corre en |
|---|---|---|
| `PicoLPBridge/` | Firmware de la Pico 2 W: recibe la pose por WiFi (UDP) y la manda por CAN | Pico 2 W (en la PCB) |
| `ZumoPartA50cm/` | Parte A: avanzar 50 cm en y con la cámara, ya con la PCB | Zumo 2040 |
| `ZumoWaypointFollower/` | Firmware del Zumo: recibe la pose por CAN y sigue la ruta | Zumo 2040 |
| `sim/` | Simulador del seguidor y prueba unitaria de `goto_wp.c`; compilan **el mismo** código del Zumo | PC |
| `gui/` | Función innovadora: GUI para mandar al Zumo a cualquiera de los 14 waypoints | PC (Python) |
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
- `ZumoWaypointFollower/main.c`, `on_arrival()`: **lo que hace al llegar está pendiente de definir**; por ahora imprime `# LLEGO a Pn` y parpadea el LED amarillo.
- Un `GOTO` nuevo o `STOP` interrumpe cualquier recorrido, también la vuelta del botón A. Con un `GOTO` nuevo vuelve a planear desde donde quedó.

### Uso
1. PC conectada al WiFi del laboratorio (Python 3 con tkinter; no requiere instalar nada más).
2. Ver la IP de la Pico 2 W en su monitor serial (`Conectado, IP: …`).
3. `python gui/elegir_waypoint.py --pico <IP de la Pico>` (sin `--pico` manda broadcast). Clic en un punto = ir; **Alto** = detener. La GUI también se suscribe a la cámara y dibuja al Zumo en vivo.
4. Prueba unitaria en la PC: `cd sim && make test` (6 casos con PASS/FAIL, incluye 42 trayectos simulados en lazo cerrado que deben detenerse a menos de 40 mm del destino).

## Cómo funciona el seguidor (`follower.c`)

1. **La ruta se arma a partir de los waypoints.** Entre dos waypoints con el mismo rumbo hay una recta. Entre dos con rumbo distinto hay un arco: su centro y su radio se calculan con los dos puntos medidos, y el sentido de giro con un producto cruz. Así se obtienen 7 rectas, 5 arcos de 180° y 2 de 90°, todos de R ≈ 7 cm.
2. **En cada pose nueva:**
   - se compensa el retardo de la cámara;
   - se proyecta la pose sobre el tramo actual (avance t y error lateral e_y);
   - si t ≥ 1, se pasa al siguiente tramo, en orden y sin saltarse waypoints.
3. **Giro:** ω = v/R (solo en arcos) + k_θ·(rumbo ref − θ), donde el rumbo de referencia es la tangente − atan(k_e·e_y). k_e se ajusta con la velocidad para mantener el amortiguamiento ζ constante. Si fuera fija, el Zumo serpentea al acelerar.
4. **Velocidad:** `v_recta` en las rectas, con frenado para llegar a `v_arco` justo al entrar a cada vuelta.
5. Se detiene al cerrar la vuelta en P14, o si pasan 300 ms sin pose.

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
- [ ] Medir VCC_CAN con pilas, con el Zumo quieto y con motores andando (para el reporte).
- [ ] Calibración con el botón B → `MOTOR_K`, `MOTOR_DEAD`, `WHEEL_BASE` y `CAM_LATENCY`.
- [ ] Vueltas en pista: subir velocidades poco a poco (parte D), 3 tiempos, video y gráfica medida.
- [ ] GUI de waypoints en pista: comprobar UDP 5002 con la Pico, que el Zumo llegue a varios puntos y definir `on_arrival()`.
- [ ] Parte E: medir t_i(r) con α = 0 en 5–7 radios (por ejemplo 0.25, 0.5, 0.75, 1.0, 1.25, 1.5 y 1.75 m) y cargarlos en la tabla de `alpha_test.c`. Opcional: validar con α = 45° y 90° en r = 1.5 m.
