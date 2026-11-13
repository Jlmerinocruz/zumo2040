# Second Delivery: vuelta a la pista con waypoints (TE2004B)

Reto: el Zumo 2040 da una vuelta completa a la pista en serpentina siguiendo 14 waypoints. Su pose (x, y, θ) viene de la cámara del techo: llega por WiFi a la Pico 2 W y de ahí por CAN al Zumo. También incluye un método para medir el ángulo α con los sensores de reflectancia (parte E).

| Carpeta | Qué es | Corre en |
|---|---|---|
| `ZumoWaypointFollower/` | Firmware del Zumo: recibe la pose por CAN y sigue la ruta | Zumo 2040 |
| `sim/` | Simulador del seguidor; compila **el mismo** `follower.c` | PC |
| `analysis/` | Mapa de waypoints y gráfica de ruta deseada vs medida | PC (Python) |
| `alpha/` | Parte E: método y código para α, con su prueba | PC (portable al Zumo) |
| `docs/diagramas.md` | Diagramas de contexto, despliegue y actividades; requisitos; justificación de waypoints | — |

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
3. **Botón B, la primera vez:** calibración con la cámara. Avanza recto y luego gira en su lugar, e imprime `MOTOR_K` y `WHEEL_BASE`. Copiar esos valores a `main.c`.
4. Colocar el Zumo sobre **P14** mirando hacia +y y presionar **A**. Al terminar imprime `# FIN en P14. Tiempo de vuelta: …`.
5. **B durante la vuelta = paro.** El **botón C no se usa**: comparte GP0 con el MISO del MCP2515.
6. Guardar toda la salida serial en un `.txt` para la gráfica. Solo se usan las líneas `L,...`.

## Resultados del simulador

Modelo usado:
- motores con retardo de 80 ms;
- oruga izquierda 5 % más lenta;
- ruido de cámara σ = 5 mm y 0.5°.

| v_recta / v_arco | Cámara | Tiempo de vuelta | Error lateral máx. | RMS |
|---|---|---|---|---|
| 0.30 / 0.15 m/s | 15 Hz, 80 ms | 27.6 s | 18 mm | 8 mm |
| 0.40 / 0.20 m/s | 15 Hz, 80 ms | 21.6 s | 30 mm | 12 mm |
| 0.30 / 0.15 m/s | 10 Hz, 150 ms | 28.2 s | 29 mm | 12 mm |
| 0.40 / 0.20 m/s | 10 Hz, 150 ms | 21.9 s | 39 mm | 17 mm |

Ganancias: k_θ = 5 y ζ = 0.7, elegidas con un barrido. Con k_θ ≥ 6 el control oscila cuando la cámara es lenta. En la pista hay que medir la frecuencia real de la cámara (con los `t_ms` del log) y reajustar.

## Pendiente (requiere hardware o datos)

- [ ] **Formato CAN del compañero** (ID, DLC, bytes y unidades). Ajustar `LP_CAN_ID` y `lp_decode()` en `lp_can.c`. Hoy se supone: ID 0x100, DLC 6, x/y en mm y ángulo en centésimas de grado (int16, little-endian).
- [ ] Parte A: PCB soldada (fotos de las dos caras), Zumo con PCB y marcador (foto) y video de los 50 cm.
- [ ] Calibración con el botón B → `MOTOR_K` y `WHEEL_BASE`.
- [ ] Vueltas en pista: subir velocidades poco a poco (parte D), 3 tiempos, video y gráfica medida.
- [ ] Parte E: medir t_i(r) con α = 0 en 5–7 radios (por ejemplo 0.25, 0.5, 0.75, 1.0, 1.25, 1.5 y 1.75 m) y cargarlos en la tabla de `alpha_test.c`. Opcional: validar con α = 45° y 90° en r = 1.5 m.
