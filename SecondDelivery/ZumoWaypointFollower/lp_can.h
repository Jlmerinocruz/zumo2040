// Decodificación del mensaje CAN con la pose de la cámara (LPdata).
//
// La Pico 2 W recibe (x, y, angle) por WiFi/UDP y lo reenvía por CAN.
// Este es el único lugar que depende del formato del mensaje.
//
// Base de datos CAN-ID del equipo (reporte "Experiment Wireless Interfacing", 3b):
//   LP_POSE, ID 0x100 (11 bits), Pico 2 W → Zumo, DLC 6, BIG-endian, enteros con signo:
//     bytes 0-1: x      en mm               (int16)
//     bytes 2-3: y      en mm               (int16)
//     bytes 4-5: ángulo en décimas de grado (int16)
//   Ejemplo real: x=0.176 m, y=0.079 m, angle=-69.0° → 00 B0 00 4F FD 4E
//   La Pico solo lo manda cuando la cámara ve el marcador 4 (~8 veces por segundo).
//
// Ir a un waypoint (función innovadora, GUI de la PC → Pico 2 W → Zumo):
//   WP_GOTO,   ID 0x110, Pico 2 W → Zumo, DLC 1:
//     byte 0: waypoint destino 1..14, 0 = alto
//   WP_STATUS, ID 0x111, Zumo → Pico 2 W, DLC 2 (la Pico lo reenvía a la GUI por UDP):
//     byte 0: estado (wp_state_t)   byte 1: waypoint destino 1..14
//
// Mapa de calor del suelo (función innovadora, se pinta en la GUI):
//   FLOOR, ID 0x112, Zumo → Pico 2 W, DLC 5, 10 veces por segundo durante un recorrido
//   (la Pico lo reenvía a la GUI por UDP como "PISO b1 b2 b3 b4 b5"):
//     byte i: tiempo de descarga del sensor S(i+1) en unidades de 4 µs (uint8, 255 =
//     1020 µs o más). S1 es el de la izquierda del Zumo y S5 el de la derecha.
//     Claro ≈ 10-20 (40-75 µs), gris oscuro ≈ 60-120 (240-475 µs).
#ifndef LP_CAN_H
#define LP_CAN_H

#include <stdbool.h>
#include "mcp2515.h"
#include "follower.h"

#define LP_CAN_ID   0x100
#define LP_CAN_DLC  6

#define WP_GOTO_ID    0x110
#define WP_GOTO_DLC   1
#define WP_STATUS_ID  0x111
#define WP_STATUS_DLC 2
#define FLOOR_ID      0x112
#define FLOOR_DLC     5

typedef enum {
    WP_IDLE = 0,      // en espera
    WP_MOVING = 1,    // en camino al destino
    WP_ARRIVED = 2,   // llegó al destino
    WP_REJECTED = 3,  // no arrancó: sin pose reciente o fuera de la pista
    WP_STOPPED = 4,   // se detuvo antes de llegar: alto (GUI o botón B) o sin pose
} wp_state_t;

// Devuelve true si el frame es de LPdata y lo pudo convertir a pose (m, m, rad).
bool lp_decode(const can_frame_t *frame, pose_t *pose);

// Devuelve true si el frame es WP_GOTO; *waypoint = 1..14, o 0 = alto.
bool wp_goto_decode(const can_frame_t *frame, uint8_t *waypoint);

// Arma el frame WP_STATUS (waypoint 1..14).
can_frame_t wp_status_frame(wp_state_t state, uint8_t waypoint);

// Arma el frame FLOOR con los 5 tiempos de descarga en µs (line_sensors de Pololu).
can_frame_t floor_frame(const uint16_t t_us[FLOOR_DLC]);

#endif
