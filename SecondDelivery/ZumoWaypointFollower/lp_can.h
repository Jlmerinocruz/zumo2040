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
#ifndef LP_CAN_H
#define LP_CAN_H

#include <stdbool.h>
#include "mcp2515.h"
#include "follower.h"

#define LP_CAN_ID   0x100
#define LP_CAN_DLC  6

// Devuelve true si el frame es de LPdata y lo pudo convertir a pose (m, m, rad).
bool lp_decode(const can_frame_t *frame, pose_t *pose);

#endif
