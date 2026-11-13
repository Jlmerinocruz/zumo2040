// Decodificación del mensaje CAN con la pose de la cámara (LPdata).
//
// La Pico 2 W recibe (x, y, angle) por WiFi/UDP y lo reenvía por CAN.
// ESTE ES EL ÚNICO LUGAR QUE DEPENDE DEL FORMATO DEL MENSAJE: cuando tengamos el
// código de la Pico 2 W del compañero, ajustar LP_CAN_ID y lp_decode() a su ID,
// DLC, orden de bytes y unidades.
//
// Formato supuesto por ahora (DLC = 6, little-endian, enteros con signo):
//   bytes 0-1: x      en mm          (int16)
//   bytes 2-3: y      en mm          (int16)
//   bytes 4-5: ángulo en centésimas de grado (int16, -18000..18000)
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
