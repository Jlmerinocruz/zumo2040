#include "waypoints.h"

// Orden de recorrido: P14 → P1 → P2 → … → P13 → P14 (sentido horario).
const waypoint_t WAYPOINTS[WAYPOINT_COUNT] = {
    /* P1  */ {-0.36f, 1.09f,   91.0f},  // entrada vuelta 180° (arriba)
    /* P2  */ {-0.22f, 1.08f,  -90.0f},  // salida
    /* P3  */ {-0.21f, 0.40f,  -90.0f},  // entrada vuelta 180° (abajo)
    /* P4  */ {-0.07f, 0.41f,   90.0f},  // salida
    /* P5  */ {-0.07f, 1.09f,   90.0f},  // entrada vuelta 180° (arriba)
    /* P6  */ { 0.07f, 1.08f,  -90.0f},  // salida
    /* P7  */ { 0.08f, 0.40f,  -90.0f},  // entrada vuelta 180° (abajo)
    /* P8  */ { 0.22f, 0.40f,   90.0f},  // salida
    /* P9  */ { 0.22f, 1.08f,   90.0f},  // entrada vuelta 180° (arriba)
    /* P10 */ { 0.36f, 1.09f,  -90.0f},  // salida
    /* P11 */ { 0.35f, 0.22f,  -90.0f},  // entrada vuelta 90° (esquina inf. der.)
    /* P12 */ { 0.29f, 0.15f, -180.0f},  // salida
    /* P13 */ {-0.28f, 0.16f, -180.0f},  // entrada vuelta 90° (esquina inf. izq.)
    /* P14 */ {-0.36f, 0.23f,   90.0f},  // salida = INICIO y FIN de la vuelta
};
