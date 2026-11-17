#include "waypoints.h"

// Orden de recorrido: P14 → P1 → P2 → … → P13 → P14 (sentido horario).
// Medidos el 17/11 con la cámara del techo y el marcador ya montado sobre el soporte
// de la PCB (mediana de 8 poses con el Zumo quieto; variación ±1–2 mm y ±1°).
// La tabla anterior (13/11) se midió con el soporte de protoboard: el marcador quedaba
// en otra posición sobre el Zumo y los puntos salían corridos hasta ~8 cm.
const waypoint_t WAYPOINTS[WAYPOINT_COUNT] = {
    /* P1  */ {-0.375f, 1.005f,   92.0f},  // entrada vuelta 180° (arriba)
    /* P2  */ {-0.226f, 1.062f,  -88.0f},  // salida
    /* P3  */ {-0.214f, 0.359f,  -88.0f},  // entrada vuelta 180° (abajo)
    /* P4  */ {-0.075f, 0.355f,   92.0f},  // salida
    /* P5  */ {-0.073f, 1.034f,   94.0f},  // entrada vuelta 180° (arriba)
    /* P6  */ { 0.075f, 1.058f,  -88.0f},  // salida
    /* P7  */ { 0.074f, 0.362f,  -89.0f},  // entrada vuelta 180° (abajo)
    /* P8  */ { 0.223f, 0.346f,   91.0f},  // salida
    /* P9  */ { 0.225f, 1.038f,   91.0f},  // entrada vuelta 180° (arriba)
    /* P10 */ { 0.371f, 1.067f,  -88.0f},  // salida
    /* P11 */ { 0.373f, 0.163f,  -88.0f},  // entrada vuelta 90° (esquina inf. der.)
    /* P12 */ { 0.304f, 0.075f, -179.0f},  // salida
    /* P13 */ {-0.278f, 0.079f, -179.0f},  // entrada vuelta 90° (esquina inf. izq.)
    /* P14 */ {-0.372f, 0.193f,   93.0f},  // salida = INICIO y FIN de la vuelta
};
