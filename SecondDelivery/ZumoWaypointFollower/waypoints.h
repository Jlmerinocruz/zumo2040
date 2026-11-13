// Tabla de waypoints de la pista (Second Delivery, parte B).
//
// Coordenadas medidas con el sistema de posicionamiento local (cámara del techo),
// colocando el Zumo sobre cada punto. Convención: x, y en metros; ángulo en grados,
// 0° = +x, 90° = +y (antihorario positivo).
//
// Cada vuelta de la pista tiene un waypoint a su entrada y otro a su salida:
//   P1→P2, P3→P4, P5→P6, P7→P8, P9→P10 : medias vueltas de 180° (R ≈ 7 cm)
//   P11→P12, P13→P14                    : vueltas de 90° (R ≈ 7 cm)
// Entre ellas hay rectas. La vuelta inicia y termina en P14.
#ifndef WAYPOINTS_H
#define WAYPOINTS_H

typedef struct {
    float x;          // m
    float y;          // m
    float theta_deg;  // grados
} waypoint_t;

#define WAYPOINT_COUNT 14

// Índice (0-based) del waypoint de inicio/fin de la vuelta: P14.
#define WAYPOINT_START 13

extern const waypoint_t WAYPOINTS[WAYPOINT_COUNT];

#endif
