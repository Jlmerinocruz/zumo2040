// Ir a un waypoint elegido desde la GUI de la PC (función innovadora).
//
// El Zumo no corta camino: sigue la pista en el sentido de la vuelta desde el tramo
// donde está hasta el waypoint destino, con el mismo seguidor (follower.c) que la
// vuelta completa. Solo cambia dónde empieza y cuántos tramos recorre.
//
// C portable (sin Pico SDK): se prueba en la PC con ../sim/test_goto.c.
#ifndef GOTO_WP_H
#define GOTO_WP_H

#include "follower.h"

#define GOTO_ROUTE_TOL_M  0.10f   // distancia máxima a la pista para aceptar el destino
#define GOTO_ARRIVED_M    0.03f   // más cerca que esto del destino = ya está ahí

typedef enum {
    GOTO_OK = 0,          // ruta lista en el follower
    GOTO_ALREADY_THERE,   // el robot ya está sobre el destino
    GOTO_OFF_ROUTE,       // el robot está a más de GOTO_ROUTE_TOL_M de la pista
    GOTO_BAD_TARGET,      // índice fuera de 0..WAYPOINT_COUNT-1
} goto_result_t;

// Índice (0-based) del tramo de la vuelta (follower_init con start_idx = 0) en el
// que está la pose, o -1 si está a más de GOTO_ROUTE_TOL_M de la pista.
int goto_locate(const follower_t *lap, const pose_t *pose);

// Arma en f la ruta desde donde está el robot hasta el waypoint target (0-based)
// siguiendo la pista. Si el robot ya pasó el destino dentro de su tramo, da la
// vuelta completa para volver a él.
goto_result_t goto_plan(follower_t *f, const follower_params_t *p,
                        const pose_t *pose, int target);

#endif
