#include "goto_wp.h"
#include <math.h>

// Un robot un poco antes del inicio de un tramo (t < 0) también cuenta como dentro,
// pero solo si ningún tramo lo contiene con t ≥ 0. Si no, al final de un arco
// (pegado al inicio de la recta siguiente) se elegiría la recta y el waypoint de
// salida del arco quedaría "atrás", forzando una vuelta completa para llegar a él.
#define GOTO_BEFORE_START_M 0.03f

int goto_locate(const follower_t *lap, const pose_t *pose) {
    int best = -1;
    bool best_before = true;
    float best_e = GOTO_ROUTE_TOL_M;
    for (int i = 0; i < lap->n_seg; i++) {
        const segment_t *s = &lap->seg[i];
        float t, e_y, th;
        follower_project(s, pose->x, pose->y, &t, &e_y, &th);
        // t < 1: sobre el waypoint final de un tramo, ese punto ya es el inicio del siguiente.
        if (t * s->length < -GOTO_BEFORE_START_M || t >= 1.0f || fabsf(e_y) >= GOTO_ROUTE_TOL_M)
            continue;
        bool before = t < 0.0f;
        if ((best_before && !before) || (before == best_before && fabsf(e_y) < best_e)) {
            best = i;
            best_before = before;
            best_e = fabsf(e_y);
        }
    }
    return best;
}

goto_result_t goto_plan(follower_t *f, const follower_params_t *p,
                        const pose_t *pose, int target) {
    if (target < 0 || target >= WAYPOINT_COUNT) return GOTO_BAD_TARGET;
    const waypoint_t *w = &WAYPOINTS[target];
    if (hypotf(pose->x - w->x, pose->y - w->y) < GOTO_ARRIVED_M) return GOTO_ALREADY_THERE;

    follower_init(f, p, WAYPOINTS, WAYPOINT_COUNT, 0);
    int seg = goto_locate(f, pose);
    if (seg < 0) return GOTO_OFF_ROUTE;

    int from = f->seg[seg].from;
    int n = (target - from + WAYPOINT_COUNT) % WAYPOINT_COUNT;
    if (n == 0) n = WAYPOINT_COUNT;   // el destino quedó atrás: vuelta completa
    follower_init(f, p, WAYPOINTS, WAYPOINT_COUNT, from);
    f->n_seg = n;
    return GOTO_OK;
}
