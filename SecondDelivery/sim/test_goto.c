// Prueba unitaria de goto_wp.c (ir a un waypoint elegido desde la GUI).
//
// Compila el MISMO goto_wp.c y follower.c que el Zumo. Cada caso imprime PASS o FAIL;
// el programa regresa 0 solo si todos pasan.
//   1. goto_locate: puntos sobre cada tramo (con ±2 cm de error lateral) se ubican
//      en ese tramo.
//   2. goto_plan: desde cada tramo hacia cada destino, la ruta empieza en el tramo
//      del robot, termina en el destino y no se salta waypoints.
//   3. Casos límite: destino inválido, robot ya en el destino, robot fuera de la pista.
//   4. Lazo cerrado: robot ideal con cámara a 8 Hz y 120 ms de retardo; desde la mitad
//      de cada tramo va a 3 destinos y debe detenerse a menos de ARRIVE_TOL_M.
#include <math.h>
#include <stdio.h>
#include "goto_wp.h"

#define ARRIVE_TOL_M 0.04f

static int n_pass = 0, n_fail = 0;

static void check(int ok, const char *name) {
    printf("%s  %s\n", ok ? "PASS" : "FAIL", name);
    if (ok) n_pass++; else n_fail++;
}

// Punto del tramo s en el avance u ∈ [0,1], desplazado e_y a la izquierda, y rumbo.
static pose_t point_on(const segment_t *s, float u, float e_y) {
    pose_t p;
    if (s->type == SEG_LINE) {
        p.theta = atan2f(s->uy, s->ux);
        p.x = s->ax + u * (s->bx - s->ax) - s->uy * e_y;
        p.y = s->ay + u * (s->by - s->ay) + s->ux * e_y;
    } else {
        float phi = s->phi0 + s->dir * u * s->span;
        float r = s->r - s->dir * e_y;
        p.x = s->cx + r * cosf(phi);
        p.y = s->cy + r * sinf(phi);
        p.theta = phi + s->dir * 1.5707963f;
    }
    return p;
}

static void test_locate(const follower_t *lap) {
    int ok = 1;
    for (int i = 0; i < lap->n_seg; i++)
        for (float u = 0.1f; u < 1.0f; u += 0.4f)
            for (float e = -0.02f; e <= 0.021f; e += 0.04f) {
                pose_t p = point_on(&lap->seg[i], u, e);
                int got = goto_locate(lap, &p);
                if (got != i) {
                    printf("      tramo %d u=%.1f e=%+.2f -> %d\n", i, u, e, got);
                    ok = 0;
                }
            }
    check(ok, "goto_locate ubica 84 puntos en su tramo");
}

static void test_plan(const follower_t *lap, const follower_params_t *prm) {
    int ok = 1;
    static follower_t f;
    for (int i = 0; i < lap->n_seg; i++) {
        pose_t p = point_on(&lap->seg[i], 0.5f, 0.0f);
        for (int target = 0; target < WAYPOINT_COUNT; target++) {
            if (goto_plan(&f, prm, &p, target) != GOTO_OK) { ok = 0; continue; }
            int from = lap->seg[i].from;
            int expect = (target - from + WAYPOINT_COUNT) % WAYPOINT_COUNT;
            if (expect == 0) expect = WAYPOINT_COUNT;
            int good = f.n_seg == expect && f.seg[0].from == from &&
                       f.seg[f.n_seg - 1].to == target;
            for (int k = 1; k < f.n_seg; k++) good &= f.seg[k].from == f.seg[k - 1].to;
            if (!good) {
                printf("      desde tramo %d a P%d: %d tramos, P%d -> P%d\n", i, target + 1,
                       f.n_seg, f.seg[0].from + 1, f.seg[f.n_seg - 1].to + 1);
                ok = 0;
            }
        }
    }
    check(ok, "goto_plan arma 196 rutas (14 tramos x 14 destinos) sin saltarse waypoints");
}

static void test_limits(const follower_params_t *prm) {
    static follower_t f;
    const waypoint_t *w = &WAYPOINTS[6];
    pose_t on_p7 = {w->x + 0.01f, w->y, 0.0f};
    pose_t far1 = {0.60f, 0.60f, 0.0f};   // fuera de la pista, a la derecha
    pose_t far2 = {0.00f, 1.30f, 0.0f};   // arriba de las vueltas de 180°
    check(goto_plan(&f, prm, &on_p7, -1) == GOTO_BAD_TARGET &&
          goto_plan(&f, prm, &on_p7, WAYPOINT_COUNT) == GOTO_BAD_TARGET,
          "destino fuera de 1..14 -> GOTO_BAD_TARGET");
    check(goto_plan(&f, prm, &on_p7, 6) == GOTO_ALREADY_THERE,
          "robot a 1 cm de P7 con destino P7 -> GOTO_ALREADY_THERE");
    check(goto_plan(&f, prm, &far1, 3) == GOTO_OFF_ROUTE &&
          goto_plan(&f, prm, &far2, 3) == GOTO_OFF_ROUTE,
          "robot lejos de la pista -> GOTO_OFF_ROUTE");
}

// Robot ideal (uniciclo) con cámara a 8 Hz y retardo de 120 ms. Devuelve la distancia
// final al destino, o -1 si no terminó en 120 s.
static float drive(const follower_params_t *prm, pose_t start, int target) {
    static follower_t f;
    if (goto_plan(&f, prm, &start, target) != GOTO_OK) return -1.0f;
    const float dt = 0.001f, cam_dt = 0.125f, delay = 0.12f;
    enum { QN = 8 };
    pose_t q[QN]; float qt[QN]; int qh = 0, qn = 0;
    float x = start.x, y = start.y, th = start.theta, vl = 0, vr = 0, next_cam = 0, last = 0;
    for (float t = 0.0f; t < 120.0f; t += dt) {
        if (t >= next_cam) {
            int i = (qh + qn) % QN;
            q[i] = (pose_t){x, y, th};
            qt[i] = t + delay;
            qn++;
            next_cam += cam_dt;
        }
        if (qn > 0 && t >= qt[qh]) {
            pose_t meas = q[qh];
            qh = (qh + 1) % QN; qn--;
            if (!follower_update(&f, &meas, t - last, &vl, &vr))
                return hypotf(x - WAYPOINTS[target].x, y - WAYPOINTS[target].y);
            last = t;
        }
        float v = 0.5f * (vl + vr), w = (vr - vl) / prm->wheel_base;
        x += v * cosf(th) * dt;
        y += v * sinf(th) * dt;
        th += w * dt;
    }
    return -1.0f;
}

static void test_closed_loop(const follower_t *lap, const follower_params_t *prm) {
    int ok = 1;
    float worst = 0.0f;
    for (int i = 0; i < lap->n_seg; i++) {
        pose_t p = point_on(&lap->seg[i], 0.5f, 0.0f);
        for (int k = 1; k <= 3; k++) {
            int target = (lap->seg[i].to + 4 * k) % WAYPOINT_COUNT;
            float d = drive(prm, p, target);
            if (d < 0.0f || d > ARRIVE_TOL_M) {
                printf("      desde tramo %d a P%d: %s %.1f mm\n", i, target + 1,
                       d < 0.0f ? "no llegó" : "se detuvo a", d * 1000.0f);
                ok = 0;
            }
            if (d > worst) worst = d;
        }
    }
    char name[96];
    snprintf(name, sizeof name, "lazo cerrado: 42 trayectos llegan a < %.0f mm (peor %.1f mm)",
             ARRIVE_TOL_M * 1000.0f, worst * 1000.0f);
    check(ok, name);
}

int main(void) {
    // Mismas velocidades y retardo que ZumoWaypointFollower/main.c.
    follower_params_t prm = follower_default_params();
    prm.v_straight = 0.08f;
    prm.v_arc = 0.06f;
    prm.latency = 0.12f;
    static follower_t lap;
    follower_init(&lap, &prm, WAYPOINTS, WAYPOINT_COUNT, 0);

    test_locate(&lap);
    test_plan(&lap, &prm);
    test_limits(&prm);
    test_closed_loop(&lap, &prm);
    printf("\n%d PASS, %d FAIL\n", n_pass, n_fail);
    return n_fail ? 1 : 0;
}
