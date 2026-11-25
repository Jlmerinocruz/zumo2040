#include "follower.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD(d) ((float)(d) * (float)M_PI / 180.0f)
#define TWO_PI (2.0f * (float)M_PI)

float follower_wrap_pi(float a) {
    while (a > (float)M_PI)   a -= TWO_PI;
    while (a <= -(float)M_PI) a += TWO_PI;
    return a;
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

follower_params_t follower_default_params(void) {
    follower_params_t p = {
        .v_straight = 0.30f,
        .v_arc      = 0.15f,
        .accel      = 0.60f,
        .decel      = 0.60f,
        // Ganancias elegidas con el barrido del simulador (ver SecondDelivery/README.md):
        // k_theta más alta oscila por el retardo y la baja frecuencia de la cámara.
        // Cámara a ~8 Hz (reporte Wireless). Con retardo de 100 ms k_theta=5 va bien,
        // pero con 200 ms oscila; 4 / 0.8 aguanta ambos casos.
        .k_theta    = 4.0f,
        .zeta       = 0.8f,
        .k_e_max    = 30.0f,
        .omega_max  = 6.0f,
        .wheel_base = 0.085f,
        .wheel_max  = 0.60f,
        .latency    = 0.05f,
        .adapt      = true,
    };
    return p;
}

// Arco entre A (rumbo θa) y B (rumbo θb). El sentido de giro sale de en qué lado
// del rumbo inicial queda B (producto cruz); así funciona también para medias
// vueltas de 180°, donde la diferencia de rumbos no dice hacia dónde girar.
static void build_arc(segment_t *s, float ta, float tb) {
    float hx = cosf(ta), hy = sinf(ta);
    float dx = s->bx - s->ax, dy = s->by - s->ay;
    float cross = hx * dy - hy * dx;
    s->dir = (cross >= 0.0f) ? 1 : -1;

    // Normal hacia el centro desde cada waypoint: izquierda del rumbo si dir=+1.
    float nax = -sinf(ta) * s->dir, nay = cosf(ta) * s->dir;
    float nbx = -sinf(tb) * s->dir, nby = cosf(tb) * s->dir;
    // Centro = A + R·na = B + R·nb  →  R por mínimos cuadrados.
    float mx = nax - nbx, my = nay - nby;
    float den = mx * mx + my * my;
    float r = (den > 1e-6f) ? -((s->ax - s->bx) * mx + (s->ay - s->by) * my) / den
                            : 0.5f * sqrtf(dx * dx + dy * dy);
    if (r < 0.02f) r = 0.5f * sqrtf(dx * dx + dy * dy);
    s->r = r;
    s->cx = 0.5f * ((s->ax + r * nax) + (s->bx + r * nbx));
    s->cy = 0.5f * ((s->ay + r * nay) + (s->by + r * nby));

    s->phi0 = atan2f(s->ay - s->cy, s->ax - s->cx);
    float phi1 = atan2f(s->by - s->cy, s->bx - s->cx);
    float span = (phi1 - s->phi0) * s->dir;          // ángulo en el sentido de giro
    while (span <= 0.0f)   span += TWO_PI;
    while (span > TWO_PI)  span -= TWO_PI;
    s->span = span;
    s->length = r * span;
}

void follower_init(follower_t *f, const follower_params_t *p,
                   const waypoint_t *wp, int n_wp, int start_idx) {
    f->p = *p;
    f->n_seg = 0;
    for (int k = 0; k < n_wp && k < FOLLOWER_MAX_SEGMENTS; k++) {
        int i = (start_idx + k) % n_wp;
        int j = (i + 1) % n_wp;
        segment_t *s = &f->seg[f->n_seg++];
        s->from = (uint8_t)i;
        s->to = (uint8_t)j;
        s->ax = wp[i].x; s->ay = wp[i].y;
        s->bx = wp[j].x; s->by = wp[j].y;
        float ta = DEG2RAD(wp[i].theta_deg), tb = DEG2RAD(wp[j].theta_deg);
        float dx = s->bx - s->ax, dy = s->by - s->ay;
        float len = sqrtf(dx * dx + dy * dy);
        if (fabsf(follower_wrap_pi(tb - ta)) < DEG2RAD(30)) {
            s->type = SEG_LINE;
            s->length = len;
            s->ux = dx / len; s->uy = dy / len;
        } else {
            s->type = SEG_ARC;
            build_arc(s, ta, tb);
        }
    }
    f->cur = 0;
    f->done = false;
    f->v_cmd = f->omega_cmd = f->v_mot = f->w_mot = 0.0f;
    f->t = f->e_y = f->e_theta = 0.0f;
    f->x = f->y = f->th = 0.0f;
    f->est_ok = f->cam_ok = false;
    f->clock = f->cum_v = f->cum_w = 0.0f;
    f->h_i = f->h_n = 0;
    // Arranca en ganancia 1 con un peso equivalente a unos cuantos intervalos.
    f->nv = f->dv = FOLLOWER_PRIOR_V;
    f->nw = f->dw = FOLLOWER_PRIOR_W;
    f->g_v = f->g_w = 1.0f;
}

// Proyecta la pose sobre el tramo: avance t, error lateral e_y y rumbo de la tangente.
void follower_project(const segment_t *s, float x, float y,
                    float *t, float *e_y, float *path_theta) {
    if (s->type == SEG_LINE) {
        float px = x - s->ax, py = y - s->ay;
        *t = (px * s->ux + py * s->uy) / s->length;
        *e_y = s->ux * py - s->uy * px;             // + = a la izquierda
        *path_theta = atan2f(s->uy, s->ux);
    } else {
        float rx = x - s->cx, ry = y - s->cy;
        float d = sqrtf(rx * rx + ry * ry);
        float phi = atan2f(ry, rx);
        // Avance angular centrado a la mitad del arco, para que un robot un poco
        // antes del inicio no parezca haber terminado (ambigüedad de ±π).
        float half = 0.5f * s->span;
        float trav = follower_wrap_pi((phi - s->phi0) * s->dir - half) + half;
        *t = trav / s->span;
        *e_y = s->dir * (s->r - d);
        *path_theta = phi + s->dir * 0.5f * (float)M_PI;
    }
}

// Valor de las integrales de comando en el instante t (interpolando el historial).
static void hist_at(const follower_t *f, float t, float *cv, float *cw) {
    int n = f->h_n, newest = (f->h_i + FOLLOWER_HIST - 1) % FOLLOWER_HIST;
    *cv = f->hist[newest].v; *cw = f->hist[newest].w;
    for (int k = 1; k < n; k++) {
        const hist_t *b = &f->hist[(f->h_i + FOLLOWER_HIST - k) % FOLLOWER_HIST];
        const hist_t *a = &f->hist[(f->h_i + FOLLOWER_HIST - k - 1) % FOLLOWER_HIST];
        if (a->t <= t) {
            float u = (b->t > a->t) ? clampf((t - a->t) / (b->t - a->t), 0.0f, 1.0f) : 1.0f;
            *cv = a->v + u * (b->v - a->v);
            *cw = a->w + u * (b->w - a->w);
            return;
        }
        *cv = a->v; *cw = a->w;
    }
}

// Avanza el reloj dt segundos: integra la pose estimada con lo que el robot está
// haciendo (v_cmd, ω_cmd) y acumula lo que se mandó a las orugas (para el ajuste).
static void advance(follower_t *f, float dt) {
    f->clock += dt;
    float th = f->th + f->omega_cmd * dt * 0.5f;
    f->x += f->v_cmd * cosf(th) * dt;
    f->y += f->v_cmd * sinf(th) * dt;
    f->th = follower_wrap_pi(f->th + f->omega_cmd * dt);
    f->cum_v += f->v_mot * dt;
    f->cum_w += f->w_mot * dt;
    hist_t *h = &f->hist[f->h_i];
    h->t = f->clock; h->v = f->cum_v; h->w = f->cum_w;
    f->h_i = (f->h_i + 1) % FOLLOWER_HIST;
    if (f->h_n < FOLLOWER_HIST) f->h_n++;
}

// Cociente de sumas con olvido: real ≈ g·mandado. Devuelve la ganancia estimada.
// Se suman los intervalos (con el signo de lo mandado) en vez de pesarlos por
// mínimos cuadrados: así, si el instante de una foto se estima mal, lo que le
// sobra a un intervalo le falta al siguiente y el error se cancela.
static float fit_gain(float *num, float *den, float real, float sent, float g_min) {
    float sg = (sent > 0.0f) ? 1.0f : -1.0f;
    *num = FOLLOWER_FORGET * *num + real * sg;
    *den = FOLLOWER_FORGET * *den + sent * sg;
    return clampf(*num / *den, g_min, FOLLOWER_GAIN_MAX);
}

// Control a partir de la pose estimada (f->x, f->y, f->th).
static bool control(follower_t *f, float dt, float *v_left, float *v_right) {
    // Localizar en el tramo y avanzar mientras se haya pasado el final.
    float t, e_y, path_th;
    const segment_t *s;
    for (;;) {
        s = &f->seg[f->cur];
        follower_project(s, f->x, f->y, &t, &e_y, &path_th);
        if (t < 1.0f) break;
        f->cur++;
        if (f->cur >= f->n_seg) {           // llegó al waypoint final
            f->done = true;
            f->v_cmd = f->omega_cmd = f->v_mot = f->w_mot = 0.0f;
            f->t = 1.0f;
            return false;
        }
    }

    // Perfil de velocidad.
    float v_target;
    if (s->type == SEG_ARC) {
        v_target = f->p.v_arc;
    } else {
        float remain = (1.0f - clampf(t, 0.0f, 1.0f)) * s->length;
        float v_brake = sqrtf(f->p.v_arc * f->p.v_arc + 2.0f * f->p.decel * remain);
        v_target = fminf(f->p.v_straight, v_brake);
    }
    if (v_target > f->v_cmd) f->v_cmd = fminf(v_target, f->v_cmd + f->p.accel * dt);
    else                     f->v_cmd = v_target;   // frenar es inmediato
    float v = f->v_cmd;

    // Dirección: rumbo de referencia = tangente − atan(k_e·e_y).
    float k_e = f->p.k_theta / (4.0f * f->p.zeta * f->p.zeta * fmaxf(v, 0.05f));
    if (k_e > f->p.k_e_max) k_e = f->p.k_e_max;
    float th_ref = path_th - atanf(k_e * e_y);
    float e_th = follower_wrap_pi(th_ref - f->th);
    float omega_ff = (s->type == SEG_ARC) ? s->dir * v / s->r : 0.0f;
    float omega = clampf(omega_ff + f->p.k_theta * e_th, -f->p.omega_max, f->p.omega_max);
    f->omega_cmd = omega;

    // Lo que hay que mandarle a las orugas para que el robot haga (v, ω), según las
    // ganancias medidas con la cámara. (v, ω) → orugas; si una satura, se escalan
    // ambas para conservar la curvatura.
    float vm = v / f->g_v, wm = omega / f->g_w;
    float vl = vm - wm * f->p.wheel_base * 0.5f;
    float vr = vm + wm * f->p.wheel_base * 0.5f;
    float m = fmaxf(fabsf(vl), fabsf(vr));
    if (m > f->p.wheel_max) {
        float k = f->p.wheel_max / m;
        vl *= k; vr *= k; vm *= k; wm *= k;
        f->v_cmd = vm * f->g_v; f->omega_cmd = wm * f->g_w;
    }
    f->v_mot = vm; f->w_mot = wm;

    f->t = t; f->e_y = e_y; f->e_theta = e_th;
    *v_left = vl; *v_right = vr;
    return true;
}

bool follower_update(follower_t *f, const pose_t *pose, float dt,
                     float *v_left, float *v_right) {
    *v_left = *v_right = 0.0f;
    if (f->done) return false;
    advance(f, dt);

    // Ganancias reales: lo que la cámara vio moverse entre dos fotos contra lo que
    // se mandó a las orugas en ese mismo intervalo (desplazado por el retardo).
    float cv, cw;
    hist_at(f, f->clock - f->p.latency, &cv, &cw);
    // Los intervalos se juntan hasta que lo mandado pase de FOLLOWER_MIN_V/W: las
    // poses que llegan de dos en dos (5 ms entre ellas) o en ráfagas no cuadran con
    // el comando de su intervalo, pero sí con el de varios intervalos juntos.
    if (!f->cam_ok) {
        f->anc_v = *pose; f->anc_cv = cv;
        f->acc_th = 0.0f; f->anc_cw = cw;
    } else if (f->p.adapt) {
        f->acc_th += follower_wrap_pi(pose->theta - f->cam.theta);
        if (fabsf(cv - f->anc_cv) >= FOLLOWER_MIN_V) {
            float th_m = f->anc_v.theta + 0.5f * follower_wrap_pi(pose->theta - f->anc_v.theta);
            float d_s = (pose->x - f->anc_v.x) * cosf(th_m) + (pose->y - f->anc_v.y) * sinf(th_m);
            f->g_v = fit_gain(&f->nv, &f->dv, d_s, cv - f->anc_cv, FOLLOWER_GV_MIN);
            f->anc_v = *pose; f->anc_cv = cv;
        }
        // g_ω solo se ajusta en los arcos. En una recta las correcciones de rumbo son
        // pequeñas y caen en la zona muerta del giro (la prueba del 25/11 mandaba
        // diferencias constantes entre orugas sin que el Zumo girara), así que el
        // ajuste bajaba g_ω al mínimo y en el arco siguiente el comando se disparaba.
        if (f->seg[f->cur].type != SEG_ARC) {
            f->acc_th = 0.0f; f->anc_cw = cw;
        } else if (fabsf(cw - f->anc_cw) >= FOLLOWER_MIN_W) {
            f->g_w = fit_gain(&f->nw, &f->dw, f->acc_th, cw - f->anc_cw, FOLLOWER_GW_MIN);
            f->acc_th = 0.0f; f->anc_cw = cw;
        }
    }
    f->cam = *pose;
    f->cam_ok = true;

    // Compensación de retardo: se adelanta la pose con el último comando.
    float L = f->p.latency;
    f->th = pose->theta + f->omega_cmd * L * 0.5f;
    f->x = pose->x + f->v_cmd * cosf(f->th) * L;
    f->y = pose->y + f->v_cmd * sinf(f->th) * L;
    f->th = follower_wrap_pi(f->th + f->omega_cmd * L * 0.5f);
    f->est_ok = true;

    return control(f, dt, v_left, v_right);
}

bool follower_tick(follower_t *f, float dt, float *v_left, float *v_right) {
    *v_left = *v_right = 0.0f;
    if (f->done) return false;
    advance(f, dt);
    if (!f->est_ok) return true;            // aún no llega la primera pose
    return control(f, dt, v_left, v_right);
}

void follower_resume(follower_t *f) {
    f->v_cmd = f->omega_cmd = f->v_mot = f->w_mot = 0.0f;
    f->cam_ok = false;                       // no ajustar con un intervalo detenido
    f->est_ok = false;
}
