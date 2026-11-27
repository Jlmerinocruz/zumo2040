#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include <pololu_zumo_2040_robot.h>
#include "comm.h"
#include "encoders.pio.h"
#include "wheels.h"

// Pines de los encoders (los mismos que usa Pololu en encoders.py): canal A y B en
// pines consecutivos. pio1 lo ocupan los sensores de reflectancia de la librería.
#define ENC_PIO        pio0
#define ENC_LEFT_PIN   12
#define ENC_RIGHT_PIN  8
// Reporte de lazo cerrado (4/10): entre 7699 y 7950 cuentas/m según el perfil de
// velocidad. El error de escala lo absorbe g_v del seguidor.
#define COUNTS_PER_M   7950.0f
#define WHEELS_PERIOD_US 10000u
// La velocidad se mide sobre los últimos WIN periodos (40 ms): una cuenta ≈ 3 mm/s.
#define WIN            4
#define KP             3000.0f   // cuentas por m/s de error
#define KI             30000.0f  // cuentas por (m/s)·s: corrige en ~0.1 s
// Tope del integrador (≈ 0.17 m/s del modelo): alcanza para vencer la fricción de la
// oruga interior sin que el integrador se pase cuando el comando cambia rápido.
#define I_MAX          1200.0f
#define V_MIN          0.005f    // m/s: abajo de esto la oruga se apaga
#define STALL_V        0.005f    // m/s: oruga "parada"
#define STALL_REF      0.02f     // m/s: ... cuando se le pedía al menos esto

typedef struct {
    uint sm;
    float ref, integ;
    int32_t hist[WIN + 1];
    float err2;                  // estadísticas del recorrido
    uint32_t n, n_ask, n_stall;
} track_t;

static track_t tr[2];            // 0 = izquierda, 1 = derecha
static uint32_t t_hist[WIN + 1];
static int h_i;
static uint32_t last_us;
static float motor_k, motor_dead;
static bool closed;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

// Con la librería de Pololu ambos encoders cuentan negativo hacia adelante.
static int32_t read_counts(uint sm) {
    pio_sm_put_blocking(ENC_PIO, sm, 1);
    return -(int32_t)pio_sm_get_blocking(ENC_PIO, sm);
}

static float feedforward(float v) {
    return copysignf(motor_dead + motor_k * fabsf(v), v);
}

static void apply(float left, float right) {
    motors_set_speeds((int32_t)lroundf(clampf(left, -MOTORS_MAX_SPEED, MOTORS_MAX_SPEED)),
                      (int32_t)lroundf(clampf(right, -MOTORS_MAX_SPEED, MOTORS_MAX_SPEED)));
}

void wheels_init(float k, float dead, bool closed_loop) {
    wheels_set_model(k, dead);
    closed = closed_loop;
    pio_add_program_at_offset(ENC_PIO, &quadrature_counter_program, 0);
    const uint pins[2] = {ENC_LEFT_PIN, ENC_RIGHT_PIN};
    for (int i = 0; i < 2; i++) {
        tr[i].sm = pio_claim_unused_sm(ENC_PIO, true);
        quadrature_counter_program_init(ENC_PIO, tr[i].sm, pins[i]);
    }
    wheels_stop();
}

void wheels_set_model(float k, float dead) {
    motor_k = k;
    motor_dead = dead;
}

void wheels_set(float v_left, float v_right) {
    tr[0].ref = fabsf(v_left) < V_MIN ? 0.0f : v_left;
    tr[1].ref = fabsf(v_right) < V_MIN ? 0.0f : v_right;
    if (!closed) apply(tr[0].ref ? feedforward(tr[0].ref) : 0.0f,
                       tr[1].ref ? feedforward(tr[1].ref) : 0.0f);
}

void wheels_stop(void) {
    motors_set_speeds(0, 0);
    uint32_t now = time_us_32();
    for (int i = 0; i < 2; i++) {
        track_t *t = &tr[i];
        t->ref = t->integ = 0.0f;
        int32_t c = read_counts(t->sm);
        for (int j = 0; j <= WIN; j++) t->hist[j] = c;
    }
    for (int j = 0; j <= WIN; j++) t_hist[j] = now;
    h_i = 0;
    last_us = now;
}

void wheels_update(void) {
    uint32_t now = time_us_32();
    if (now - last_us < WHEELS_PERIOD_US) return;
    float dt = (now - last_us) / 1e6f;
    last_us = now;
    int old = (h_i + 1) % (WIN + 1);             // la muestra de hace WIN periodos
    float span = (now - t_hist[old]) / 1e6f;
    t_hist[old] = now;
    float u[2];
    for (int i = 0; i < 2; i++) {
        track_t *t = &tr[i];
        int32_t c = read_counts(t->sm);
        float v = (c - t->hist[old]) / COUNTS_PER_M / span;
        t->hist[old] = c;
        if (t->ref == 0.0f) { t->integ = 0.0f; u[i] = 0.0f; continue; }
        float e = t->ref - v;
        t->integ = clampf(t->integ + KI * e * dt, -I_MAX, I_MAX);
        u[i] = feedforward(t->ref) + KP * e + t->integ;
        t->err2 += e * e;
        t->n++;
        if (fabsf(t->ref) >= STALL_REF) {
            t->n_ask++;
            if (fabsf(v) < STALL_V) t->n_stall++;
        }
    }
    h_i = old;
    if (closed) apply(u[0], u[1]);
}

void wheels_counts(int32_t *left, int32_t *right) {
    *left = read_counts(tr[0].sm);
    *right = read_counts(tr[1].sm);
}

void wheels_print_stats(void) {
    static const char *name[2] = {"izq", "der"};
    for (int i = 0; i < 2; i++) {
        const track_t *t = &tr[i];
        log_printf("# oruga %s (%s): error RMS %.0f mm/s, parada %lu de %lu muestras en que debia avanzar\n",
                   name[i], closed ? "PI con encoder" : "lazo abierto",
                   t->n ? 1000.0f * sqrtf(t->err2 / t->n) : 0.0f,
                   (unsigned long)t->n_stall, (unsigned long)t->n_ask);
    }
    for (int i = 0; i < 2; i++) {
        tr[i].err2 = 0.0f;
        tr[i].n = tr[i].n_ask = tr[i].n_stall = 0;
    }
}
