// Zumo 2040 — Second Delivery: seguimiento de waypoints con la cámara del techo.
//
// Flujo:
//   Pico 2 W --(CAN, LPdata x/y/θ)--> MCP2515 --> este programa --> motores
//
// Botones (el botón C NO se usa: comparte GP0 con el MISO del MCP2515):
//   A : arranca la vuelta (colocar el Zumo sobre P14 mirando hacia +y).
//   B : calibración con la cámara: avanza recto 1.5 s y luego gira en su lugar 1.5 s.
//       Imprime K (cuentas de motor por m/s) y la separación efectiva de orugas.
//       Copiar esos valores a MOTOR_K y WHEEL_BASE.
//
// Salida serial: líneas "L,..." en CSV para graficar la trayectoria
// (ver ../analysis/plot_route.py).
#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include <pololu_zumo_2040_robot.h>
#include "mcp2515.h"
#include "lp_can.h"
#include "follower.h"
#include "waypoints.h"

// ---- Parámetros a calibrar en pista (botón B) ----
#define MOTOR_K        8000.0f  // cuentas de motor (0..6000) por m/s
#define WHEEL_BASE     0.085f   // m, separación efectiva entre orugas
// ---- Seguridad y calibración ----
#define POSE_TIMEOUT_MS 300     // sin pose por este tiempo → se detiene
#define CAL_COUNTS     2000
#define CAL_MS         1500

static float motor_k = MOTOR_K;
static float wheel_base = WHEEL_BASE;

static pose_t last_pose;
static bool have_pose = false;
static uint32_t last_pose_ms = 0;

static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

// Lee todos los frames pendientes; devuelve true si llegó al menos una pose nueva.
static bool poll_pose(void) {
    bool fresh = false;
    while (mcp2515_message_available()) {
        can_frame_t rx;
        if (!mcp2515_read_message(&rx)) break;
        pose_t p;
        if (lp_decode(&rx, &p)) {
            last_pose = p;
            have_pose = true;
            last_pose_ms = now_ms();
            fresh = true;
        }
    }
    return fresh;
}

static void set_wheels(float v_left, float v_right) {
    motors_set_speeds((int32_t)lroundf(v_left * motor_k), (int32_t)lroundf(v_right * motor_k));
}

static void wait_release_a(void) { while (button_a_is_pressed()) sleep_ms(10); sleep_ms(50); }
static void wait_release_b(void) { while (button_b_is_pressed()) sleep_ms(10); sleep_ms(50); }

static void print_route(const follower_t *f) {
    printf("Ruta: %d tramos\n", f->n_seg);
    for (int i = 0; i < f->n_seg; i++) {
        const segment_t *s = &f->seg[i];
        if (s->type == SEG_LINE)
            printf("  %2d P%-2d->P%-2d recta  L=%.3f m\n", i, s->from + 1, s->to + 1, s->length);
        else
            printf("  %2d P%-2d->P%-2d arco   R=%.3f m  centro=(%.3f, %.3f)  %s %.0f deg\n",
                   i, s->from + 1, s->to + 1, s->r, s->cx, s->cy,
                   s->dir > 0 ? "izq" : "der", s->span * 57.2958f);
    }
}

// Corre las orugas a velocidad fija durante ms milisegundos, acumulando el giro
// medido por la cámara (desenvuelto) y la pose final.
static void drive_measure(int32_t left, int32_t right, uint32_t ms,
                          pose_t *start, pose_t *end, float *dtheta) {
    poll_pose();
    *start = last_pose;
    float prev_th = last_pose.theta, acc = 0.0f;
    motors_set_speeds(left, right);
    uint32_t t0 = now_ms();
    while (now_ms() - t0 < ms) {
        if (poll_pose()) {
            acc += follower_wrap_pi(last_pose.theta - prev_th);
            prev_th = last_pose.theta;
        }
        sleep_ms(2);
    }
    motors_set_speeds(0, 0);
    uint32_t t1 = now_ms();
    while (now_ms() - t1 < 400) {                 // esperar a que se asiente y llegue pose
        if (poll_pose()) { acc += follower_wrap_pi(last_pose.theta - prev_th); prev_th = last_pose.theta; }
        sleep_ms(2);
    }
    *end = last_pose;
    *dtheta = acc;
}

static void calibrate(void) {
    if (!have_pose) { printf("CAL: no hay pose de la camara, abortado\n"); return; }
    pose_t a, b; float dth;
    printf("CAL 1/2: recto %d cuentas por %d ms...\n", CAL_COUNTS, CAL_MS);
    sleep_ms(1000);
    drive_measure(CAL_COUNTS, CAL_COUNTS, CAL_MS, &a, &b, &dth);
    float d = hypotf(b.x - a.x, b.y - a.y);
    float v = d / (CAL_MS / 1000.0f);
    if (v > 0.02f) motor_k = CAL_COUNTS / v;
    printf("CAL: avanzo %.3f m -> v=%.3f m/s -> MOTOR_K=%.0f cuentas/(m/s)\n", d, v, motor_k);

    printf("CAL 2/2: giro en su lugar...\n");
    sleep_ms(1000);
    drive_measure(-CAL_COUNTS, CAL_COUNTS, CAL_MS, &a, &b, &dth);
    float w = dth / (CAL_MS / 1000.0f);
    float vw = CAL_COUNTS / motor_k;
    if (fabsf(w) > 0.2f) wheel_base = 2.0f * vw / fabsf(w);
    printf("CAL: giro %.1f deg -> w=%.2f rad/s -> WHEEL_BASE=%.3f m\n", dth * 57.2958f, w, wheel_base);
    printf("CAL: copiar a main.c: #define MOTOR_K %.0ff  #define WHEEL_BASE %.3ff\n", motor_k, wheel_base);
}

static void run_lap(void) {
    follower_params_t p = follower_default_params();
    p.wheel_base = wheel_base;
    static follower_t f;
    follower_init(&f, &p, WAYPOINTS, WAYPOINT_COUNT, WAYPOINT_START);

    printf("Vuelta: inicio en P%d. Arranca en 1 s...\n", WAYPOINT_START + 1);
    sleep_ms(1000);
    printf("L,t_ms,x,y,theta_deg,seg,t,ey_mm,vl,vr\n");

    uint32_t t0 = now_ms(), prev = t0;
    int last_seg = -1;
    while (true) {
        if (poll_pose()) {
            uint32_t t = now_ms();
            float vl, vr;
            bool running = follower_update(&f, &last_pose, (t - prev) / 1000.0f, &vl, &vr);
            prev = t;
            set_wheels(vl, vr);
            printf("L,%lu,%.3f,%.3f,%.1f,%d,%.2f,%.0f,%.3f,%.3f\n",
                   (unsigned long)(t - t0), last_pose.x, last_pose.y, last_pose.theta * 57.2958f,
                   f.cur, f.t, f.e_y * 1000.0f, vl, vr);
            if (running && f.cur != last_seg) {
                const segment_t *s = &f.seg[f.cur];
                printf("# tramo %d: P%d -> P%d (%s)\n", f.cur, s->from + 1, s->to + 1,
                       s->type == SEG_LINE ? "recta" : "arco");
                last_seg = f.cur;
            }
            if (!running) {
                motors_set_speeds(0, 0);
                printf("# FIN en P%d. Tiempo de vuelta: %.2f s\n", WAYPOINT_START + 1,
                       (t - t0) / 1000.0f);
                return;
            }
        }
        if (now_ms() - last_pose_ms > POSE_TIMEOUT_MS) {
            motors_set_speeds(0, 0);
            printf("# ALTO: sin pose de la camara por %d ms (tramo %d)\n", POSE_TIMEOUT_MS, f.cur);
            return;
        }
        if (button_b_is_pressed()) {              // paro de emergencia
            motors_set_speeds(0, 0);
            printf("# ALTO: boton B\n");
            wait_release_b();
            return;
        }
        sleep_ms(1);
    }
}

int main(void) {
    stdio_init_all();
    motors_init();
    sleep_ms(2000);

    printf("\n--- Zumo Waypoint Follower ---\n");
    mcp2515_init();
    mcp2515_set_bitrate_500kbps_8mhz();
    mcp2515_set_mode(MCP2515_MODE_NORMAL);

    static follower_t preview;
    follower_params_t p = follower_default_params();
    follower_init(&preview, &p, WAYPOINTS, WAYPOINT_COUNT, WAYPOINT_START);
    print_route(&preview);
    printf("A = correr vuelta, B = calibrar (y paro durante la vuelta)\n");

    uint32_t last_print = 0;
    while (true) {
        poll_pose();
        if (now_ms() - last_print > 500) {
            last_print = now_ms();
            if (have_pose && now_ms() - last_pose_ms < 1000)
                printf("pose: x=%.3f y=%.3f th=%.1f deg\n",
                       last_pose.x, last_pose.y, last_pose.theta * 57.2958f);
            else
                printf("esperando LPdata por CAN (ID 0x%03X)...\n", LP_CAN_ID);
        }
        if (button_a_is_pressed()) {
            wait_release_a();
            if (have_pose && now_ms() - last_pose_ms < POSE_TIMEOUT_MS) run_lap();
            else printf("No hay pose reciente; no arranco.\n");
        }
        if (button_b_is_pressed()) {
            wait_release_b();
            calibrate();
        }
        sleep_ms(5);
    }
}
