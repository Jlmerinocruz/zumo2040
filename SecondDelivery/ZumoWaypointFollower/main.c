// Zumo 2040 — Second Delivery: seguimiento de waypoints con la cámara del techo.
//
// Flujo:
//   Pico 2 W --(CAN, LPdata x/y/θ)--> MCP2515 --> núcleo 1 --(cola)--> núcleo 0 --> motores
//
// Reparto entre núcleos (ver comm.h):
//   núcleo 0: lazo de control (follower.c), motores, botones, calibración y seguridad.
//   núcleo 1: MCP2515 por SPI (con la hora de llegada de cada pose) y salida serial.
// El programa corre desde RAM (copy_to_ram en CMakeLists.txt): leer el botón B
// desconecta un momento la flash (comparte pin con su CS) y el otro núcleo no puede
// estar leyendo instrucciones de ella en ese instante.
//
// Botones (el botón C NO se usa: comparte GP0 con el MISO del MCP2515):
//   A : arranca la vuelta (colocar el Zumo sobre P14 mirando hacia +y; a ≤ 10 cm).
//   B durante un recorrido: paro.
//   B (mantener 3 s): calibración con la cámara: avanza recto 1.5 s y luego gira 1.5 s.
//       Imprime K (cuentas de motor por m/s) y la separación efectiva de orugas.
//       Copiar esos valores a MOTOR_K y WHEEL_BASE.
//
// Obstáculos (función innovadora): si el sensor de proximidad del frente ve algo a
// ~10-15 cm durante un recorrido (vuelta o GOTO), el Zumo frena, prende los
// intermitentes del frente y pita; cuando se quita, sigue la ruta. Umbral en obstacle.h.
//
// GUI de la PC (función innovadora): CAN WP_GOTO (0x110) = ir a un waypoint siguiendo
// la pista o alto; el Zumo responde con WP_STATUS (0x111). Ver lp_can.h y ../gui/.
//
// Salida serial: líneas "L,..." en CSV para graficar la trayectoria
// (ver ../analysis/plot_route.py).
#include <math.h>
#include "pico/stdlib.h"
#include <pololu_zumo_2040_robot.h>
#include "comm.h"
#include "follower.h"
#include "goto_wp.h"
#include "obstacle.h"
#include "waypoints.h"

// ---- Parámetros a calibrar en pista (botón B) ----
// Modelo del motor: cuentas = signo(v)·(MOTOR_DEAD + MOTOR_K·|v|).
// MOTOR_DEAD es la zona muerta: con el peso del power bank el Zumo no se mueve
// a 600 cuentas y sí a 900 (reporte Wireless, problema 5; en la parte A 900 también
// avanzó). Por ahora la Pico va con power bank, así que se parte de 850; el valor
// real lo mide la calibración (botón B). Sin compensarla, la oruga interior de cada
// vuelta (≈0.06 m/s) quedaría parada.
#define MOTOR_K        7000.0f  // cuentas por m/s, arriba de la zona muerta
#define MOTOR_DEAD     850.0f   // cuentas para vencer la fricción
// Ya no hace falta que MOTOR_K y WHEEL_BASE sean exactos: el seguidor mide con la
// cámara cuánto avanza y gira el Zumo de verdad (g_v, g_w) y corrige el comando.
#define WHEEL_BASE     0.085f   // m, separación geométrica entre orugas
// Retardo cámara + arranque del motor. En el simulador 0.16 s funcionó mejor que 0.12
// (sirve entre 0.12 y 0.20); la calibración (botón B) imprime el valor medido.
#define CAM_LATENCY    0.16f    // s
// Ajuste de g_v y g_ω con la cámara. Si en pista se porta raro, poner false: queda el
// control anterior (más el control entre poses) para comparar.
#define ADAPT_GAINS    true
// ---- Velocidades (parte D: subirlas poco a poco cuando la vuelta salga completa) ----
// Prueba 17/11: a 0.20 m/s la cámara dejó de reconocer el marcador en cuanto el Zumo
// se movió (imagen movida). En la parte A (~0.05 m/s) sí lo seguía. Se empieza lento
// y se sube poco a poco mientras la cámara lo siga viendo. Ojo: antes el Zumo iba
// ~1.3 veces más rápido de lo mandado en rectas (0.08 → ~0.10 m/s reales) y ahora va
// a lo que se manda. Prueba 26/11 a 0.15 / 0.10: vuelta de 50.8 s sin perder la pose;
// el equipo se quedó con esta velocidad.
#define V_STRAIGHT     0.15f    // m/s en rectas
#define V_ARC          0.10f    // m/s en las vueltas
// ---- Seguridad y calibración ----
// En el laboratorio la cámara manda ~9.5 poses/s (una cada ~105 ms), pero por WiFi
// llegan en ráfagas: cada ~0.8 s hay un hueco de ~450 ms y luego 4-5 poses atrasadas
// en ~10 ms (vueltas 1 y 2 del 17/11: 61-68 huecos, ~30 s detenido por vuelta).
// Durante un hueco de hasta POSE_TIMEOUT_MS el Zumo sigue con el último comando
// (en un arco, ω = v/R mantiene la curva); si se pasa, se detiene y espera a que
// vuelva la pose. Si no vuelve en POSE_ABORT_MS, termina la vuelta.
#define POSE_TIMEOUT_MS 800
#define BURST_WAIT_US  20000u   // tiempo para juntar la ráfaga y usar solo la última pose
#define TICK_US        20000u   // periodo del control entre poses (follower_tick)
#define POSE_ABORT_MS  3000
// Distancia máxima a P14 para aceptar el arranque (evita arrancar mal colocado).
#define START_TOL_M    0.10f
#define CAL_C1         1500     // dos velocidades para obtener K y la zona muerta
#define CAL_C2         3000
#define CAL_MS         1500
static float motor_k = MOTOR_K;
static float motor_dead = MOTOR_DEAD;
static float wheel_base = WHEEL_BASE;

// Última pose que el núcleo 0 tomó de la cola (solo la usa el núcleo 0).
static pose_t last_pose;
static bool have_pose = false;
static uint32_t last_pose_us = 0;   // hora de llegada al MCP2515
// WP_GOTO recibido y aún no atendido: -1 = ninguno, 0 = alto, 1..14 = destino.
static int goto_req = -1;

static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }
static uint32_t pose_age_ms(void) { return (time_us_32() - last_pose_us) / 1000u; }

// Toma de la cola la pose más reciente; devuelve true si había una nueva.
static bool update_pose(void) {
    pose_msg_t m;
    if (!comm_take_pose(&m)) return false;
    last_pose = m.pose;
    last_pose_us = m.t_us;
    have_pose = true;
    return true;
}

static void update_goto(void) {
    uint8_t wp;
    if (comm_take_goto(&wp)) goto_req = wp;
}

static int32_t to_counts(float v) {
    if (fabsf(v) < 0.005f) return 0;
    float c = motor_dead + motor_k * fabsf(v);
    return (int32_t)lroundf(v > 0 ? c : -c);
}

static void set_wheels(float v_left, float v_right) {
    motors_set_speeds(to_counts(v_left), to_counts(v_right));
}

static void wait_release_a(void) { while (button_a_is_pressed()) sleep_ms(10); sleep_ms(50); }
static void wait_release_b(void) { while (button_b_is_pressed()) sleep_ms(10); sleep_ms(50); }

static void print_route(const follower_t *f) {
    log_printf("Ruta: %d tramos\n", f->n_seg);
    for (int i = 0; i < f->n_seg; i++) {
        const segment_t *s = &f->seg[i];
        if (s->type == SEG_LINE)
            log_printf("  %2d P%-2d->P%-2d recta  L=%.3f m\n", i, s->from + 1, s->to + 1, s->length);
        else
            log_printf("  %2d P%-2d->P%-2d arco   R=%.3f m  centro=(%.3f, %.3f)  %s %.0f deg\n",
                       i, s->from + 1, s->to + 1, s->r, s->cx, s->cy,
                       s->dir > 0 ? "izq" : "der", s->span * 57.2958f);
    }
}

// Resultados extra de drive_measure (para medir la cámara).
static uint32_t dm_first_move_ms;  // ms desde que arrancan los motores hasta ver > 8 mm
static int dm_poses;                // poses recibidas mientras se movía

// Corre las orugas a velocidad fija durante ms milisegundos, acumulando el giro
// medido por la cámara (desenvuelto) y la pose final.
static void drive_measure(int32_t left, int32_t right, uint32_t ms,
                          pose_t *start, pose_t *end, float *dtheta) {
    update_pose();
    *start = last_pose;
    float prev_th = last_pose.theta, acc = 0.0f;
    dm_first_move_ms = 0;
    dm_poses = 0;
    motors_set_speeds(left, right);
    uint32_t t0 = time_us_32();
    while (time_us_32() - t0 < ms * 1000u) {
        if (update_pose()) {
            dm_poses++;
            acc += follower_wrap_pi(last_pose.theta - prev_th);
            prev_th = last_pose.theta;
            if (!dm_first_move_ms && last_pose_us - t0 < ms * 1000u &&
                hypotf(last_pose.x - start->x, last_pose.y - start->y) > 0.008f)
                dm_first_move_ms = (last_pose_us - t0) / 1000u;
        }
        sleep_ms(2);
    }
    motors_set_speeds(0, 0);
    uint32_t t1 = now_ms();
    while (now_ms() - t1 < 400) {                 // esperar a que se asiente y llegue pose
        if (update_pose()) { acc += follower_wrap_pi(last_pose.theta - prev_th); prev_th = last_pose.theta; }
        sleep_ms(2);
    }
    *end = last_pose;
    *dtheta = acc;
}

static void calibrate(void) {
    if (!have_pose) { log_printf("CAL: no hay pose de la camara, abortado\n"); return; }
    pose_t a, b; float dth;
    // Dos rectas a velocidades distintas: la pendiente da K y la ordenada la zona muerta.
    // Nota: la primera va hacia +x/+y desde donde esté; dejar ~1 m libre al frente.
    float v[2];
    const int32_t c[2] = {CAL_C1, CAL_C2};
    for (int i = 0; i < 2; i++) {
        log_printf("CAL %d/3: recto %ld cuentas por %d ms...\n", i + 1, (long)c[i], CAL_MS);
        sleep_ms(1000);
        drive_measure(c[i], c[i], CAL_MS, &a, &b, &dth);
        float d = hypotf(b.x - a.x, b.y - a.y);
        v[i] = d / (CAL_MS / 1000.0f);
        log_printf("CAL: %ld cuentas -> avanzo %.3f m -> v=%.3f m/s\n", (long)c[i], d, v[i]);
        // Retardo efectivo = arranque del motor + retardo de la cámara (menos lo que
        // tarda en recorrer los 8 mm del umbral). Es el valor de CAM_LATENCY.
        if (dm_first_move_ms && v[i] > 0.02f)
            log_printf("CAL: camara %.1f poses/s, retardo efectivo ~%.0f ms\n",
                       dm_poses / (CAL_MS / 1000.0f), dm_first_move_ms - 8.0f / v[i]);
    }
    if (v[1] - v[0] > 0.02f) {
        motor_k = (CAL_C2 - CAL_C1) / (v[1] - v[0]);
        motor_dead = fmaxf(0.0f, CAL_C1 - motor_k * v[0]);
    }
    log_printf("CAL: MOTOR_K=%.0f cuentas/(m/s), MOTOR_DEAD=%.0f cuentas\n", motor_k, motor_dead);

    log_printf("CAL 3/3: giro en su lugar...\n");
    sleep_ms(1000);
    drive_measure(-CAL_C1, CAL_C1, CAL_MS, &a, &b, &dth);
    float w = dth / (CAL_MS / 1000.0f);
    float vw = (CAL_C1 - motor_dead) / motor_k;
    if (fabsf(w) > 0.2f && vw > 0.0f) wheel_base = 2.0f * vw / fabsf(w);
    log_printf("CAL: giro %.1f deg -> w=%.2f rad/s -> WHEEL_BASE=%.3f m\n", dth * 57.2958f, w, wheel_base);
    log_printf("CAL: copiar a main.c: MOTOR_K %.0ff  MOTOR_DEAD %.0ff  WHEEL_BASE %.3ff\n",
               motor_k, motor_dead, wheel_base);
}

// Mediciones del reparto entre núcleos durante una vuelta (evidencia de la prueba
// de integración multinúcleo).
typedef struct {
    uint32_t n;            // actualizaciones del seguidor
    uint32_t work_max_us;  // tiempo máximo de cálculo del núcleo 0 por pose
    uint32_t age_max_us;   // edad máxima de la pose al mandar el comando a los motores
    uint64_t age_sum_us;
    uint32_t replaced0, dropped0;
} lap_stats_t;

static void print_lap_stats(const lap_stats_t *st) {
    log_printf("# nucleo 0: %lu actualizaciones, calculo max %lu us, edad de la pose al usarla "
               "prom %lu ms / max %lu ms\n", (unsigned long)st->n, (unsigned long)st->work_max_us,
               (unsigned long)(st->n ? st->age_sum_us / st->n / 1000u : 0),
               (unsigned long)(st->age_max_us / 1000u));
    log_printf("# colas: %lu poses reemplazadas por una mas nueva, %lu mensajes de log perdidos\n",
               (unsigned long)(comm_replaced_poses() - st->replaced0),
               (unsigned long)(comm_dropped_logs() - st->dropped0));
}

// Termina un recorrido que llegó a su destino: motores en 0, ganancias y estadísticas.
static void finish_route(const follower_t *f, uint32_t route_us, const lap_stats_t *st, float *t_s) {
    motors_set_speeds(0, 0);
    *t_s = route_us / 1e6f;
    log_printf("# Ganancias medidas: g_v=%.2f g_w=%.2f\n", f->g_v, f->g_w);
    print_lap_stats(st);
}

// Parámetros del seguidor con lo calibrado en este Zumo.
static follower_params_t route_params(void) {
    follower_params_t p = follower_default_params();
    p.wheel_base = wheel_base;
    p.latency = CAM_LATENCY;
    p.adapt = ADAPT_GAINS;
    p.v_straight = V_STRAIGHT;
    p.v_arc = V_ARC;
    return p;
}

// Recorre los tramos ya armados en f hacia el waypoint dest (1..14). Devuelve true si
// llegó al final y false si se detuvo antes: botón B, sin pose de la cámara o un
// WP_GOTO nuevo (alto u otro destino, que el lazo principal atiende al regresar).
// Con un obstáculo al frente espera detenido. *t_s = duración del recorrido.
static bool run_route(follower_t *f, uint8_t dest, float *t_s) {
    log_printf("L,t_ms,x,y,theta_deg,seg,t,ey_mm,vl,vr\n");

    update_pose();                                 // descarta lo que llegó durante la espera
    uint32_t t0 = time_us_32(), call_us = t0;
    lap_stats_t st = { .replaced0 = comm_replaced_poses(), .dropped0 = comm_dropped_logs() };
    int last_seg = -1;
    bool pending = false, paused = false, blocked = false;
    uint32_t clear_us = t0;                        // cuándo se quitó el último obstáculo
    while (true) {
        pending |= update_pose();
        update_goto();
        if (comm_obstacle() != blocked) {
            blocked = !blocked;
            comm_set_alert(blocked);
            if (blocked) {
                motors_set_speeds(0, 0);
                paused = true;                     // al quitarlo arranca con la siguiente pose
                log_printf("# OBSTACULO al frente (nivel %u de %d, tramo %d): detenido\n",
                           comm_front_level(), OBST_LEVELS, f->cur);
                comm_send_status(WP_OBSTACLE, dest);
            } else {
                clear_us = time_us_32();
                log_printf("# obstaculo retirado\n");
                comm_send_status(WP_MOVING, dest);
            }
        }
        if (blocked) pending = false;              // no usar poses de mientras estaba detenido
        // Las poses llegan juntas: en ráfagas tras un hueco del WiFi y, en la prueba del
        // 25/11, de dos en dos (5 ms entre ellas) cada 200 ms. La primera de cada grupo
        // es más vieja de lo que supone la compensación de retardo, así que se espera
        // BURST_WAIT_US desde la última y solo se usa la más reciente.
        if (pending && time_us_32() - last_pose_us >= BURST_WAIT_US) {
            pending = false;
            uint32_t start_us = time_us_32();
            if (paused) {                          // volvió la pose: arrancar desde cero
                log_printf("# pose recuperada, sigue\n");
                paused = false;
                follower_resume(f);
                call_us = start_us;
            }
            float vl, vr;
            bool running = follower_update(f, &last_pose, (start_us - call_us) / 1e6f, &vl, &vr);
            call_us = start_us;
            set_wheels(vl, vr);
            uint32_t done_us = time_us_32();
            st.n++;
            if (done_us - start_us > st.work_max_us) st.work_max_us = done_us - start_us;
            if (done_us - last_pose_us > st.age_max_us) st.age_max_us = done_us - last_pose_us;
            st.age_sum_us += done_us - last_pose_us;

            lap_sample_t s = { .t_ms = (last_pose_us - t0) / 1000u, .pose = last_pose,
                               .seg = (int8_t)f->cur, .t = f->t, .e_y = f->e_y,
                               .v_left = vl, .v_right = vr };
            log_sample(&s);
            if (running && f->cur != last_seg) {
                const segment_t *sg = &f->seg[f->cur];
                log_printf("# tramo %d: P%d -> P%d (%s)\n", f->cur, sg->from + 1, sg->to + 1,
                           sg->type == SEG_LINE ? "recta" : "arco");
                last_seg = f->cur;
            }
            if (!running) { finish_route(f, last_pose_us - t0, &st, t_s); return true; }
        } else if (!paused && time_us_32() - call_us >= TICK_US) {
            // Entre poses se sigue con la pose estimada, para no ir a ciegas durante
            // los huecos del WiFi.
            uint32_t now = time_us_32();
            float vl, vr;
            bool running = follower_tick(f, (now - call_us) / 1e6f, &vl, &vr);
            call_us = now;
            set_wheels(vl, vr);
            if (!running) { finish_route(f, now - t0, &st, t_s); return true; }
        }
        // La cámara puede perder el marcador mientras se quita el obstáculo: el plazo
        // para abortar cuenta desde que se quitó.
        uint32_t lost = pose_age_ms(), since_clear = (time_us_32() - clear_us) / 1000u;
        if (lost > POSE_TIMEOUT_MS && !paused) {
            motors_set_speeds(0, 0);
            paused = true;
            log_printf("# PAUSA: sin pose de la camara por %d ms (tramo %d), esperando...\n",
                       POSE_TIMEOUT_MS, f->cur);
        }
        if (!blocked && lost > POSE_ABORT_MS && since_clear > POSE_ABORT_MS) {
            motors_set_speeds(0, 0);
            log_printf("# ALTO: sin pose de la camara por %d ms (tramo %d)\n", POSE_ABORT_MS, f->cur);
            print_lap_stats(&st);
            return false;
        }
        if (button_b_is_pressed()) {              // paro de emergencia
            motors_set_speeds(0, 0);
            comm_set_alert(false);
            log_printf("# ALTO: boton B\n");
            print_lap_stats(&st);
            wait_release_b();
            return false;
        }
        if (goto_req >= 0) {                       // la GUI mandó alto u otro destino
            motors_set_speeds(0, 0);
            comm_set_alert(false);
            log_printf("# ALTO: WP_GOTO %d desde la GUI\n", goto_req);
            print_lap_stats(&st);
            return false;
        }
        sleep_ms(1);
    }
}

static void run_lap(void) {
    follower_params_t p = route_params();
    static follower_t f;
    follower_init(&f, &p, WAYPOINTS, WAYPOINT_COUNT, WAYPOINT_START);

    log_printf("Vuelta: inicio en P%d, v_recta=%.2f v_arco=%.2f m/s, bat=%u mV, "
               "K=%.0f DEAD=%.0f WB=%.3f. Arranca en 1 s...\n",
               WAYPOINT_START + 1, V_STRAIGHT, V_ARC, battery_get_level_millivolts(),
               motor_k, motor_dead, wheel_base);
    sleep_ms(1000);
    float t_s;
    if (run_route(&f, WAYPOINT_START + 1, &t_s))
        log_printf("# FIN en P%d. Tiempo de vuelta: %.2f s\n", WAYPOINT_START + 1, t_s);
}

// Qué hace el Zumo al llegar al waypoint elegido en la GUI. PENDIENTE: el equipo
// todavía no lo define; por ahora solo lo avisa y parpadea el LED amarillo.
static void on_arrival(int waypoint) {
    log_printf("# LLEGO a P%d: accion pendiente de definir\n", waypoint);
    for (int i = 0; i < 3; i++) {
        yellow_led(1); sleep_ms(150);
        yellow_led(0); sleep_ms(150);
    }
}

// Va al waypoint (1..14) siguiendo la pista desde donde esté el Zumo.
static void run_goto(int waypoint) {
    static follower_t f;
    follower_params_t p = route_params();
    update_pose();
    if (!have_pose || pose_age_ms() > POSE_TIMEOUT_MS) {
        log_printf("GOTO P%d: no hay pose reciente; no arranco.\n", waypoint);
        comm_send_status(WP_REJECTED, (uint8_t)waypoint);
        return;
    }
    switch (goto_plan(&f, &p, &last_pose, waypoint - 1)) {
    case GOTO_OK:
        break;
    case GOTO_ALREADY_THERE:
        log_printf("GOTO P%d: ya estoy ahi.\n", waypoint);
        comm_send_status(WP_ARRIVED, (uint8_t)waypoint);
        on_arrival(waypoint);
        return;
    case GOTO_OFF_ROUTE:
        log_printf("GOTO P%d: estoy a mas de %.0f cm de la pista; no arranco.\n",
                   waypoint, GOTO_ROUTE_TOL_M * 100.0f);
        comm_send_status(WP_REJECTED, (uint8_t)waypoint);
        return;
    case GOTO_BAD_TARGET:
        log_printf("GOTO P%d: destino invalido (1..%d).\n", waypoint, WAYPOINT_COUNT);
        return;
    }
    log_printf("GOTO P%d: desde P%d, %d tramos.\n", waypoint, f.seg[0].from + 1, f.n_seg);
    comm_send_status(WP_MOVING, (uint8_t)waypoint);
    float t_s;
    if (run_route(&f, (uint8_t)waypoint, &t_s)) {
        log_printf("# FIN en P%d. Tiempo: %.2f s\n", waypoint, t_s);
        comm_send_status(WP_ARRIVED, (uint8_t)waypoint);
        on_arrival(waypoint);
    } else {
        comm_send_status(WP_STOPPED, (uint8_t)waypoint);
    }
}

int main(void) {
    stdio_init_all();
    motors_init();
    sleep_ms(2000);

    comm_start();
    log_printf("\n--- Zumo Waypoint Follower ---\n");

    static follower_t preview;
    follower_params_t p = follower_default_params();
    follower_init(&preview, &p, WAYPOINTS, WAYPOINT_COUNT, WAYPOINT_START);
    print_route(&preview);
    log_printf("A = correr vuelta, B 3 s = calibrar, B durante un recorrido = paro, "
               "GUI = ir a un waypoint (CAN 0x%03X)\n", WP_GOTO_ID);

    const waypoint_t *w0 = &WAYPOINTS[WAYPOINT_START];
    uint32_t last_print = 0;
    uint8_t last_front = 0;
    while (true) {
        update_pose();
        update_goto();
        if (comm_front_level() != last_front) {   // para calibrar OBST_STOP_LEVEL
            last_front = comm_front_level();
            log_printf("frente: nivel %u de %d\n", last_front, OBST_LEVELS);
        }
        // Sin pose reciente el núcleo 1 avisa "esperando LPdata" con los errores CAN.
        if (now_ms() - last_print > 500) {
            last_print = now_ms();
            if (have_pose && pose_age_ms() < 1000)
                log_printf("pose: x=%.3f y=%.3f th=%.1f deg | a P%d: %.0f mm | bat=%u mV\n",
                           last_pose.x, last_pose.y, last_pose.theta * 57.2958f, WAYPOINT_START + 1,
                           1000.0f * hypotf(last_pose.x - w0->x, last_pose.y - w0->y),
                           battery_get_level_millivolts());
        }
        if (button_a_is_pressed()) {
            wait_release_a();
            update_pose();                         // el núcleo 1 siguió recibiendo
            if (!have_pose || pose_age_ms() > POSE_TIMEOUT_MS)
                log_printf("No hay pose reciente; no arranco.\n");
            else if (hypotf(last_pose.x - w0->x, last_pose.y - w0->y) > START_TOL_M)
                log_printf("Muy lejos de P%d (%.2f, %.2f); colocarlo ahí mirando hacia +y.\n",
                           WAYPOINT_START + 1, w0->x, w0->y);
            else run_lap();
        }
        while (goto_req >= 0) {                    // WP_GOTO de la GUI (alto u otro destino)
            int wp = goto_req;
            goto_req = -1;
            if (wp == 0) comm_send_status(WP_IDLE, 0);
            else run_goto(wp);
        }
        // Calibrar solo si B se mantiene 3 s. Prueba 17/11 (vuelta 3): un toque de B en
        // reposo (pensando que era "paro") calibró con el Zumo mal colocado y dejó
        // WHEEL_BASE ≈ 0.19 m: toda la vuelta dio giros bruscos.
        if (button_b_is_pressed()) {
            uint32_t t = now_ms();
            while (button_b_is_pressed() && now_ms() - t < 3000) sleep_ms(10);
            if (now_ms() - t >= 3000) {
                log_printf("Calibrando (B 3 s). Suelta B.\n");
                wait_release_b();
                calibrate();
            } else {
                log_printf("B: para calibrar mantenlo 3 s (no se cambió nada)\n");
                sleep_ms(50);
            }
        }
        sleep_ms(5);
    }
}
