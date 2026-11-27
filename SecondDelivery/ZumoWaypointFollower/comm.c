#include "comm.h"
#include <stdarg.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "mcp2515.h"
#include "lp_can.h"
#include "obstacle.h"

#define POSE_QUEUE_LEN   8
#define LOG_QUEUE_LEN    32
#define GOTO_QUEUE_LEN   4
#define TX_QUEUE_LEN     4
#define LOG_TEXT_LEN     128
#define CHECK_PERIOD_MS  500    // revisión del MCP2515 y mensajes de estado
#define NO_POSE_MS       1000   // sin pose por más de esto: avisar y mostrar errores CAN
#define MAX_FRAMES       4      // por pasada: si MISO quedara en alto (módulo sin energía)
                                // READ STATUS diría siempre "hay mensaje"

typedef enum { LOG_TEXT, LOG_SAMPLE } log_kind_t;

typedef struct {
    log_kind_t kind;
    union {
        char text[LOG_TEXT_LEN];
        lap_sample_t sample;
    };
} log_msg_t;

static queue_t pose_q;   // núcleo 1 -> núcleo 0
static queue_t log_q;    // núcleo 0 -> núcleo 1
static queue_t goto_q;   // núcleo 1 -> núcleo 0, uint8_t del WP_GOTO
static queue_t tx_q;     // núcleo 0 -> núcleo 1, can_frame_t a enviar

// Cada contador lo escribe un solo núcleo; el otro solo lo lee (lectura de 32 bits atómica).
static volatile uint32_t pose_overflow;   // escribe el núcleo 1
static uint32_t pose_replaced;            // escribe el núcleo 0
static uint32_t log_dropped;              // escribe el núcleo 0
// Obstáculo al frente: el núcleo 1 mide y avisa; el núcleo 0 decide si frena y
// prende la alerta (solo durante un recorrido). Variables de un byte, un solo escritor.
static volatile uint8_t front_level;      // escribe el núcleo 1
static volatile bool obstacle;            // escribe el núcleo 1
static volatile bool alert_on;            // escribe el núcleo 0

// ---------------------------------------------------------------- núcleo 0 ----

bool comm_take_pose(pose_msg_t *out) {
    int n = 0;
    while (queue_try_remove(&pose_q, out)) n++;
    if (n > 1) pose_replaced += n - 1;
    return n > 0;
}

uint32_t comm_replaced_poses(void) { return pose_replaced + pose_overflow; }

uint32_t comm_dropped_logs(void) { return log_dropped; }

bool comm_obstacle(void) { return obstacle; }

uint8_t comm_front_level(void) { return front_level; }

void comm_set_alert(bool on) { alert_on = on; }

bool comm_take_goto(uint8_t *waypoint) {
    bool got = false;
    while (queue_try_remove(&goto_q, waypoint)) got = true;   // solo cuenta el último
    return got;
}

void comm_send_status(wp_state_t state, uint8_t waypoint) {
    can_frame_t frame = wp_status_frame(state, waypoint);
    if (!queue_try_add(&tx_q, &frame)) log_printf("# WP_STATUS descartado: cola CAN llena\n");
}

static void log_push(const log_msg_t *m) {
    if (!queue_try_add(&log_q, m)) log_dropped++;
}

void log_printf(const char *fmt, ...) {
    log_msg_t m = { .kind = LOG_TEXT };
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(m.text, sizeof m.text, fmt, ap);
    va_end(ap);
    log_push(&m);
}

void log_sample(const lap_sample_t *s) {
    log_msg_t m = { .kind = LOG_SAMPLE, .sample = *s };
    log_push(&m);
}

// ---------------------------------------------------------------- núcleo 1 ----

static const mcp2515_pins_t PINS = MCP2515_PINS_ZUMO;

// La interrupción solo anota la hora: el SPI se usa únicamente desde el lazo del
// núcleo 1, así nunca hay dos accesos al MCP2515 a la vez.
static volatile uint32_t irq_t_us;
static void on_can_irq(uint gpio, uint32_t events) {
    (void)gpio; (void)events;
    irq_t_us = time_us_32();
}

static void print_can_errors(void) {
    // CNF2 distingue un MCP2515 sano sin tráfico (0x90 y errores en 0) de un MISO
    // muerto (todo se lee 0x00).
    printf("# CAN: EFLG=0x%02X TEC=%u REC=%u CANSTAT=0x%02X CNF2=0x%02X (debe ser 0x90)\n",
           mcp2515_read_register(MCP2515_EFLG), mcp2515_read_register(MCP2515_TEC),
           mcp2515_read_register(MCP2515_REC), mcp2515_read_register(MCP2515_CANSTAT),
           mcp2515_read_register(MCP2515_CNF2));
}

static void push_pose(const pose_msg_t *m) {
    // Cola llena (el núcleo 0 no ha leído): se tira la más vieja, nunca la nueva.
    if (!queue_try_add(&pose_q, m)) {
        pose_msg_t old;
        if (queue_try_remove(&pose_q, &old)) pose_overflow++;
        queue_try_add(&pose_q, m);
    }
}

// Lee los mensajes pendientes; devuelve true si llegó al menos una pose.
static bool read_frames(void) {
    bool got = false;
    uint32_t now = time_us_32();
    // Si no hubo flanco reciente (p. ej. el mensaje ya estaba al habilitar la
    // interrupción) se usa la hora actual.
    uint32_t t = (now - irq_t_us < 50000u) ? irq_t_us : now;
    can_frame_t rx;
    for (int n = 0; n < MAX_FRAMES && mcp2515_read_message(&rx); n++) {
        pose_msg_t m = { .t_us = t };
        uint8_t wp;
        if (lp_decode(&rx, &m.pose)) { push_pose(&m); got = true; }
        else if (wp_goto_decode(&rx, &wp) && !queue_try_add(&goto_q, &wp))
            printf("# WP_GOTO %u descartado: cola llena\n", wp);
    }
    return got;
}

static void print_log(const log_msg_t *m) {
    if (m->kind == LOG_TEXT) { fputs(m->text, stdout); return; }
    const lap_sample_t *s = &m->sample;
    printf("L,%lu,%.3f,%.3f,%.1f,%d,%.2f,%.0f,%.3f,%.3f\n",
           (unsigned long)s->t_ms, s->pose.x, s->pose.y, s->pose.theta * 57.2958f,
           s->seg, s->t, s->e_y * 1000.0f, s->v_left, s->v_right);
}

static void core1_main(void) {
    obstacle_init();
    // En la PCB el MCP2515 se alimenta de VSW: con el switch del Zumo en OFF no
    // responde, así que el arranque se reintenta cada CHECK_PERIOD_MS.
    bool can_ready = mcp2515_start();
    if (!can_ready)
        printf("MCP2515 no responde: revisar switch del Zumo en ON (alimenta la PCB) y el header SPI\n");
    gpio_set_irq_enabled_with_callback(PINS.irq, GPIO_IRQ_EDGE_FALL, true, on_can_irq);

    uint32_t last_pose_ms = 0, last_check_ms = 0, last_obst_ms = 0;
    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (can_ready && !gpio_get(PINS.irq) && read_frames()) last_pose_ms = now;

        if (now - last_check_ms >= CHECK_PERIOD_MS) {
            last_check_ms = now;
            if (!can_ready) {
                can_ready = mcp2515_start();
                printf(can_ready ? "MCP2515 listo\n" : "MCP2515 no responde: switch del Zumo en ON?\n");
            } else if (!mcp2515_in_normal_mode()) {
                // Un bajón en la alimentación del MCP2515 lo reinicia en modo
                // configuración: deja de recibir y de dar ACK. Se reconfigura.
                printf("# MCP2515 fuera de modo normal (CANSTAT=0x%02X): reconfigurando\n",
                       mcp2515_read_register(MCP2515_CANSTAT));
                can_ready = mcp2515_start();
            } else if (now - last_pose_ms > NO_POSE_MS) {
                printf("esperando LPdata por CAN (ID 0x%03X)...\n", LP_CAN_ID);
                print_can_errors();
            }
        }

        if (now - last_obst_ms >= OBST_PERIOD_MS) {
            last_obst_ms = now;
            front_level = obstacle_front_level();
            obstacle = obstacle_detect(front_level, now);
        }
        obstacle_alert(alert_on, now);

        can_frame_t tx;
        if (queue_try_remove(&tx_q, &tx) && !(can_ready && mcp2515_send_message(&tx)))
            printf("# CAN 0x%03lX sin ACK de la Pico\n", (unsigned long)tx.id);

        log_msg_t m;
        if (queue_try_remove(&log_q, &m)) print_log(&m);
        else sleep_us(500);
    }
}

void comm_start(void) {
    queue_init(&pose_q, sizeof(pose_msg_t), POSE_QUEUE_LEN);
    queue_init(&log_q, sizeof(log_msg_t), LOG_QUEUE_LEN);
    queue_init(&goto_q, sizeof(uint8_t), GOTO_QUEUE_LEN);
    queue_init(&tx_q, sizeof(can_frame_t), TX_QUEUE_LEN);
    mcp2515_init(PINS);
    multicore_launch_core1(core1_main);
}
