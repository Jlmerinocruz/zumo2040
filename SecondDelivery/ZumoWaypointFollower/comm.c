#include "comm.h"
#include <stdarg.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "mcp2515.h"
#include "lp_can.h"

#define POSE_QUEUE_LEN   8
#define LOG_QUEUE_LEN    32
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

// Cada contador lo escribe un solo núcleo; el otro solo lo lee (lectura de 32 bits atómica).
static volatile uint32_t pose_overflow;   // escribe el núcleo 1
static uint32_t pose_replaced;            // escribe el núcleo 0
static uint32_t log_dropped;              // escribe el núcleo 0

// ---------------------------------------------------------------- núcleo 0 ----

bool comm_take_pose(pose_msg_t *out) {
    int n = 0;
    while (queue_try_remove(&pose_q, out)) n++;
    if (n > 1) pose_replaced += n - 1;
    return n > 0;
}

uint32_t comm_replaced_poses(void) { return pose_replaced + pose_overflow; }

uint32_t comm_dropped_logs(void) { return log_dropped; }

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
    printf("# CAN: EFLG=0x%02X TEC=%u REC=%u CANSTAT=0x%02X\n",
           mcp2515_read_register(MCP2515_EFLG), mcp2515_read_register(MCP2515_TEC),
           mcp2515_read_register(MCP2515_REC), mcp2515_read_register(MCP2515_CANSTAT));
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
        if (lp_decode(&rx, &m.pose)) { push_pose(&m); got = true; }
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
    // En la PCB el MCP2515 se alimenta de VSW: con el switch del Zumo en OFF no
    // responde, así que el arranque se reintenta cada CHECK_PERIOD_MS.
    bool can_ready = mcp2515_start();
    if (!can_ready)
        printf("MCP2515 no responde: revisar switch del Zumo en ON (alimenta la PCB) y el header SPI\n");
    gpio_set_irq_enabled_with_callback(PINS.irq, GPIO_IRQ_EDGE_FALL, true, on_can_irq);

    uint32_t last_pose_ms = 0, last_check_ms = 0;
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

        log_msg_t m;
        if (queue_try_remove(&log_q, &m)) print_log(&m);
        else sleep_us(500);
    }
}

void comm_start(void) {
    queue_init(&pose_q, sizeof(pose_msg_t), POSE_QUEUE_LEN);
    queue_init(&log_q, sizeof(log_msg_t), LOG_QUEUE_LEN);
    mcp2515_init(PINS);
    multicore_launch_core1(core1_main);
}
