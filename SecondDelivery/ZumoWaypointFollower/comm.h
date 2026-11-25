// Núcleo 1 del Zumo: comunicación.
//
// Es el único que usa el MCP2515 (SPI0) y el único que escribe al serial USB, así
// que ni la espera del SPI ni un printf lento pueden frenar el lazo de control del
// núcleo 0. Los dos núcleos se comunican solo por dos colas queue_t del SDK
// (FIFO protegidas por un spinlock de hardware, sin bloquear al otro núcleo):
//
//   núcleo 1 --(cola de poses, con la hora de llegada)--> núcleo 0
//   núcleo 1 --(cola de WP_GOTO de la GUI)--------------> núcleo 0
//   núcleo 0 --(cola de telemetría: muestras y texto)----> núcleo 1 --> printf
//   núcleo 0 --(cola de frames CAN a enviar: WP_STATUS)--> núcleo 1 --> MCP2515
#ifndef COMM_H
#define COMM_H

#include <stdbool.h>
#include <stdint.h>
#include "follower.h"
#include "lp_can.h"

typedef struct {
    pose_t pose;
    uint32_t t_us;   // llegada al MCP2515 (interrupción en INT), time_us_32()
} pose_msg_t;

// Una línea "L,..." del log de la vuelta (la formatea el núcleo 1).
typedef struct {
    uint32_t t_ms;
    pose_t pose;
    int8_t seg;
    float t, e_y, v_left, v_right;
} lap_sample_t;

// Inicializa las colas y arranca el núcleo 1.
void comm_start(void);

// Toma la pose más reciente y descarta las anteriores que estuvieran en la cola
// (en una ráfaga de WiFi llegan 4-5 seguidas y solo importa la última).
// Devuelve false si no llegó ninguna desde la llamada anterior.
bool comm_take_pose(pose_msg_t *out);

// Poses que no se usaron porque llegó una más nueva antes de procesarlas.
uint32_t comm_replaced_poses(void);

// Último WP_GOTO de la GUI (1..14 = destino, 0 = alto); false si no llegó ninguno
// desde la llamada anterior.
bool comm_take_goto(uint8_t *waypoint);

// Manda WP_STATUS por CAN (lo envía el núcleo 1). Nunca bloquea: con la cola llena
// se descarta y el núcleo 1 lo avisa por serial.
void comm_send_status(wp_state_t state, uint8_t waypoint);

// Telemetría desde el núcleo 0. Nunca bloquea: si la cola está llena el mensaje se
// descarta y se cuenta (comm_dropped_logs).
void log_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void log_sample(const lap_sample_t *s);
uint32_t comm_dropped_logs(void);

#endif
