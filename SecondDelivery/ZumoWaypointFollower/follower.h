// Seguidor de trayectoria por tramos (Second Delivery, partes C y D).
//
// C portable (sin Pico SDK): se compila igual en el Zumo y en el simulador de PC
// (../sim/sim_follower.c). Así lo que se ajusta en la simulación es exactamente
// lo que corre en el robot.
//
// Idea:
//   1. A partir de la tabla de waypoints se arma la ruta como una lista de tramos:
//      RECTA entre dos waypoints con el mismo rumbo, ARCO entre dos con rumbo distinto
//      (entrada y salida de cada vuelta). El centro y el radio del arco se calculan
//      con los dos waypoints medidos.
//   2. En cada medición nueva de la cámara (x, y, θ):
//      - se localiza el robot sobre el tramo actual (avance t ∈ [0,1], error lateral e_y);
//      - si t ≥ 1 se pasa al siguiente tramo, en orden y sin saltarse waypoints;
//      - el rumbo de referencia es la tangente del tramo, corregida por el error lateral;
//      - ω = ω_feedforward (v/R en arcos) + k_θ·(error de rumbo);
//      - la velocidad sale de un perfil: rápido en rectas, frena antes de cada arco;
//      - se convierte (v, ω) a velocidad de rueda izquierda/derecha.
//   3. Entre dos fotos de la cámara (huecos de hasta ~450 ms por WiFi) la pose se
//      estima integrando (v, ω) y el control se repite a tasa fija (follower_tick).
//   4. El Zumo no hace lo que se le manda: en las vueltas del 17/11 avanzaba ~1.3 veces
//      la v mandada en rectas y giraba ~0.3 veces la ω mandada en los arcos (las
//      orugas patinan de lado), por eso se abría 15–25 mm en todos los arcos. Con cada
//      pose se comparan el avance y el giro vistos por la cámara contra lo mandado y se
//      ajustan dos ganancias (g_v, g_ω) por mínimos cuadrados con olvido; el comando a
//      las orugas se divide entre ellas.
//   5. Al terminar el último tramo (llegar a P14) se detiene.
#ifndef FOLLOWER_H
#define FOLLOWER_H

#include <stdbool.h>
#include <stdint.h>
#include "waypoints.h"

typedef struct {
    float x;      // m
    float y;      // m
    float theta;  // rad, 0 = +x, antihorario positivo
} pose_t;

typedef enum { SEG_LINE = 0, SEG_ARC = 1 } seg_type_t;

typedef struct {
    seg_type_t type;
    uint8_t from, to;   // índices de waypoint (0-based)
    float ax, ay;       // punto inicial (waypoint "from")
    float bx, by;       // punto final (waypoint "to")
    float length;       // m (recta: |B-A|; arco: R·span)
    // Solo recta:
    float ux, uy;       // vector unitario A→B
    // Solo arco:
    float cx, cy, r;    // centro y radio
    int8_t dir;         // +1 = izquierda (antihorario), -1 = derecha (horario)
    float phi0;         // ángulo polar del punto inicial respecto al centro
    float span;         // ángulo total recorrido (rad, > 0)
} segment_t;

typedef struct {
    float v_straight;   // m/s   velocidad máxima en rectas
    float v_arc;        // m/s   velocidad en arcos
    float accel;        // m/s²  aceleración máxima
    float decel;        // m/s²  frenado para llegar a v_arc al inicio del arco
    float k_theta;      // 1/s   ganancia de rumbo
    // Peso del error lateral: k_e = k_theta / (4·ζ²·v). Linealizando, el error
    // lateral cumple ë + k_θ·ė + k_θ·k_e·v·e = 0; con esta k_e el amortiguamiento
    // queda en ζ a cualquier velocidad (si k_e fuera fija, al acelerar oscilaría).
    float zeta;         // -    amortiguamiento deseado (0.7–1.0)
    float k_e_max;      // 1/m   límite de k_e a velocidades bajas
    float omega_max;    // rad/s límite de giro
    float wheel_base;   // m     separación entre orugas (medir en el Zumo)
    float wheel_max;    // m/s   velocidad máxima de una oruga
    float latency;      // s     retardo de la cámara a compensar (0 = no compensar)
    bool adapt;         // ajustar g_v y g_ω con la cámara
} follower_params_t;

#define FOLLOWER_MAX_SEGMENTS WAYPOINT_COUNT
#define FOLLOWER_HIST     32      // historial de comandos (≥ retardo a la tasa de follower_tick)
#define FOLLOWER_FORGET   0.95f   // olvido por pose (~20 poses ≈ 2 s de memoria)
#define FOLLOWER_PRIOR_V  0.01f   // peso inicial de g_v = 1 (m)
#define FOLLOWER_PRIOR_W  0.1f    // peso inicial de g_ω = 1 (rad)
#define FOLLOWER_MIN_V    0.01f   // m   avance mandado que junta un intervalo de ajuste
#define FOLLOWER_MIN_W    0.1f    // rad giro mandado que junta un intervalo de ajuste
// Límites de las ganancias. g_v no baja de 0.5 para que, si el Zumo se atora y no
// avanza, no se le mande cada vez más velocidad. g_ω llegaba a ~0.25 en los arcos de
// las vueltas del 17/11; con 0.15 la prueba del 25/11 saturaba las orugas, así que no
// se deja amplificar el giro más de 4 veces.
#define FOLLOWER_GV_MIN   0.5f
#define FOLLOWER_GW_MIN   0.25f
#define FOLLOWER_GAIN_MAX 2.0f

typedef struct { float t, v, w; } hist_t;

typedef struct {
    follower_params_t p;
    segment_t seg[FOLLOWER_MAX_SEGMENTS];
    int n_seg;
    int cur;            // tramo actual
    bool done;
    float v_cmd;        // v que se le pide al robot (con rampa)
    float omega_cmd;    // ω que se le pide al robot
    float v_mot, w_mot; // lo que se manda a las orugas: v_cmd/g_v, omega_cmd/g_w
    float x, y, th;     // pose estimada en este instante (cámara + retardo + integración)
    bool est_ok;        // ya hay pose estimada
    // Ajuste de ganancias reales (real ≈ g·mandado):
    float g_v, g_w;
    float nv, dv, nw, dw;          // sumas de mínimos cuadrados
    float clock;                   // s desde follower_init
    float cum_v, cum_w;            // ∫v_mot dt, ∫w_mot dt
    hist_t hist[FOLLOWER_HIST];    // (clock, cum_v, cum_w) para leerlas con retardo
    int h_i, h_n;
    pose_t cam;                    // pose anterior de la cámara (para sumar Δθ)
    pose_t anc_v;                  // pose donde empieza el intervalo que se junta para g_v
    float anc_cv, anc_cw;          // integrales al inicio de los intervalos de g_v y g_ω
    float acc_th;                  // giro visto desde el inicio del intervalo de g_ω
    bool cam_ok;
    // Diagnóstico de la última actualización:
    float t;            // avance en el tramo actual [0..1]
    float e_y;          // error lateral (m), + = robot a la izquierda de la ruta
    float e_theta;      // error de rumbo (rad)
} follower_t;

// Parámetros por defecto (punto de partida; ajustar con el simulador y en pista).
follower_params_t follower_default_params(void);

// Arma los tramos de una vuelta completa empezando en el waypoint start_idx.
void follower_init(follower_t *f, const follower_params_t *p,
                   const waypoint_t *wp, int n_wp, int start_idx);

// Procesa una pose nueva de la cámara. dt = tiempo desde la llamada anterior a
// follower_update o follower_tick (s). Devuelve las velocidades de las orugas en m/s.
// Devuelve false cuando terminó la vuelta (en ese caso v_left = v_right = 0).
bool follower_update(follower_t *f, const pose_t *pose, float dt,
                     float *v_left, float *v_right);

// Entre poses: integra la pose estimada dt segundos y recalcula el comando. Se llama a
// tasa fija (p. ej. cada 20 ms) mientras no haya pose nueva.
bool follower_tick(follower_t *f, float dt, float *v_left, float *v_right);

// Después de detener los motores (pausa por falta de pose): arranca desde v = 0 y
// espera la siguiente pose de la cámara antes de volver a estimar.
void follower_resume(follower_t *f);

// Proyecta (x, y) sobre un tramo: avance t (0 = inicio, 1 = fin), error lateral e_y
// (+ = a la izquierda de la ruta) y rumbo de la tangente.
void follower_project(const segment_t *s, float x, float y,
                      float *t, float *e_y, float *path_theta);

float follower_wrap_pi(float a);

#endif
