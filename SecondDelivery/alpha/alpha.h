// Parte E: estimación del ángulo α con los 5 sensores de reflectancia.
//
// α = ángulo entre la fila de sensores y la tangente al arco de radio constante
// que pasa por el sensor central (S3). El terreno se oscurece con el radio r medido
// desde la esquina blanca, así que cada lectura indica a qué radio está su sensor.
//
// Pasos:
//   1. Calibración (una vez): con α = 0 (fila tangente al arco, todos los sensores
//      al mismo r) se leen los 5 tiempos en varios radios. Se guarda una tabla
//      t_i(r) POR SENSOR: cada sensor tiene ganancia distinta (S1/S5 leen ~1.8× S3).
//   2. Cada lectura t_i se convierte a radio r_i invirtiendo la tabla del sensor i
//      (interpolación lineal por tramos; la tabla debe crecer con r).
//   3. Geometría: el sensor i está a s_i metros de S3 sobre la fila. Si la fila forma
//      un ángulo α con la tangente, su componente radial es s_i·sin α:
//          r_i = r0 + s_i·sin α        (exacto hasta O(s²/2r) ≈ 1 mm a r = 1 m)
//   4. Se ajusta la recta r_i vs s_i por mínimos cuadrados con los 5 sensores:
//          sin α = Σ (s_i − s̄)(r_i − r̄) / Σ (s_i − s̄)²,   r0 = r̄ − s̄·sin α
//   5. α = asin(sin α), en [−90°, +90°]. Positivo = el lado de S5 queda a mayor radio
//      (más oscuro). α y 180° − α dan la misma lectura (la fila no tiene "frente").
#ifndef ALPHA_H
#define ALPHA_H

#include <stdbool.h>

#define ALPHA_SENSORS   5
#define ALPHA_MAX_CAL   12

typedef struct {
    int n;                                  // número de radios calibrados
    float r[ALPHA_MAX_CAL];                 // m, creciente
    float t[ALPHA_SENSORS][ALPHA_MAX_CAL];  // µs de cada sensor en ese radio (α = 0)
} alpha_cal_t;

// Posición de cada sensor sobre la fila, S3 = 0 (m). Medido en Stage1 parte 1 c).
extern const float ALPHA_S[ALPHA_SENSORS];

// Convierte la lectura de un sensor a radio con su tabla de calibración.
float alpha_radius(const alpha_cal_t *cal, int sensor, float t_us);

// Estima α (rad) y el radio del sensor central r0 (m). Devuelve false si la lectura
// queda fuera de la tabla de calibración (el resultado no sería confiable).
bool alpha_estimate(const alpha_cal_t *cal, const float t_us[ALPHA_SENSORS],
                    float *alpha_rad, float *r0);

#endif
