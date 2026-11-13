#include "alpha.h"
#include <math.h>

const float ALPHA_S[ALPHA_SENSORS] = {-0.045f, -0.010f, 0.0f, 0.010f, 0.045f};

float alpha_radius(const alpha_cal_t *cal, int sensor, float t_us) {
    const float *t = cal->t[sensor];
    int n = cal->n;
    // Fuera de rango: extrapola con el tramo extremo.
    int k = 0;
    while (k < n - 2 && t_us > t[k + 1]) k++;
    float dt = t[k + 1] - t[k];
    if (dt < 1e-3f) return cal->r[k];
    return cal->r[k] + (t_us - t[k]) * (cal->r[k + 1] - cal->r[k]) / dt;
}

bool alpha_estimate(const alpha_cal_t *cal, const float t_us[ALPHA_SENSORS],
                    float *alpha_rad, float *r0) {
    bool in_range = true;
    float r[ALPHA_SENSORS];
    float sm = 0.0f, rm = 0.0f;
    for (int i = 0; i < ALPHA_SENSORS; i++) {
        if (t_us[i] < cal->t[i][0] || t_us[i] > cal->t[i][cal->n - 1]) in_range = false;
        r[i] = alpha_radius(cal, i, t_us[i]);
        sm += ALPHA_S[i];
        rm += r[i];
    }
    sm /= ALPHA_SENSORS;
    rm /= ALPHA_SENSORS;
    float num = 0.0f, den = 0.0f;
    for (int i = 0; i < ALPHA_SENSORS; i++) {
        num += (ALPHA_S[i] - sm) * (r[i] - rm);
        den += (ALPHA_S[i] - sm) * (ALPHA_S[i] - sm);
    }
    float sin_a = num / den;
    if (sin_a > 1.0f)  sin_a = 1.0f;
    if (sin_a < -1.0f) sin_a = -1.0f;
    *alpha_rad = asinf(sin_a);
    *r0 = rm - sm * sin_a;
    return in_range;
}
