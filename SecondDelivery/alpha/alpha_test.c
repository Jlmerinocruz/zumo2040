// Prueba en PC del método de α (parte E).
//
//   make && ./alpha_test
//
// 1. Aplica el método a las lecturas reales de Stage1 (α = 0 en r = 1 m).
// 2. Simula el terreno con la geometría exacta y ruido de los sensores para ver
//    qué tan preciso es α según el radio. Usa la tabla de calibración actual,
//    que SOLO tiene 3 radios: hay que medir más (ver README.md).
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "alpha.h"

#define DEG (180.0f / 3.14159265f)

// Medianas de ZumoReflectanceStage1/mediciones.md (α = 0).
// r = 0.07: esquina blanca; r = 1.00: mediana de los 4 puntos del inciso g);
// r = 2.05: esquina oscura (S1/S5 afectados por el borde blanco del tapete).
static const alpha_cal_t CAL_STAGE1 = {
    .n = 3,
    .r = {0.07f, 1.00f, 2.05f},
    .t = {
        {70.0f, 88.5f, 464.0f},   // S1
        {52.0f, 68.0f, 384.0f},   // S2
        {39.0f, 49.5f, 242.0f},   // S3
        {53.0f, 67.0f, 293.0f},   // S4
        {76.0f, 101.0f, 397.0f},  // S5
    },
};

// Lectura que daría el sensor i a radio r (modelo directo, inverso de alpha_radius).
static float forward_t(const alpha_cal_t *c, int i, float r) {
    int k = 0;
    while (k < c->n - 2 && r > c->r[k + 1]) k++;
    return c->t[i][k] + (r - c->r[k]) * (c->t[i][k + 1] - c->t[i][k]) / (c->r[k + 1] - c->r[k]);
}

static float gauss(float s) {
    float u1 = (rand() + 1.0f) / (RAND_MAX + 2.0f), u2 = (rand() + 1.0f) / (RAND_MAX + 2.0f);
    return s * sqrtf(-2.0f * logf(u1)) * cosf(6.2831853f * u2);
}

int main(void) {
    const alpha_cal_t *cal = &CAL_STAGE1;

    printf("== 1. Datos reales de Stage1, r = 1.00 m, alfa real = 0 ==\n");
    const struct { const char *name; float t[5]; } real[] = {
        {"P3 (0.942,0.329)", {89, 64, 49, 67, 101}},
        {"P1 (0.80,0.60)  ", {86, 69, 50, 67, 101}},
        {"P4 (0.60,0.80)  ", {89, 67, 49, 64, 100}},
        {"P2 (0.30,0.95)  ", {88, 69, 52, 70, 104}},
    };
    for (unsigned k = 0; k < sizeof real / sizeof real[0]; k++) {
        float a, r0;
        bool ok = alpha_estimate(cal, real[k].t, &a, &r0);
        printf("  %s  ->  alfa = %6.1f deg   r0 = %.3f m %s\n", real[k].name, a * DEG, r0,
               ok ? "" : "(fuera de tabla)");
    }

    printf("\n== 2. Simulacion: geometria exacta + ruido sigma = 1.5 us, 500 lecturas ==\n");
    printf("  (con la tabla actual de 3 radios)\n");
    printf("   r0 (m) | alfa real | alfa medio | desv. est.\n");
    const float radii[] = {0.5f, 1.0f, 1.3f, 1.6f, 1.9f};
    const float alphas[] = {0.0f, 30.0f, 60.0f, 90.0f};
    srand(2);
    for (unsigned ir = 0; ir < 5; ir++) {
        for (unsigned ia = 0; ia < 4; ia++) {
            float r0 = radii[ir], al = alphas[ia] / DEG, sum = 0, sum2 = 0;
            const int N = 500;
            for (int n = 0; n < N; n++) {
                float t[5];
                for (int i = 0; i < 5; i++) {
                    float s = ALPHA_S[i];
                    float ri = sqrtf(r0 * r0 + 2.0f * r0 * s * sinf(al) + s * s);
                    t[i] = forward_t(cal, i, ri) + gauss(1.5f);
                }
                float a, rr;
                alpha_estimate(cal, t, &a, &rr);
                sum += a * DEG; sum2 += a * DEG * a * DEG;
            }
            float m = sum / N, sd = sqrtf(fmaxf(sum2 / N - m * m, 0));
            printf("   %5.2f  |  %5.0f    |  %7.1f   |  %5.1f\n", r0, alphas[ia], m, sd);
        }
    }
    return 0;
}
