#ifndef WHEELS_H
#define WHEELS_H

#include <stdbool.h>
#include <stdint.h>

// Velocidad de cada oruga. En lazo cerrado un PI por oruga, con los encoders, hace
// que cada oruga vaya a la velocidad mandada; en lazo abierto solo se usa el modelo
// del motor: cuentas = signo(v)·(dead + k·|v|).
//
// Por qué (vuelta del 26/11): en los arcos la oruga interior debe ir a ~0.03 m/s,
// pero con ~1000 cuentas el motor no vence la fricción al girar y se queda parada,
// arrastrada por la otra. El seguidor medía entonces muy poco giro (g_ω = 0.25) y
// pedía 4 veces más diferencia: la interior terminaba mandada en reversa (−0.09 m/s)
// en 84 de 106 poses de arco.

// closed_loop = false deja el lazo abierto de antes (para comparar en pista).
void wheels_init(float k, float dead, bool closed_loop);
void wheels_set_model(float k, float dead);     // después de calibrar (botón B)
void wheels_set(float v_left, float v_right);   // m/s
void wheels_stop(void);                         // motores en 0 y PI desde cero
// Corre el PI cada WHEELS_PERIOD_US; llamarla seguido (en el lazo del recorrido).
void wheels_update(void);
// Cuentas de los encoders (positivas hacia adelante), para revisar el signo a mano.
void wheels_counts(int32_t *left, int32_t *right);
// Resumen del recorrido: error RMS del PI y tiempo con la oruga parada cuando se
// le pedía avanzar (también en lazo abierto, para comparar). Después las reinicia.
void wheels_print_stats(void);

#endif
