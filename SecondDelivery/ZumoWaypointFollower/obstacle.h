// Obstáculos al frente: sensor de proximidad IR del frente, intermitentes (LEDs RGB
// del frente) y buzzer. Lo usa solo el núcleo 1 (ver comm.h): los LEDs RGB comparten
// SPI0 y GP3 con el MCP2515, así que nunca se escriben a la vez que el CAN.
//
// Cómo mide la distancia: los LEDs IR del frente se prenden con OBST_LEVELS brillos
// distintos y se cuenta con cuántos el sensor del frente (TSSP770) ve el reflejo.
// Nivel 0 = nada al frente; OBST_LEVELS = muy cerca. Se usa el mayor de los dos lados
// (LEDs izquierdos o derechos), así se ve también un obstáculo un poco de lado.
#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>

#define OBST_LEVELS      6
// ---- A calibrar en el lab ----
// Nivel a partir del cual se frena. Poner un objeto a ~12 cm frente al Zumo en reposo
// y leer en el serial "frente: nivel N"; usar ese N. Más alto = frena más cerca.
#define OBST_STOP_LEVEL  5
#define OBST_PERIOD_MS   50     // una lectura cada 50 ms (cada lectura tarda ~7 ms)
#define OBST_CONFIRM     2      // lecturas seguidas arriba del umbral para frenar
#define OBST_CLEAR_MS    500    // tiempo sin obstáculo para seguir (evita arrancones)

// Configura los LEDs IR, el sensor del frente, el buzzer y los LEDs RGB.
// Llamarla después de mcp2515_init(): repone la configuración de SPI0 del MCP2515.
void obstacle_init(void);

// Mide el frente (~7 ms) y devuelve el nivel 0..OBST_LEVELS.
uint8_t obstacle_front_level(void);

// Aplica la histéresis (OBST_CONFIRM y OBST_CLEAR_MS) a la lectura nueva y devuelve
// si hay obstáculo.
bool obstacle_detect(uint8_t level, uint32_t now_ms);

// Intermitentes y pitido mientras on es true; llamarla seguido (cada ~1 ms).
void obstacle_alert(bool on, uint32_t now_ms);

#endif
