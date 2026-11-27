// Driver del MCP2515 (CAN 2.0A, 500 kbps, cristal de 8 MHz), común al Zumo 2040 y a
// la Pico 2 W. Solo cambian los pines (tabla en el README y en CLAUDE.md).
//
// No es seguro llamarlo desde dos núcleos a la vez: en cada placa solo un núcleo
// usa el MCP2515 (el bus SPI0 tiene un solo dueño).
#ifndef MCP2515_H
#define MCP2515_H

#include <stdbool.h>
#include <stdint.h>
#include "hardware/spi.h"

typedef struct {
    uint miso, cs, sck, mosi;
    uint irq;   // INT del MCP2515: en bajo mientras haya un mensaje recibido sin leer
} mcp2515_pins_t;

#define MCP2515_PINS_ZUMO   ((mcp2515_pins_t){ .miso = 0,  .cs = 1,  .sck = 2,  .mosi = 3,  .irq = 28 })
#define MCP2515_PINS_PICO2W ((mcp2515_pins_t){ .miso = 16, .cs = 17, .sck = 18, .mosi = 19, .irq = 20 })

// Registros que se leen para diagnóstico.
#define MCP2515_TEC         0x1C   // contador de errores de transmisión
#define MCP2515_REC         0x1D   // contador de errores de recepción
#define MCP2515_EFLG        0x2D   // banderas de error
#define MCP2515_CANSTAT     0x0E
#define MCP2515_CNF2        0x29   // 0x90 tras mcp2515_start(); si se lee 0x00 el SPI no responde

#define MCP2515_MODE_NORMAL 0x00
#define MCP2515_MODE_CONFIG 0x80
#define MCP2515_MODE_MASK   0xE0

typedef struct {
    uint32_t id;      // 11 bits
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

void    mcp2515_init(mcp2515_pins_t pins);
uint8_t mcp2515_read_register(uint8_t address);
// Reset + 500 kbps + modo normal. Devuelve false si el MCP2515 no responde
// (p. ej. switch del Zumo en OFF: la PCB se alimenta de VSW).
bool    mcp2515_start(void);
// true si el MCP2515 sigue en modo normal y con la configuración de mcp2515_start().
// Detecta un bajón de voltaje (lo regresa a modo configuración con CNF en 0) y un
// módulo sin energía o MISO muerto (todo se lee 0x00, que solo con CANSTAT parecería
// "modo normal").
bool    mcp2515_in_normal_mode(void);
// Envía por TXB0 y espera el ACK hasta 10 ms. false = nadie confirmó (se aborta).
bool    mcp2515_send_message(const can_frame_t *frame);
// Lee un mensaje de RXB0 o RXB1; false si no hay ninguno.
bool    mcp2515_read_message(can_frame_t *frame);

#endif
