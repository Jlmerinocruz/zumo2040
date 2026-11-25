#include "mcp2515.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// Instrucciones SPI (datasheet MCP2515, tabla 12-1).
#define CMD_RESET          0xC0
#define CMD_READ           0x03
#define CMD_WRITE          0x02
#define CMD_BIT_MODIFY     0x05
#define CMD_READ_STATUS    0xA0
#define CMD_LOAD_TXB0      0x40   // escribe TXB0 desde SIDH
#define CMD_RTS_TXB0       0x81
#define CMD_READ_RXB0      0x90   // lee RXB0 desde SIDH y limpia RX0IF al subir CS
#define CMD_READ_RXB1      0x94   // igual para RXB1

#define REG_CANCTRL        0x0F
#define REG_CNF3           0x28
#define REG_CANINTE        0x2B
#define REG_TXB0CTRL       0x30
#define REG_RXB0CTRL       0x60
#define REG_RXB1CTRL       0x70

#define STATUS_RX0IF       0x01   // bits de READ STATUS
#define STATUS_RX1IF       0x02
#define TXB0CTRL_TXREQ     0x08

#define SPI_BAUD           1000000u
#define CNF2_500K          0x90   // CNF1/2/3 = 0x00/0x90/0x02: 500 kbps con cristal de 8 MHz

static uint pin_cs;

static inline void cs_select(void)   { gpio_put(pin_cs, 0); }
static inline void cs_deselect(void) { gpio_put(pin_cs, 1); }

static void transfer(const uint8_t *tx, uint8_t *rx, size_t n) {
    cs_select();
    if (rx) spi_write_read_blocking(spi0, tx, rx, n);
    else    spi_write_blocking(spi0, tx, n);
    cs_deselect();
}

void mcp2515_init(mcp2515_pins_t pins) {
    spi_init(spi0, SPI_BAUD);
    gpio_set_function(pins.miso, GPIO_FUNC_SPI);
    gpio_set_function(pins.sck, GPIO_FUNC_SPI);
    gpio_set_function(pins.mosi, GPIO_FUNC_SPI);

    pin_cs = pins.cs;
    gpio_init(pin_cs);
    gpio_set_dir(pin_cs, GPIO_OUT);
    cs_deselect();

    gpio_init(pins.irq);
    gpio_set_dir(pins.irq, GPIO_IN);
    gpio_pull_up(pins.irq);
}

uint8_t mcp2515_read_register(uint8_t address) {
    uint8_t tx[3] = { CMD_READ, address, 0x00 }, rx[3];
    transfer(tx, rx, sizeof tx);
    return rx[2];
}

static void modify_register(uint8_t address, uint8_t mask, uint8_t value) {
    uint8_t tx[4] = { CMD_BIT_MODIFY, address, mask, value };
    transfer(tx, NULL, sizeof tx);
}

static uint8_t read_status(void) {
    uint8_t tx[2] = { CMD_READ_STATUS, 0x00 }, rx[2];
    transfer(tx, rx, sizeof tx);
    return rx[1];
}

// Espera máximo 100 ms al cambio de modo (si el MCP2515 no tiene energía nunca cambia).
static bool set_mode(uint8_t mode) {
    modify_register(REG_CANCTRL, MCP2515_MODE_MASK, mode);
    for (int i = 0; i < 100; i++) {
        if ((mcp2515_read_register(MCP2515_CANSTAT) & MCP2515_MODE_MASK) == mode) return true;
        sleep_ms(1);
    }
    return false;
}

bool mcp2515_start(void) {
    uint8_t cmd = CMD_RESET;
    transfer(&cmd, NULL, 1);
    sleep_ms(10);
    if (!set_mode(MCP2515_MODE_CONFIG)) return false;   // CNFx solo se escriben aquí
    // CNF3, CNF2, CNF1 son consecutivos (0x28-0x2A): 500 kbps con cristal de 8 MHz.
    const uint8_t cnf[] = { CMD_WRITE, REG_CNF3, 0x02, CNF2_500K, 0x00 };
    transfer(cnf, NULL, sizeof cnf);
    // INT con mensaje en RXB0 o RXB1. RXB0 acepta todo y, si está lleno, el siguiente
    // mensaje pasa a RXB1 (rollover) en vez de perderse: la cámara llega en ráfagas.
    const uint8_t inte[] = { CMD_WRITE, REG_CANINTE, STATUS_RX0IF | STATUS_RX1IF };
    const uint8_t rxb0[] = { CMD_WRITE, REG_RXB0CTRL, 0x64 };
    const uint8_t rxb1[] = { CMD_WRITE, REG_RXB1CTRL, 0x60 };
    transfer(inte, NULL, sizeof inte);
    transfer(rxb0, NULL, sizeof rxb0);
    transfer(rxb1, NULL, sizeof rxb1);
    return set_mode(MCP2515_MODE_NORMAL);
}

bool mcp2515_in_normal_mode(void) {
    return (mcp2515_read_register(MCP2515_CANSTAT) & MCP2515_MODE_MASK) == MCP2515_MODE_NORMAL &&
           mcp2515_read_register(MCP2515_CNF2) == CNF2_500K;
}

bool mcp2515_send_message(const can_frame_t *frame) {
    if (frame->dlc > 8) return false;
    // Una sola transacción: SIDH, SIDL, EID8, EID0, DLC y datos.
    uint8_t tx[6 + 8] = { CMD_LOAD_TXB0, (uint8_t)(frame->id >> 3),
                          (uint8_t)((frame->id & 0x07) << 5), 0, 0, frame->dlc };
    for (uint8_t i = 0; i < frame->dlc; i++) tx[6 + i] = frame->data[i];
    transfer(tx, NULL, 6u + frame->dlc);

    uint8_t rts = CMD_RTS_TXB0;
    transfer(&rts, NULL, 1);
    for (int i = 0; i < 10; i++) {
        if (!(mcp2515_read_register(REG_TXB0CTRL) & TXB0CTRL_TXREQ)) return true;
        sleep_ms(1);
    }
    modify_register(REG_TXB0CTRL, TXB0CTRL_TXREQ, 0);   // sin ACK: abortar
    return false;
}

bool mcp2515_read_message(can_frame_t *frame) {
    uint8_t st = read_status();
    uint8_t cmd;
    if (st & STATUS_RX0IF)      cmd = CMD_READ_RXB0;
    else if (st & STATUS_RX1IF) cmd = CMD_READ_RXB1;
    else return false;
    // Una sola transacción: SIDH, SIDL, EID8, EID0, DLC y 8 datos.
    uint8_t tx[1 + 13] = { cmd }, rx[1 + 13];
    transfer(tx, rx, sizeof tx);
    frame->id = ((uint32_t)rx[1] << 3) | (rx[2] >> 5);
    frame->dlc = rx[5] & 0x0F;
    if (frame->dlc > 8) frame->dlc = 8;
    for (uint8_t i = 0; i < frame->dlc; i++) frame->data[i] = rx[6 + i];
    return true;
}
