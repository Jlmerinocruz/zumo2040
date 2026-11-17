// Zumo 2040 — Second Delivery, parte A: avanzar 50 cm en y usando la cámara, ya con la PCB.
//
// Es el programa de la parte 2 del reporte "Experiment Wireless Interfacing"
// (allí dio 503 mm medidos contra 49.6 cm con cinta), con estos cambios:
//   - Usa el driver MCP2515 corregido y lp_decode() de ZumoWaypointFollower.
//   - Si el MCP2515 no responde (switch del Zumo en OFF: la PCB no tiene energía),
//     reintenta cada segundo en vez de quedarse sin CAN.
//   - Imprime el estado del bus si se pierde la pose.
//
// Uso:
//   1. Switch del Zumo en ON (alimenta la Pico y los dos módulos CAN por VSW).
//   2. Colocar el Zumo en el waypoint inicial mirando hacia +y y presionar A.
//   3. Avanza 500 mm en y y se detiene. B = paro de emergencia.
//   El botón C no se usa: comparte GP0 con el MISO del MCP2515.
#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include <pololu_zumo_2040_robot.h>
#include "mcp2515.h"
#include "lp_can.h"

#define TARGET_DY_MM   500           // avance en y; ajustar si la cinta marca otra distancia
#define SLOW_ZONE_MM   80            // últimos mm a velocidad baja para no pasarse
#define CRUISE_SPEED   1200          // escala de la librería: 0 a 6000
#define APPROACH_SPEED 900           // con 600 el Zumo (con power bank) se atoraba
#define LP_TIMEOUT_MS  500           // sin posición por más tiempo -> detenerse

// GP3 es MOSI del MCP2515 y también datos de los LEDs RGB. Con el reloj
// de los LEDs (GP6) en bajo, el tráfico SPI no les cambia el color.
#define RGB_CLOCK_PIN 6

typedef enum { WAIT_BUTTON, DRIVING } state_t;

static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

static bool can_start(void) {
    mcp2515_reset();
    return mcp2515_set_bitrate_500kbps_8mhz() && mcp2515_set_mode(MCP2515_MODE_NORMAL);
}

static void print_can_errors(void) {
    printf("CAN: EFLG=0x%02X TEC=%u REC=%u\n", mcp2515_read_register(0x2D),
           mcp2515_read_register(0x1C), mcp2515_read_register(0x1D));
}

int main(void) {
    stdio_init_all();
    motors_init();
    gpio_init(RGB_CLOCK_PIN);
    gpio_set_dir(RGB_CLOCK_PIN, GPIO_OUT);
    gpio_put(RGB_CLOCK_PIN, 0);
    sleep_ms(2000);

    printf("\n--- Zumo parte A: avanzar %d mm en y ---\n", TARGET_DY_MM);
    mcp2515_init();
    bool can_ok = can_start();

    state_t state = WAIT_BUTTON;
    bool have_pose = false, paused = false, a_was = false;
    pose_t p = {0};
    int start_mm = 0;
    uint32_t last_rx = 0, last_print = 0;

    while (true) {
        if (!can_ok) {
            if (now_ms() - last_print > 1000) {
                last_print = now_ms();
                can_ok = can_start();
                printf(can_ok ? "MCP2515 listo\n"
                              : "MCP2515 no responde: switch del Zumo en ON? header SPI?\n");
            }
            continue;
        }

        // Si el MCP2515 se reinició (bajón de voltaje al encender el WiFi de la Pico, que
        // le da sus 3.3 V), queda en modo configuración y deja de recibir y de dar ACK.
        // Se revisa cada segundo y se reconfigura.
        static uint32_t last_check = 0;
        if (now_ms() - last_check > 1000) {
            last_check = now_ms();
            uint8_t st = mcp2515_read_register(MCP2515_CANSTAT);
            if ((st & 0xE0) != MCP2515_MODE_NORMAL) {
                printf("MCP2515 fuera de modo normal (CANSTAT=0x%02X): reconfigurando\n", st);
                can_ok = can_start();
                if (!can_ok) continue;
            }
        }

        bool fresh = false;
        if (mcp2515_message_available()) {
            can_frame_t rx;
            if (mcp2515_read_message(&rx) && lp_decode(&rx, &p)) {
                fresh = true;
                have_pose = true;
                last_rx = now_ms();
            }
        }
        int y_mm = (int)lroundf(p.y * 1000.0f);
        bool pose_valid = have_pose && now_ms() - last_rx < LP_TIMEOUT_MS;

        bool a = button_a_is_pressed();
        bool a_clicked = a && !a_was;
        a_was = a;

        switch (state) {
        case WAIT_BUTTON:
            if (now_ms() - last_print > 500) {
                last_print = now_ms();
                if (pose_valid) printf("pose: x=%d y=%d mm ang=%.1f bat=%u mV (A = arrancar)\n",
                                       (int)lroundf(p.x * 1000.0f), y_mm, p.theta * 57.2958f,
                                       battery_get_level_millivolts());
                else printf("esperando pose por CAN (ID 0x%03X)... bat=%u mV CANSTAT=0x%02X EFLG=0x%02X TEC=%u REC=%u INTF=0x%02X\n",
                            LP_CAN_ID, battery_get_level_millivolts(), mcp2515_read_register(MCP2515_CANSTAT), mcp2515_read_register(0x2D),
                            mcp2515_read_register(0x1C), mcp2515_read_register(0x1D),
                            mcp2515_read_register(MCP2515_CANINTF));
            }
            if (a_clicked) {
                if (!pose_valid) {
                    printf("Sin datos de posicion por CAN\n");
                } else {
                    start_mm = y_mm;
                    paused = false;
                    printf("INICIO y0=%d mm, meta=%d mm\n", start_mm, start_mm + TARGET_DY_MM);
                    state = DRIVING;
                }
            }
            break;

        case DRIVING:
            if (button_b_is_pressed()) {
                motors_set_speeds(0, 0);
                printf("ALTO: boton B\n");
                while (button_b_is_pressed()) sleep_ms(10);
                state = WAIT_BUTTON;
            } else if (!pose_valid) {
                if (!paused) {
                    motors_set_speeds(0, 0);
                    printf("Sin posicion valida: detenido\n");
                    print_can_errors();
                    paused = true;
                }
            } else if (fresh) {
                paused = false;
                int remaining = start_mm + TARGET_DY_MM - y_mm;
                if (remaining <= 0) {
                    motors_set_speeds(0, 0);
                    printf("META ALCANZADA y=%d mm, avance=%d mm, t=%lu ms\n",
                           y_mm, y_mm - start_mm, (unsigned long)now_ms());
                    state = WAIT_BUTTON;
                } else {
                    int speed = remaining > SLOW_ZONE_MM ? CRUISE_SPEED : APPROACH_SPEED;
                    motors_set_speeds(speed, speed);
                    printf("x=%d y=%d mm ang=%.1f | faltan %d mm | vel=%d\n",
                           (int)lroundf(p.x * 1000.0f), y_mm, p.theta * 57.2958f, remaining, speed);
                }
            }
            break;
        }
        sleep_ms(1);
    }
}
