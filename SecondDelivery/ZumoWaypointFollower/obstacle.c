#include "obstacle.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include <rgb_leds.h>

// Pines del Zumo 2040 (los mismos que usa Pololu en proximity_sensors.py y buzzer.py).
#define IR_LEFT_PIN      17     // LEDs IR del lado izquierdo (PWM 0B)
#define IR_RIGHT_PIN     16     // LEDs IR del lado derecho (PWM 0A)
#define FRONT_SENSOR_PIN 27     // TSSP770 del frente: en bajo cuando ve el reflejo
#define BUZZER_PIN       7      // PWM 3B
#define RGB_DATA_PIN     3      // datos de los LEDs RGB = MOSI del MCP2515

// El TSSP770 responde a pulsos de 56 kHz; los tiempos son los de Pololu (14 y 18
// periodos) para no leer antes de que responda ni después de que se apague.
#define IR_FREQ_HZ       56000u
#define PULSE_ON_US      (14u * 1000000u / IR_FREQ_HZ)
#define PULSE_OFF_US     (18u * 1000000u / IR_FREQ_HZ)
// Brillo de cada nivel: ns encendido por periodo de 17.9 µs (valores de Pololu).
static const uint16_t BRIGHTNESS_NS[OBST_LEVELS] = { 313, 1000, 2063, 3500, 5375, 7563 };

// Intermitentes: LEDs D (frente derecha) y F (frente izquierda), ámbar.
#define LED_COUNT        6
#define LED_FRONT_RIGHT  3
#define LED_FRONT_LEFT   5
#define LED_BRIGHTNESS   31
#define BLINK_MS         350    // medio periodo (≈1.4 parpadeos/s, como un carro)
#define BUZZER_HZ        2000u
#define BUZZER_DIV       4      // con 125 MHz el tope del PWM queda en 15 624 (< 65 535)
#define BEEP_MS          120    // pitido al prender los intermitentes

static uint32_t ir_level[OBST_LEVELS];   // brillos en cuentas del PWM
static uint16_t buzzer_on_level;

static void pwm_start(uint pin, float div, uint32_t freq_hz) {
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, div);
    pwm_config_set_wrap(&cfg, (uint16_t)(clock_get_hz(clk_sys) / div / freq_hz - 1));
    pwm_init(pwm_gpio_to_slice_num(pin), &cfg, true);
    pwm_set_gpio_level(pin, 0);
    gpio_set_function(pin, GPIO_FUNC_PWM);
}

// rgb_leds_write deja SPI0 a 20 MHz y suelta GP3: se repone lo que usa el MCP2515.
static void write_front_leds(bool on) {
    rgb_color c[LED_COUNT] = { 0 };
    if (on) c[LED_FRONT_LEFT] = c[LED_FRONT_RIGHT] = (rgb_color){ 255, 100, 0 };
    uint32_t cpsr = spi0_hw->cpsr, cr0 = spi0_hw->cr0;
    rgb_leds_write(c, LED_COUNT, LED_BRIGHTNESS);
    spi0_hw->cpsr = cpsr;
    spi0_hw->cr0 = cr0;
    gpio_set_function(RGB_DATA_PIN, GPIO_FUNC_SPI);
}

void obstacle_init(void) {
    uint32_t cpsr = spi0_hw->cpsr, cr0 = spi0_hw->cr0;
    rgb_leds_init();                 // también deja GP6 (reloj de los LEDs) en bajo
    spi0_hw->cpsr = cpsr;
    spi0_hw->cr0 = cr0;
    gpio_set_function(RGB_DATA_PIN, GPIO_FUNC_SPI);

    pwm_start(IR_LEFT_PIN, 1.0f, IR_FREQ_HZ);    // 16 y 17 comparten el PWM 0
    pwm_start(IR_RIGHT_PIN, 1.0f, IR_FREQ_HZ);
    for (int i = 0; i < OBST_LEVELS; i++)
        ir_level[i] = (uint32_t)((uint64_t)BRIGHTNESS_NS[i] * clock_get_hz(clk_sys) / 1000000000u);
    gpio_init(FRONT_SENSOR_PIN);
    gpio_pull_up(FRONT_SENSOR_PIN);

    pwm_start(BUZZER_PIN, BUZZER_DIV, BUZZER_HZ);
    buzzer_on_level = (uint16_t)(clock_get_hz(clk_sys) / BUZZER_DIV / BUZZER_HZ / 2);   // 50 %
}

static bool pulse_seen(uint pin, uint32_t level) {
    pwm_set_gpio_level(pin, (uint16_t)level);
    sleep_us(PULSE_ON_US);
    bool seen = !gpio_get(FRONT_SENSOR_PIN);
    pwm_set_gpio_level(pin, 0);
    sleep_us(PULSE_OFF_US);
    return seen;
}

uint8_t obstacle_front_level(void) {
    uint8_t left = 0, right = 0;
    for (int i = 0; i < OBST_LEVELS; i++) {
        left += pulse_seen(IR_LEFT_PIN, ir_level[i]);
        right += pulse_seen(IR_RIGHT_PIN, ir_level[i]);
    }
    return left > right ? left : right;
}

bool obstacle_detect(uint8_t level, uint32_t now_ms) {
    static int above;
    static bool blocked;
    static uint32_t last_seen_ms;
    if (level >= OBST_STOP_LEVEL) {
        last_seen_ms = now_ms;
        if (++above >= OBST_CONFIRM) blocked = true;
    } else {
        above = 0;
        if (blocked && now_ms - last_seen_ms >= OBST_CLEAR_MS) blocked = false;
    }
    return blocked;
}

void obstacle_alert(bool on, uint32_t now_ms) {
    static bool lit;
    uint32_t phase = now_ms % (2u * BLINK_MS);
    pwm_set_gpio_level(BUZZER_PIN, on && phase < BEEP_MS ? buzzer_on_level : 0);
    bool want = on && phase < BLINK_MS;
    if (want != lit) {
        lit = want;
        write_front_leds(lit);
    }
}
