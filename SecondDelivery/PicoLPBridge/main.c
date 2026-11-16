// Pico 2 W — puente WiFi -> CAN para la Second Delivery.
//
// La cámara del techo manda por UDP (puerto 5000) un JSON con los marcadores.
// Este programa toma el marcador 4 (el del Zumo) y lo manda por CAN:
//
//   LP_POSE  ID 0x100  DLC 6  big-endian
//     bytes 0-1: x [mm] int16 | bytes 2-3: y [mm] int16 | bytes 4-5: ángulo [0.1°] int16
//
// Basado en el código del reporte "Experiment Wireless Interfacing" (ya probado),
// con estos cambios:
//   - Driver MCP2515 corregido (set_mode con timeout y ACK con timeout real).
//   - Si el MCP2515 no responde al arrancar, se reintenta cada segundo.
//   - El LED de la Pico indica el estado (ver abajo), porque montada en el Zumo
//     normalmente no hay monitor serial.
//   - La red WiFi se configura en wifi_config.h (no se sube al repo).
//
// LED de la Pico 2 W:
//   parpadeo rápido   = conectando al WiFi
//   encendido fijo    = WiFi conectado, esperando poses de la cámara
//   parpadea con cada pose = la pose salió por CAN y el Zumo la confirmó (ACK)
//   apagado con WiFi  = llegan poses pero nadie da ACK (revisar bus CAN / Zumo)
//
// Pines (PCB): GP16 MISO, GP17 CS, GP18 SCK (puente al pin 6 del módulo), GP19 MOSI, GP20 INT.
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "mcp2515.h"
#include "wifi_config.h"   // WIFI_SSID y WIFI_PASSWORD (copiar de wifi_config.example.h)

#define TELEMETRY_PORT 5000

// La coma final evita confundir el 4 con el 41.
static const char *MARKER_KEY = "\"id\": 4,";

#define LP_POSE_ID  0x100
#define LP_POSE_DLC 6

typedef struct {
    float x;      // m
    float y;      // m
    float angle;  // grados
} pose_t;

static pose_t pose;
static volatile bool pose_ready = false;
static uint32_t n_rx = 0, n_ok = 0, n_fail = 0, n_nomarker = 0;

static bool parse_marker(const char *json, pose_t *out) {
    const char *m = strstr(json, MARKER_KEY);
    return m && sscanf(m, "\"id\": %*d, \"x\": %f, \"y\": %f, \"angle\": %f",
                       &out->x, &out->y, &out->angle) == 3;
}

static void on_udp_packet(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                          const ip_addr_t *addr, u16_t port) {
    (void)arg; (void)pcb; (void)addr; (void)port;
    if (p == NULL) return;

    char json[1024];
    uint16_t len = pbuf_copy_partial(p, json, sizeof(json) - 1, 0);
    json[len] = '\0';
    pbuf_free(p);

    n_rx++;
    if (parse_marker(json, &pose)) pose_ready = true;
    else n_nomarker++;
}

static int16_t to_fixed(float value, float scale) {
    float scaled = value * scale;
    return (int16_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static void put_i16(uint8_t *dst, int16_t value) {
    dst[0] = (uint8_t)((uint16_t)value >> 8);
    dst[1] = (uint8_t)value;
}

static bool send_pose(const pose_t *p) {
    can_frame_t frame = { .id = LP_POSE_ID, .dlc = LP_POSE_DLC };
    put_i16(&frame.data[0], to_fixed(p->x, 1000.0f));
    put_i16(&frame.data[2], to_fixed(p->y, 1000.0f));
    put_i16(&frame.data[4], to_fixed(p->angle, 10.0f));

    bool sent = mcp2515_send_message(&frame);

    printf("LP x=%.3f y=%.3f ang=%.1f | TX ID=0x%03X DLC=%d Data=",
           p->x, p->y, p->angle, (unsigned)frame.id, frame.dlc);
    for (int i = 0; i < frame.dlc; i++) printf("%02X ", frame.data[i]);
    printf("%s\n", sent ? "OK" : "FALLO (sin ACK)");
    return sent;
}

static void led(bool on) { cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on); }

static bool can_start(void) {
    mcp2515_reset();
    return mcp2515_set_bitrate_500kbps_8mhz() && mcp2515_set_mode(MCP2515_MODE_NORMAL);
}

int main(void) {
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();

    // CAN primero: así se puede revisar el bus aunque el WiFi tarde.
    mcp2515_init();
    bool can_ok = can_start();
    printf("MCP2515: %s\n", can_ok ? "listo" : "no responde (se reintenta)");

    printf("Conectando a \"%s\"...\n", WIFI_SSID);
    cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);
    bool blink = false;
    while (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) != CYW43_LINK_UP) {
        int st = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        if (st == CYW43_LINK_FAIL || st == CYW43_LINK_NONET || st == CYW43_LINK_BADAUTH) {
            printf("Wi-Fi falló (estado %d), reintentando...\n", st);
            cyw43_arch_wifi_connect_async(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);
        }
        led(blink = !blink);
        for (int i = 0; i < 20; i++) { cyw43_arch_poll(); sleep_ms(5); }
    }
    printf("Conectado, IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
    led(true);

    // Sin esto el radio acumula los broadcast y llegan con segundos de retraso.
    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);
    struct udp_pcb *pcb = udp_new();
    udp_bind(pcb, IP_ADDR_ANY, TELEMETRY_PORT);
    udp_recv(pcb, on_udp_packet, NULL);

    absolute_time_t next_status = make_timeout_time_ms(2000);
    while (true) {
        cyw43_arch_poll();

        if (!can_ok && time_reached(next_status)) can_ok = can_start();

        if (pose_ready) {
            pose_ready = false;
            if (can_ok && send_pose(&pose)) { n_ok++; led(n_ok & 1); }
            else { n_fail++; led(false); }
        }

        // Resumen cada 2 s: sirve para medir cuántas poses por segundo manda la cámara.
        if (time_reached(next_status)) {
            next_status = make_timeout_time_ms(2000);
            printf("# 2 s: UDP=%lu sin_marcador4=%lu CAN_ok=%lu CAN_fallo=%lu MCP2515=%s\n",
                   (unsigned long)n_rx, (unsigned long)n_nomarker, (unsigned long)n_ok,
                   (unsigned long)n_fail, can_ok ? "ok" : "NO RESPONDE");
            n_rx = n_nomarker = n_ok = n_fail = 0;
        }
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(1));
    }
}
