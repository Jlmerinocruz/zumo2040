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
#include "hardware/watchdog.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/raw.h"
#include "lwip/etharp.h"
#include "lwip/prot/ip4.h"
#include "lwip/prot/udp.h"
#include "mcp2515.h"
#include "wifi_config.h"   // WIFI_SSID y WIFI_PASSWORD (copiar de wifi_config.example.h)

#define TELEMETRY_PORT 5000
#define SUBSCRIBE_PORT 5001   // el servidor de la cámara acepta suscripciones en este puerto

#define MARKER_ID 4   // marcador del Zumo
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

static bool get_num(const char *obj, const char *key, float *out) {
    const char *k = strstr(obj, key);
    if (!k) return false;
    char *end;
    float v = strtof(k + strlen(key), &end);   // strtof ignora espacios iniciales
    if (end == k + strlen(key)) return false;
    *out = v;
    return true;
}

// Busca el marcador MARKER_ID en {"markers":[{"id":..,"x":..,"y":..,"angle":..},...]}.
// Funciona con o sin espacios después de los dos puntos (formato nuevo del servidor y
// formato de la guía). Misma idea que el gateway de otro equipo que ya funcionó.
static bool parse_marker(char *json, pose_t *out) {
    char *p = json;
    while ((p = strstr(p, "\"id\":")) != NULL) {
        int id = atoi(p + 5);                  // atoi también ignora espacios
        char *end = strchr(p, '}');
        if (!end) return false;
        if (id == MARKER_ID) {
            *end = '\0';                       // limita la búsqueda a este marcador
            bool ok = get_num(p, "\"x\":", &out->x) && get_num(p, "\"y\":", &out->y) &&
                      get_num(p, "\"angle\":", &out->angle);
            *end = '}';
            return ok;
        }
        p = end;
    }
    return false;
}

static void on_udp_packet(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                          const ip_addr_t *addr, u16_t port) {
    (void)arg; (void)pcb; (void)addr; (void)port;
    if (p == NULL) return;

    // static: el callback corre dentro de cyw43_arch_poll() y la pila del núcleo 0
    // es chica (sscanf con float también usa bastante); 1 KB en la pila la desbordaba.
    static char json[1536];
    uint16_t len = pbuf_copy_partial(p, json, sizeof(json) - 1, 0);
    json[len] = '\0';
    pbuf_free(p);

    n_rx++;
    if (parse_marker(json, &pose)) pose_ready = true;
    else n_nomarker++;
}

// Diagnóstico: ve TODOS los paquetes UDP que llegan (cualquier puerto) sin consumirlos.
static uint32_t n_any = 0;
static char last_any[80] = "";
static u8_t on_raw_udp(void *arg, struct raw_pcb *pcb, struct pbuf *p, const ip_addr_t *addr) {
    (void)arg; (void)pcb;
    if (p->len >= 28) {
        const struct ip_hdr *ip = (const struct ip_hdr *)p->payload;
        const struct udp_hdr *u = (const struct udp_hdr *)((const u8_t *)p->payload + IPH_HL_BYTES(ip));
        n_any++;
        char dst[16];
        ip4_addr_t d;
        memcpy(&d, (const u8_t *)p->payload + 16, sizeof d);   // destino del encabezado IP
        ip4addr_ntoa_r(&d, dst, sizeof dst);
        snprintf(last_any, sizeof last_any, "%s:%u -> %s:%u (%u bytes)",
                 ipaddr_ntoa(addr), lwip_ntohs(u->src), dst, lwip_ntohs(u->dest), p->tot_len);
    }
    return 0;   // 0 = no se consume; lwIP sigue entregándolo normalmente
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

// Diagnóstico: lista todas las redes con el SSID del laboratorio (puede haber más de
// un hotspot "embedded" con la misma IP 192.168.12.1, uno por cada pista).
static int scan_cb(void *env, const cyw43_ev_scan_result_t *r) {
    (void)env;
    if (r && r->ssid_len == strlen(WIFI_SSID) && memcmp(r->ssid, WIFI_SSID, r->ssid_len) == 0)
        printf("  AP \"%s\" BSSID %02X:%02X:%02X:%02X:%02X:%02X  RSSI %d dBm  canal %u\n",
               WIFI_SSID, r->bssid[0], r->bssid[1], r->bssid[2], r->bssid[3], r->bssid[4],
               r->bssid[5], r->rssi, r->channel);
    return 0;
}

static void led(bool on) { cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on); }

static bool can_start(void) {
    mcp2515_reset();
    return mcp2515_set_bitrate_500kbps_8mhz() && mcp2515_set_mode(MCP2515_MODE_NORMAL);
}

int main(void) {
    stdio_init_all();
    bool wd_reboot = watchdog_caused_reboot();
    sleep_ms(1500);   // tiempo para abrir el monitor serial
    if (wd_reboot) printf("AVISO: reinicio por watchdog (el programa se habia colgado)\n");

    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();

    // CAN primero: así se puede revisar el bus aunque el WiFi tarde.
    mcp2515_init();
    bool can_ok = can_start();
    printf("MCP2515: %s\n", can_ok ? "listo" : "no responde (se reintenta)");

    // Si algo se cuelga más de 3 s, la Pico se reinicia sola en vez de quedarse muerta.
    watchdog_enable(3000, true);

    printf("Buscando redes \"%s\"...\n", WIFI_SSID);
    cyw43_wifi_scan_options_t opts = {0};
    if (cyw43_wifi_scan(&cyw43_state, &opts, NULL, scan_cb) == 0) {
        absolute_time_t t_end = make_timeout_time_ms(6000);
        while (cyw43_wifi_scan_active(&cyw43_state) && !time_reached(t_end)) {
            cyw43_arch_poll(); sleep_ms(10); watchdog_update();
        }
    }
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
        watchdog_update();
    }
    printf("Conectado, IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
    {
        uint8_t b[6] = {0};
        cyw43_wifi_get_bssid(&cyw43_state, b);
        int32_t rssi = 0;
        cyw43_wifi_get_rssi(&cyw43_state, &rssi);
        printf("Conectado al AP %02X:%02X:%02X:%02X:%02X:%02X, RSSI %ld dBm\n",
               b[0], b[1], b[2], b[3], b[4], b[5], (long)rssi);
    }
    led(true);

    // Sin esto el radio acumula los broadcast y llegan con segundos de retraso.
    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);
    struct udp_pcb *pcb = udp_new();
    udp_bind(pcb, IP_ADDR_ANY, TELEMETRY_PORT);
    udp_recv(pcb, on_udp_packet, NULL);
    struct raw_pcb *rpcb = raw_new(IP_PROTO_UDP);
    raw_recv(rpcb, on_raw_udp, NULL);
    printf("IP %s, ", ip4addr_ntoa(netif_ip4_addr(netif_list)));
    printf("servidor %s, escuchando UDP %d\n", ip4addr_ntoa(netif_ip4_gw(netif_list)), TELEMETRY_PORT);

    // El servidor de la cámara manda broadcast al 5000, pero ese broadcast puede no
    // salir por el hotspot. También acepta suscripciones en UDP 5001: le mandamos un
    // paquete (desde nuestro puerto 5000) y registra nuestra IP para enviarnos directo. Se repite cada 2 s
    // por si el servidor reinicia o expira la suscripción.
    ip_addr_t server = *netif_ip4_gw(netif_list);
    absolute_time_t next_sub = get_absolute_time();

    absolute_time_t next_status = make_timeout_time_ms(2000);
    while (true) {
        watchdog_update();
        cyw43_arch_poll();

        if (!can_ok && time_reached(next_status)) can_ok = can_start();

        if (time_reached(next_sub)) {
            next_sub = make_timeout_time_ms(2000);
            static const char msg[] = "subscribe";   // inofensivo si el servidor no lo usa
            struct pbuf *sp = pbuf_alloc(PBUF_TRANSPORT, sizeof msg - 1, PBUF_RAM);
            if (sp) {
                memcpy(sp->payload, msg, sizeof msg - 1);
                udp_sendto(pcb, sp, &server, SUBSCRIBE_PORT);
                pbuf_free(sp);
            }
        }

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
            {
                struct eth_addr *mac; const ip4_addr_t *ipr;
                ssize_t idx = etharp_find_addr(netif_list, ip_2_ip4(&server), &mac, &ipr);
                if (idx >= 0) printf("#   ARP del servidor OK: %02X:%02X:%02X:%02X:%02X:%02X (la Pico si recibe de el)\n",
                                     mac->addr[0], mac->addr[1], mac->addr[2], mac->addr[3], mac->addr[4], mac->addr[5]);
                else printf("#   ARP del servidor SIN respuesta (no hay comunicacion con 192.168.12.1)\n");
            }
            if (n_any) printf("#   UDP de cualquier puerto: %lu, ultimo: %s\n", (unsigned long)n_any, last_any);
            else printf("#   no llega ningun paquete UDP (ningun puerto)\n");
            n_rx = n_nomarker = n_ok = n_fail = n_any = 0;
        }
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(1));
    }
}
