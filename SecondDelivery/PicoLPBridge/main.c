// Pico 2 W — puente WiFi -> CAN para la Second Delivery.
//
// La cámara del techo manda por UDP (puerto 5000) un JSON con los marcadores.
// Este programa toma el marcador 4 (el del Zumo) y lo manda por CAN:
//
//   LP_POSE  ID 0x100  DLC 6  big-endian
//     bytes 0-1: x [mm] int16 | bytes 2-3: y [mm] int16 | bytes 4-5: ángulo [0.1°] int16
//
// También es el puente de la GUI de la PC para ir a un waypoint (../gui/):
//   UDP 5002 "GOTO n" (n = 1..14) o "STOP" -> CAN WP_GOTO  ID 0x110 DLC 1 (byte 0 = n, 0 = alto)
//     y contesta "OK GOTO n" / "OK STOP" si el Zumo dio ACK, o "ERR CAN" si no.
//   CAN WP_STATUS ID 0x111 DLC 2 (estado, waypoint) -> UDP "ESTADO e n" a la GUI
//     (la dirección que mandó el último comando).
//
// Reparto entre núcleos:
//   núcleo 0: WiFi y lwIP (cyw43_arch_poll), parseo del JSON, LED y watchdog. El chip
//             WiFi y lwIP solo se usan desde este núcleo (no son seguros entre núcleos).
//   núcleo 1: MCP2515 (envío por CAN con su espera de ACK de hasta 10 ms) e impresión
//             de cada pose enviada. Así el ACK y el printf no detienen la recepción WiFi.
// Sincronización: colas queue_t (spinlock de hardware). Del núcleo 0 al 1 van las poses
// y los WP_GOTO de la GUI; del 1 al 0, las respuestas para la GUI (texto) que solo el
// núcleo 0 puede mandar por UDP. Los contadores los escribe un solo núcleo y el otro solo los lee.
// El watchdog (3 s) solo se alimenta si los dos núcleos siguen vivos.
//
// LED de la Pico 2 W (montada en el Zumo normalmente no hay monitor serial):
//   parpadeo rápido   = conectando al WiFi
//   encendido fijo    = WiFi conectado, esperando poses de la cámara
//   parpadea con cada pose = la pose salió por CAN y el Zumo la confirmó (ACK)
//   apagado con WiFi  = llegan poses pero nadie da ACK (revisar bus CAN / Zumo)
//
// La red WiFi se configura en wifi_config.h (no se sube al repo).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "hardware/watchdog.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/etharp.h"
#include "mcp2515.h"
#include "wifi_config.h"   // WIFI_SSID y WIFI_PASSWORD (copiar de wifi_config.example.h)

#define TELEMETRY_PORT 5000
#define SUBSCRIBE_PORT 5001   // el servidor de la cámara acepta suscripciones en este puerto

#define MARKER_ID 4   // marcador del Zumo
#define LP_POSE_ID  0x100
#define LP_POSE_DLC 6
#define POSE_QUEUE_LEN 8      // la cámara manda ráfagas de 4-5 poses tras un hueco de WiFi
#define WP_CMD_PORT   5002
#define WP_GOTO_ID    0x110
#define WP_GOTO_DLC   1
#define WP_STATUS_ID  0x111
#define WP_STATUS_DLC 2
#define WAYPOINT_COUNT 14
#define GUI_QUEUE_LEN 4
#define MAX_RX_FRAMES 4       // por pasada, como en el Zumo

typedef struct {
    float x;      // m
    float y;      // m
    float angle;  // grados
} pose_t;

static queue_t pose_q;   // núcleo 0 -> núcleo 1
static queue_t goto_q;   // núcleo 0 -> núcleo 1, byte del WP_GOTO (0 = alto)
static queue_t reply_q;  // núcleo 1 -> núcleo 0, texto para la GUI

typedef struct { char text[24]; } gui_reply_t;

// Contadores acumulados. Cada uno lo escribe un solo núcleo.
static volatile uint32_t n_rx, n_nomarker;          // núcleo 0
static volatile uint32_t n_ok, n_fail, core1_alive; // núcleo 1
static volatile bool can_ok;                        // núcleo 1

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
    // es chica (strtof también usa bastante); 1 KB en la pila la desbordaba.
    static char json[1536];
    uint16_t len = pbuf_copy_partial(p, json, sizeof(json) - 1, 0);
    json[len] = '\0';
    pbuf_free(p);

    n_rx++;
    pose_t pose;
    if (!parse_marker(json, &pose)) { n_nomarker++; return; }
    // Cola llena (el núcleo 1 se atrasó): se tira la pose más vieja, nunca la nueva.
    if (!queue_try_add(&pose_q, &pose)) {
        pose_t old;
        queue_try_remove(&pose_q, &old);
        queue_try_add(&pose_q, &pose);
    }
}

// Dirección de la GUI (la del último comando); solo la usa el núcleo 0.
static ip_addr_t gui_addr;
static u16_t gui_port = 0;   // 0 = la GUI todavía no manda nada

static void on_cmd_packet(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                          const ip_addr_t *addr, u16_t port) {
    (void)arg; (void)pcb;
    if (p == NULL) return;
    char text[16];
    uint16_t len = pbuf_copy_partial(p, text, sizeof(text) - 1, 0);
    text[len] = '\0';
    pbuf_free(p);

    int n;
    if (strcmp(text, "STOP") == 0) n = 0;
    else if (sscanf(text, "GOTO %d", &n) != 1 || n < 1 || n > WAYPOINT_COUNT) {
        printf("GUI: comando desconocido \"%s\"\n", text);
        return;
    }
    ip_addr_copy(gui_addr, *addr);
    gui_port = port;
    uint8_t wp = (uint8_t)n;
    if (!queue_try_add(&goto_q, &wp)) printf("GUI: cola de comandos llena, se descarta\n");
}

// Manda a la GUI las respuestas que dejó el núcleo 1.
static void send_gui_replies(struct udp_pcb *pcb) {
    gui_reply_t r;
    while (queue_try_remove(&reply_q, &r)) {
        if (gui_port == 0) continue;
        size_t len = strlen(r.text);
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
        if (!p) continue;
        memcpy(p->payload, r.text, len);
        udp_sendto(pcb, p, &gui_addr, gui_port);
        pbuf_free(p);
    }
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

// ---------------------------------------------------------------- núcleo 1 ----

static void push_reply(const char *text) {
    gui_reply_t r;
    snprintf(r.text, sizeof r.text, "%s", text);
    printf("-> GUI: %s\n", r.text);
    queue_try_add(&reply_q, &r);
}

// WP_GOTO de la GUI hacia el Zumo; contesta si dio ACK.
static void send_goto(uint8_t wp) {
    can_frame_t frame = { .id = WP_GOTO_ID, .dlc = WP_GOTO_DLC, .data = { wp } };
    char text[24];
    if (!(can_ok && mcp2515_send_message(&frame))) snprintf(text, sizeof text, "ERR CAN");
    else if (wp == 0) snprintf(text, sizeof text, "OK STOP");
    else snprintf(text, sizeof text, "OK GOTO %u", wp);
    push_reply(text);
}

// WP_STATUS del Zumo hacia la GUI.
static void read_status(void) {
    can_frame_t rx;
    for (int n = 0; n < MAX_RX_FRAMES && mcp2515_read_message(&rx); n++) {
        if (rx.id != WP_STATUS_ID || rx.dlc < WP_STATUS_DLC) continue;
        char text[24];
        snprintf(text, sizeof text, "ESTADO %u %u", rx.data[0], rx.data[1]);
        push_reply(text);
    }
}

static void core1_main(void) {
    mcp2515_init(MCP2515_PINS_PICO2W);
    can_ok = mcp2515_start();
    printf("MCP2515: %s\n", can_ok ? "listo" : "no responde (se reintenta cada segundo)");
    uint32_t last_retry = to_ms_since_boot(get_absolute_time());
    while (true) {
        core1_alive++;
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (!can_ok && now - last_retry >= 1000) {
            last_retry = now;
            can_ok = mcp2515_start();
        }
        uint8_t wp;
        if (queue_try_remove(&goto_q, &wp)) send_goto(wp);
        if (can_ok && !gpio_get(MCP2515_PINS_PICO2W.irq)) read_status();
        pose_t p;
        if (!queue_try_remove(&pose_q, &p)) { sleep_ms(1); continue; }
        if (can_ok && send_pose(&p)) n_ok++;
        else n_fail++;
    }
}

// ---------------------------------------------------------------- núcleo 0 ----

static void led(bool on) { cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on); }

// Alimenta el watchdog solo si el núcleo 1 avanzó desde la última vez: si cualquiera
// de los dos núcleos se cuelga, la Pico se reinicia sola.
static void feed_watchdog(void) {
    static uint32_t seen;
    uint32_t alive = core1_alive;
    if (alive != seen) { seen = alive; watchdog_update(); }
}

// LED según los resultados que reporta el núcleo 1.
static void update_led(void) {
    static uint32_t seen_ok, seen_fail;
    uint32_t ok = n_ok, fail = n_fail;
    if (ok != seen_ok) { seen_ok = ok; led(ok & 1); }
    if (fail != seen_fail) { seen_fail = fail; led(false); }
}

// Resumen cada 2 s: sirve para medir cuántas poses por segundo manda la cámara.
static void print_status(const ip_addr_t *server) {
    static uint32_t p_rx, p_nomarker, p_ok, p_fail;
    uint32_t rx = n_rx, nomarker = n_nomarker, ok = n_ok, fail = n_fail;
    printf("# 2 s: UDP=%lu sin_marcador4=%lu CAN_ok=%lu CAN_fallo=%lu MCP2515=%s\n",
           (unsigned long)(rx - p_rx), (unsigned long)(nomarker - p_nomarker),
           (unsigned long)(ok - p_ok), (unsigned long)(fail - p_fail),
           can_ok ? "ok" : "NO RESPONDE");
    if (rx == p_rx) {
        // No llegó nada: ¿la Pico ve al servidor de la cámara?
        struct eth_addr *mac; const ip4_addr_t *ipr;
        bool arp = etharp_find_addr(netif_list, ip_2_ip4(server), &mac, &ipr) >= 0;
        printf("#   no llega UDP; ARP del servidor %s\n",
               arp ? "OK (la red funciona, revisar la camara)" : "SIN respuesta (no hay comunicacion)");
    }
    p_rx = rx; p_nomarker = nomarker; p_ok = ok; p_fail = fail;
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

    // CAN primero (núcleo 1): así se puede revisar el bus aunque el WiFi tarde.
    queue_init(&pose_q, sizeof(pose_t), POSE_QUEUE_LEN);
    queue_init(&goto_q, sizeof(uint8_t), GUI_QUEUE_LEN);
    queue_init(&reply_q, sizeof(gui_reply_t), GUI_QUEUE_LEN);
    multicore_launch_core1(core1_main);

    // Si algo se cuelga más de 3 s, la Pico se reinicia sola en vez de quedarse muerta.
    watchdog_enable(3000, true);

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
        feed_watchdog();
    }
    {
        uint8_t b[6] = {0};
        cyw43_wifi_get_bssid(&cyw43_state, b);
        int32_t rssi = 0;
        cyw43_wifi_get_rssi(&cyw43_state, &rssi);
        printf("Conectado, IP %s, AP %02X:%02X:%02X:%02X:%02X:%02X, RSSI %ld dBm\n",
               ip4addr_ntoa(netif_ip4_addr(netif_list)), b[0], b[1], b[2], b[3], b[4], b[5],
               (long)rssi);
    }
    led(true);

    // Sin esto el radio acumula los broadcast y llegan con segundos de retraso.
    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);
    struct udp_pcb *pcb = udp_new();
    udp_bind(pcb, IP_ADDR_ANY, TELEMETRY_PORT);
    udp_recv(pcb, on_udp_packet, NULL);
    struct udp_pcb *cmd_pcb = udp_new();
    udp_bind(cmd_pcb, IP_ADDR_ANY, WP_CMD_PORT);
    udp_recv(cmd_pcb, on_cmd_packet, NULL);
    printf("servidor %s, escuchando UDP %d (camara) y %d (GUI de waypoints)\n",
           ip4addr_ntoa(netif_ip4_gw(netif_list)), TELEMETRY_PORT, WP_CMD_PORT);

    // El servidor de la cámara manda broadcast al 5000, pero ese broadcast puede no
    // salir por el hotspot. También acepta suscripciones en UDP 5001: le mandamos un
    // paquete (desde nuestro puerto 5000) y registra nuestra IP para enviarnos directo.
    // Se repite cada 2 s por si el servidor reinicia o expira la suscripción.
    ip_addr_t server = *netif_ip4_gw(netif_list);
    absolute_time_t next_sub = get_absolute_time();
    absolute_time_t next_status = make_timeout_time_ms(2000);
    while (true) {
        feed_watchdog();
        cyw43_arch_poll();
        update_led();
        send_gui_replies(cmd_pcb);

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
        if (time_reached(next_status)) {
            next_status = make_timeout_time_ms(2000);
            print_status(&server);
        }
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(1));
    }
}
