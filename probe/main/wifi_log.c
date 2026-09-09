/* Ship the log over UDP, because the Zigbee stack takes the serial console
 * down with it a few seconds after boot.
 *
 * Log lines are captured with an esp_log hook, queued, and sent by a task.
 * The hook must not block or log, so it drops lines rather than waiting.
 *
 * Receive them with:  nc -ul 5555
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "wifi_log.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <lwip/sockets.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define LINE_MAX   200
#define QUEUE_LEN  32

static const char *TAG = "wifilog";

typedef struct {
    char text[LINE_MAX];
} log_line_t;

static QueueHandle_t s_queue;
static EventGroupHandle_t s_events;
#define GOT_IP_BIT BIT0
static vprintf_like_t s_previous;
static volatile bool s_connected;

/* Runs inside every ESP_LOG call, including from ISR-adjacent contexts and
 * from the Wi-Fi stack itself. Never block, never log, never allocate. */
static int log_hook(const char *format, va_list args)
{
    if (s_queue) {
        log_line_t line;
        va_list copy;
        va_copy(copy, args);
        const int n = vsnprintf(line.text, sizeof(line.text), format, copy);
        va_end(copy);
        if (n > 0) {
            /* Drop rather than wait: blocking here would deadlock the caller. */
            xQueueSend(s_queue, &line, 0);
        }
    }
    return s_previous ? s_previous(format, args) : 0;
}

static void sender_task(void *arg)
{
    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        vTaskDelete(NULL);
        return;
    }
    int broadcast = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_PROBE_LOG_UDP_PORT),
        .sin_addr.s_addr = inet_addr(CONFIG_PROBE_LOG_UDP_TARGET),
    };

    log_line_t line;
    while (true) {
        if (xQueueReceive(s_queue, &line, portMAX_DELAY) == pdTRUE && s_connected) {
            sendto(sock, line.text, strlen(line.text), 0,
                   (struct sockaddr *)&dest, sizeof(dest));
        }
    }
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        ESP_LOGW(TAG, "sta start -> connect: %s", esp_err_to_name(esp_wifi_connect()));
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *d = (const wifi_event_sta_disconnected_t *)data;
        s_connected = false;
        ESP_LOGW(TAG, "disconnected, reason %d", d ? d->reason : -1);
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        const wifi_event_sta_connected_t *c = (const wifi_event_sta_connected_t *)data;
        ESP_LOGW(TAG, "associated on channel %d", c ? c->channel : -1);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_connected = true;
        if (s_events) xEventGroupSetBits(s_events, GOT_IP_BIT);
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)data;
        ESP_LOGW(TAG, "wifi up: " IPSTR ", logging to %s:%d", IP2STR(&event->ip_info.ip),
                 CONFIG_PROBE_LOG_UDP_TARGET, CONFIG_PROBE_LOG_UDP_PORT);
    }
}

void wifi_log_start(void)
{
    if (strlen(CONFIG_PROBE_WIFI_SSID) == 0) {
        ESP_LOGW(TAG, "no SSID configured, staying on serial only");
        return;
    }

    s_queue = xQueueCreate(QUEUE_LEN, sizeof(log_line_t));
    if (!s_queue) return;
    s_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    const esp_err_t loop = esp_event_loop_create_default();
    if (loop != ESP_OK && loop != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(loop);
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        on_wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        on_wifi_event, NULL, NULL));

    wifi_config_t config = {0};
    strncpy((char *)config.sta.ssid, CONFIG_PROBE_WIFI_SSID, sizeof(config.sta.ssid) - 1);
    strncpy((char *)config.sta.password, CONFIG_PROBE_WIFI_PASSWORD,
            sizeof(config.sta.password) - 1);

    /* Channels 12 and 13 are legal in much of the world but the default
     * country setting ("01") only scans 1-11, so an AP up there is invisible
     * and every connect fails with reason 201, NO_AP_FOUND. */
    ESP_ERROR_CHECK(esp_wifi_set_country_code(CONFIG_PROBE_WIFI_COUNTRY, true));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
    /* Keep Wi-Fi power save ON. Counter-intuitively this is what makes
     * coexistence work: modem sleep is how Wi-Fi hands the shared radio to the
     * Zigbee stack. With WIFI_PS_NONE the two fight over it, Wi-Fi misses
     * beacons, and the connection drops seconds after the Zigbee stack starts. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

#if CONFIG_PROBE_WIFI_SCAN_ON_BOOT
    /* Report what the radio can actually see, so a wrong SSID or a band-split
     * network is obvious rather than showing up as NO_AP_FOUND. The STA_START
     * handler has already started connecting, and a scan during a connect
     * attempt returns immediately with nothing -- so stop that first. */
    esp_wifi_disconnect();
    const esp_err_t scan = esp_wifi_scan_start(NULL, true);
    if (scan != ESP_OK) ESP_LOGW(TAG, "scan failed: %s", esp_err_to_name(scan));
    uint16_t found = 0;
    esp_wifi_scan_get_ap_num(&found);
    if (found > 20) found = 20;
    /* On the heap: a wifi_ap_record_t is around 100 bytes, and 20 of them
     * overflow the main task's default 3.5 kB stack outright. */
    wifi_ap_record_t *records = calloc(found ? found : 1, sizeof(*records));
    if (records && found && esp_wifi_scan_get_ap_records(&found, records) == ESP_OK) {
        for (uint16_t i = 0; i < found; i++) {
            ESP_LOGW(TAG, "scan: ch%-3d %4d dBm  %s%s", records[i].primary, records[i].rssi,
                     (const char *)records[i].ssid,
                     strcmp((const char *)records[i].ssid, CONFIG_PROBE_WIFI_SSID) == 0
                         ? "   <-- configured SSID" : "");
        }
    }
    free(records);
    ESP_LOGW(TAG, "scan: %d APs visible", found);
    esp_wifi_connect();
#endif

    xTaskCreate(sender_task, "wifilog", 4096, NULL, 3, NULL);
    s_previous = esp_log_set_vprintf(log_hook);

    /* Associate before the caller starts Zigbee. A Zigbee coordinator keeps
     * the shared radio in receive permanently, and association cannot survive
     * that -- it does not even get through authentication. Once the link is
     * up, coexistence keeps it alive; establishing it is the fragile part. */
    const EventBits_t bits = xEventGroupWaitBits(s_events, GOT_IP_BIT, pdFALSE, pdTRUE,
                                                 pdMS_TO_TICKS(15000));
    if (!(bits & GOT_IP_BIT)) {
        ESP_LOGW(TAG, "no IP after 15 s, continuing without network logging");
    }
}
