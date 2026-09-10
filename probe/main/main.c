/*
 * Zigbee remote probe for the ESP32-C6.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Forms a Zigbee network as coordinator, holds the network open for joining,
 * and logs every command that arrives. The point is to discover exactly what
 * a given remote emits -- endpoint, cluster, command -- so the real firmware
 * can act on it.
 *
 * The device advertises itself as an on/off light with level and scenes
 * support, because that is what a remote looks for during Find & Bind: it
 * binds its client clusters to a matching server, and only then do its
 * button presses get delivered here.
 */

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "bdb/esp_zigbee_bdb_touchlink.h"
#include "zdo/esp_zigbee_zdo_command.h"
#include "test/esp_zigbee_test_utils.h"
#include "esp_zigbee_secur.h"
#include "nwk/esp_zigbee_nwk.h"
#include "zboss_api.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "nvs_flash.h"
#include "wifi_log.h"
#include <string.h>
#include <stdio.h>

/* The ZBOSS stack destabilises the C6's native USB-Serial-JTAG, so serial
 * logs cut out. The on-board RGB reports what happened instead:
 *   dim blue breathing = network up and open, nothing joined
 *   three green flashes = a device joined
 *   one white flash     = a command arrived (button press)
 */
/* Board specific: the pins of whatever addressable LED your board carries.
 * These are for a board with a WS2812 on GPIO20 and its power rail on GPIO19. */
#define PIN_RGB_PWR  GPIO_NUM_19
#define PIN_RGB_DATA GPIO_NUM_20

#define PROBE_ENDPOINT 1

/* An IKEA BILRESA groupcasts to one fixed group per channel. A device only
 * receives a groupcast if its endpoint is a member of that group, so join all
 * three to see traffic from any channel. */
static const uint16_t PROBE_GROUPS[] = {21658, 21659, 21660};
#define PERMIT_JOIN_S  180

/* Zigbee and Wi-Fi share one radio, so a coordinator on a channel that
 * overlaps a busy Wi-Fi network fares badly. Channels 25 and 26 (2475 and
 * 2480 MHz) sit above Wi-Fi channel 11 and clear of 1 and 6 -- unlike 15,
 * which lands on the top edge of Wi-Fi channel 1. */
#define PROBE_CHANNEL_MASK (1l << 25 | 1l << 26)

static const char *TAG = "zbprobe";

static led_strip_handle_t s_led;
static volatile int s_join_flashes;
static volatile int s_cmd_flashes;

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_led) return;
    led_strip_set_pixel(s_led, 0, r, g, b);
    led_strip_refresh(s_led);
}

static void led_task(void *arg)
{
    int t = 0;
    while (true) {
        if (s_cmd_flashes > 0) {
            s_cmd_flashes--;
            led_set(120, 120, 120);
            vTaskDelay(pdMS_TO_TICKS(90));
            led_set(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(90));
        } else if (s_join_flashes > 0) {
            s_join_flashes--;
            led_set(0, 160, 0);
            vTaskDelay(pdMS_TO_TICKS(150));
            led_set(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(150));
        } else {
            /* Slow blue breathe: alive, network open, nothing heard yet. */
            int v = (t < 30) ? t : (60 - t);
            led_set(0, 0, v / 3);
            t = (t + 1) % 60;
            vTaskDelay(pdMS_TO_TICKS(40));
        }
    }
}

static void led_start(void)
{
    gpio_config_t pwr = {.pin_bit_mask = BIT64(PIN_RGB_PWR),
                         .mode = GPIO_MODE_OUTPUT};
    gpio_config(&pwr);
    gpio_set_level(PIN_RGB_PWR, 1);

    led_strip_config_t cfg = {
        .strip_gpio_num = PIN_RGB_DATA,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt = {.clk_src = RMT_CLK_SRC_DEFAULT,
                                  .resolution_hz = 10 * 1000 * 1000};
    if (led_strip_new_rmt_device(&cfg, &rmt, &s_led) == ESP_OK) {
        led_strip_clear(s_led);
        xTaskCreate(led_task, "led", 2560, NULL, 4, NULL);
    }
}

static const char *cluster_name(uint16_t id)
{
    switch (id) {
    case 0x0000: return "Basic";
    case 0x0003: return "Identify";
    case 0x0004: return "Groups";
    case 0x0005: return "Scenes";
    case 0x0006: return "On/Off";
    case 0x0008: return "Level";
    case 0x0019: return "OTA";
    case 0x0300: return "Color";
    case 0x0b05: return "Diagnostics";
    default:     return "?";
    }
}

/* Everything the stack decodes for us arrives here. */
static esp_err_t action_handler(esp_zb_core_action_callback_id_t id,
                                const void *message)
{
    switch (id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID: {
        const esp_zb_zcl_set_attr_value_message_t *m = message;
        s_cmd_flashes += 2;
        ESP_LOGW(TAG, "ATTR  ep=%d cluster=0x%04x (%s) attr=0x%04x len=%d",
                 m->info.dst_endpoint, m->info.cluster,
                 cluster_name(m->info.cluster), m->attribute.id,
                 m->attribute.data.size);
        if (m->attribute.data.value && m->attribute.data.size <= 4) {
            uint32_t v = 0;
            memcpy(&v, m->attribute.data.value, m->attribute.data.size);
            ESP_LOGW(TAG, "      value=%lu", (unsigned long)v);
        }
        break;
    }
    case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID: {
        /* IKEA sends its arrow buttons as manufacturer-specific commands,
         * which arrive here rather than as attribute writes. */
        const esp_zb_zcl_custom_cluster_command_message_t *m = message;
        s_cmd_flashes += 2;
        ESP_LOGW(TAG, "CUSTOM ep=%d cluster=0x%04x (%s) cmd=0x%02x size=%d",
                 m->info.dst_endpoint, m->info.cluster,
                 cluster_name(m->info.cluster), m->info.command.id,
                 m->data.size);
        break;
    }
    case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
        break; /* housekeeping, not a button press */
    default:
        ESP_LOGW(TAG, "CORE  callback id 0x%x", id);
        break;
    }
    return ESP_OK;
}

/* Keep re-opening rather than relying on one 180 s window: a test tool that
 * quietly closes while you are hunting for the pairing button is useless. */
/* The attribute callback the stack normally hands you carries only status,
 * endpoint and cluster -- the addressing is already gone. A remote that
 * groupcasts (as a Touchlink-bound IKEA remote does) puts the group id in the
 * APS destination, and that is the only way to tell which of its channels a
 * press came from. Take the raw buffer to see it, then let the stack carry on.
 */
static bool raw_command(uint8_t bufid)
{
    const zb_zcl_parsed_hdr_t *h = ZB_BUF_GET_PARAM(bufid, zb_zcl_parsed_hdr_t);
    if (h) {
        const uint16_t dst = h->addr_data.common_data.dst_addr;
        ESP_LOGW(TAG, "RAW   src=0x%04x dst=0x%04x%s ep %d->%d cluster=0x%04x cmd=0x%02x",
                 h->addr_data.common_data.source.u.short_addr, dst,
                 dst >= 0x8000 ? "" : " (group)",
                 h->addr_data.common_data.src_endpoint,
                 h->addr_data.common_data.dst_endpoint,
                 h->cluster_id, h->cmd_id);
    }
    return false; /* not consumed -- normal processing continues */
}

/* Touchlink encrypts the transferred network key with the certification key
 * (index 15, public) or the master key (index 4, shared by certified devices).
 * The SDK advertises both by default and prefers the certification key, since
 * priority follows the higher bit. Installing a master key and advertising only
 * that removes the ambiguity about which key was selected. */
static void install_master_key(void)
{
    const char *hex = CONFIG_PROBE_TOUCHLINK_MASTER_KEY;
    if (strlen(hex) != 32) {
        if (strlen(hex) != 0) {
            ESP_LOGE(TAG, "master key must be 32 hex characters, ignoring");
        }
        return;
    }
    uint8_t key[16];
    for (int i = 0; i < 16; i++) {
        unsigned byte = 0;
        if (sscanf(hex + 2 * i, "%2x", &byte) != 1) {
            ESP_LOGE(TAG, "master key is not valid hex, ignoring");
            return;
        }
        key[i] = (uint8_t)byte;
    }
    esp_zb_zdo_touchlink_set_master_key(key);
    esp_zb_zdo_touchlink_set_key_bitmask(ESP_ZB_TOUCHLINK_MASTER_KEY);
    ESP_LOGW(TAG, "master key installed, advertising master key only");
}

/* Which network are we actually on? Touchlink can start a network as well as
 * join one, and the signal does not say which happened. If the parameters here
 * are not the remote's, the two sides are on separate networks with separate
 * keys, which is enough on its own to explain rejected frames. */
static void report_network(const char *when)
{
    esp_zb_ieee_addr_t ext_pan = {0}, me = {0};
    esp_zb_get_extended_pan_id(ext_pan);
    esp_zb_get_long_address(me);
    ESP_LOGW(TAG, "%s: pan 0x%04hx ext_pan %02x%02x%02x%02x%02x%02x%02x%02x "
                  "channel %d short 0x%04hx",
             when, esp_zb_get_pan_id(),
             ext_pan[7], ext_pan[6], ext_pan[5], ext_pan[4],
             ext_pan[3], ext_pan[2], ext_pan[1], ext_pan[0],
             esp_zb_get_current_channel(), esp_zb_get_short_address());
}

/* Print the whole key so a sniffer capture can be decrypted offline. */
static void report_network_key(const char *when)
{
    uint8_t k[16] = {0};
    if (esp_zb_secur_primary_network_key_get(k) != ESP_OK) {
        ESP_LOGW(TAG, "%s: no network key", when);
        return;
    }
    ESP_LOGW(TAG, "%s: network key "
             "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
             when, k[0], k[1], k[2], k[3], k[4], k[5], k[6], k[7],
             k[8], k[9], k[10], k[11], k[12], k[13], k[14], k[15]);
}

/* NWK status 0x12 says a frame arrived with a key sequence number we do not
 * hold. Report the key we ended up with, then try the plausible sequence
 * numbers in turn. If one of them stops the errors, the key material was fine
 * and only the sequence number disagreed. */
static void try_key_sequence(uint8_t param)
{
    const uint8_t seq = (uint8_t)param;
    report_network("after touchlink");
    uint8_t key[16] = {0};
    if (esp_zb_secur_primary_network_key_get(key) != ESP_OK) {
        ESP_LOGE(TAG, "cannot read network key");
        return;
    }
    bool zero = true;
    for (int i = 0; i < 16; i++) {
        if (key[i]) { zero = false; break; }
    }
    ESP_LOGW(TAG, "network key starts %02x%02x%02x%02x%s, switching to seq %u",
             key[0], key[1], key[2], key[3], zero ? " (ALL ZERO)" : "", seq);
    const esp_err_t err = esp_zb_secur_network_key_switch(key, seq);
    ESP_LOGW(TAG, "key switch to seq %u: %s", seq, esp_err_to_name(err));
    if (seq < 2) {
        esp_zb_scheduler_alarm(try_key_sequence, seq + 1, 10000);
    }
}

static bool touchlink_allow(uint8_t action)
{
    s_join_flashes += 3;
    ESP_LOGW(TAG, "*** TOUCHLINK request, action=%d -- allowing ***", action);
    return true;
}

static void reopen_alarm(uint8_t arg)
{
    esp_zb_bdb_open_network(PERMIT_JOIN_S);
    /* Re-arm Touchlink target too: it also times out, and the BILRESA's
     * hidden Zigbee mode commissions this way rather than by plain joining. */
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_TOUCHLINK_TARGET);
    esp_zb_scheduler_alarm(reopen_alarm, 0, (PERMIT_JOIN_S - 20) * 1000);
}

static void retry_formation(uint8_t arg)
{
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
}

/* Only valid once the network exists -- calling it earlier just logs
 * "Device not in a network". */
static void open_for_joining(void)
{
    esp_zb_ieee_addr_t ext_pan;
    esp_zb_get_extended_pan_id(ext_pan);
    ESP_LOGI(TAG, "network formed: channel %d, pan 0x%04hx",
             esp_zb_get_current_channel(), esp_zb_get_pan_id());
    esp_zb_bdb_open_network(PERMIT_JOIN_S);
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_TOUCHLINK_TARGET);
    esp_zb_scheduler_alarm(reopen_alarm, 0, (PERMIT_JOIN_S - 20) * 1000);
    ESP_LOGW(TAG, "*** network OPEN + touchlink target active -- now put the remote "
                  "into a joining mode. For an IKEA BILRESA that is 4x rapid presses "
                  "of the pairing button (touchlink), then 8x more (plain Zigbee join, "
                  "LEDs go dark) ***");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal)
{
    uint32_t *p = signal->p_app_signal;
    esp_err_t err = signal->esp_err_status;
    esp_zb_app_signal_type_t sig = *p;

    switch (sig) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "initialisation failed: %s", esp_err_to_name(err));
            break;
        }
        if (CONFIG_PROBE_ROLE_ROUTER) {
            /* Stay factory new and wait to be adopted. A remote that is off
             * network creates one during Touchlink and pulls the target in,
             * which a coordinator that has formed its own network cannot
             * accept. Only a joinable router or end device can be a target. */
            ESP_LOGW(TAG, "router role: waiting as a touchlink target, do not "
                          "expect a network of our own");
            report_network("before touchlink");
            report_network_key("before touchlink");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_TOUCHLINK_TARGET);
        } else if (esp_zb_bdb_is_factory_new()) {
            ESP_LOGI(TAG, "no network stored, forming one");
            esp_zb_bdb_start_top_level_commissioning(
                ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        } else {
            /* Already formed on a previous boot: just reopen for joining. */
            open_for_joining();
        }
        break;

    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err == ESP_OK) {
            open_for_joining();
        } else {
            ESP_LOGE(TAG, "formation failed: %s -- retrying",
                     esp_err_to_name(err));
            esp_zb_scheduler_alarm(retry_formation, 0, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_TOUCHLINK_NWK:
        report_network("touchlink network");
        break;

    case ESP_ZB_NLME_STATUS_INDICATION: {
        esp_zb_zdo_signal_nwk_status_indication_params_t *n =
            (esp_zb_zdo_signal_nwk_status_indication_params_t *)
                esp_zb_app_signal_get_params(p);
        if (n) {
            ESP_LOGW(TAG, "NLME status 0x%02x from 0x%04hx", n->status,
                     n->network_addr);
        }
        break;
    }

    case ESP_ZB_BDB_SIGNAL_TOUCHLINK_TARGET_FINISHED:
        report_network("target finished");
        esp_zb_scheduler_alarm(try_key_sequence, 0, 8000);
        for (size_t i = 0; i < sizeof(PROBE_GROUPS) / sizeof(PROBE_GROUPS[0]); i++) {
            const esp_err_t g = esp_zb_aps_group_table_add_group(PROBE_GROUPS[i],
                                                                 PROBE_ENDPOINT);
            ESP_LOGW(TAG, "group %u on endpoint %d: %s", PROBE_GROUPS[i],
                     PROBE_ENDPOINT, esp_err_to_name(g));
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        ESP_LOGI(TAG, "steering: %s", esp_err_to_name(err));
        break;

    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        esp_zb_zdo_signal_device_annce_params_t *d =
            (esp_zb_zdo_signal_device_annce_params_t *)
                esp_zb_app_signal_get_params(p);
        s_join_flashes += 3;
        ESP_LOGW(TAG, "*** DEVICE JOINED: short addr 0x%04hx ***",
                 d->device_short_addr);
        break;
    }
    default:
        ESP_LOGI(TAG, "signal %d %s (%s)", sig,
                 esp_zb_zdo_signal_to_string(sig), esp_err_to_name(err));
        break;
    }
}

static void zigbee_task(void *arg)
{
    esp_zb_cfg_t cfg = {
        .esp_zb_role = CONFIG_PROBE_ROLE_ROUTER ? ESP_ZB_DEVICE_TYPE_ROUTER
                                                : ESP_ZB_DEVICE_TYPE_COORDINATOR,
        .install_code_policy = false,
        .nwk_cfg.zczr_cfg = {.max_children = 10},
    };
    esp_zb_init(&cfg);

    /* Present as a light: a remote's Find & Bind looks for a matching server
     * cluster, and binds to it. Without these it has nothing to talk to. */
    esp_zb_on_off_light_cfg_t light_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
    esp_zb_ep_list_t *ep = esp_zb_on_off_light_ep_create(PROBE_ENDPOINT,
                                                         &light_cfg);

    /* Level and Scenes on top, so dimming and the scene buttons bind here
     * as well rather than being refused. */
    esp_zb_cluster_list_t *clusters =
        esp_zb_ep_list_get_ep(ep, PROBE_ENDPOINT);
    /* Scenes already ships with the on/off light endpoint; only Level has to
     * be added, so the dimming buttons have a server to bind to. */
    esp_zb_level_cluster_cfg_t level_cfg = {.current_level = 128};
    esp_zb_cluster_list_add_level_cluster(
        clusters, esp_zb_level_cluster_create(&level_cfg),
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_device_register(ep);
    esp_zb_core_action_handler_register(action_handler);
    esp_zb_touchlink_action_check_register(touchlink_allow);
    esp_zb_raw_command_handler_register(raw_command);
    /* Touchlink is proximity-based and rejects weak signals; relax it so the
     * remote does not have to be pressed against the board. */
    install_master_key();
    esp_zb_zdo_touchlink_set_rssi_threshold(-90);
    esp_zb_zdo_touchlink_target_set_timeout(300);
    esp_zb_set_primary_network_channel_set(PROBE_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());


    /* Turn the stack chatty: we want to see association attempts that fail,
     * not just ones that succeed. */
    esp_log_level_set("ESP_ZB_*", ESP_LOG_DEBUG);
    esp_log_level_set("ZBOSS", ESP_LOG_DEBUG);
    led_start();
    /* esp_zb_platform_config() must come first: it brings up the shared radio
     * PHY. Start Wi-Fi before it and the PHY is never initialised for Wi-Fi --
     * a scan then sees zero access points and every connect fails with reason
     * 201, NO_AP_FOUND. */
    esp_zb_platform_config_t platform = {
        .radio_config = {.radio_mode = ZB_RADIO_MODE_NATIVE},
        .host_config = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE},
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&platform));

    /* Then Wi-Fi, and only then the Zigbee stack itself: association has to
     * complete before a coordinator parks the radio in permanent receive. */
    wifi_log_start();
    ESP_LOGW(TAG, "Zigbee remote probe starting");
    xTaskCreate(zigbee_task, "zigbee", 8192, NULL, 5, NULL);
}
