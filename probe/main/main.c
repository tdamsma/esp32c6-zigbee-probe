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
#include "aps/esp_zigbee_aps.h"
#include "test/esp_zigbee_test_utils.h"
#include "esp_zigbee_secur.h"
#include "nwk/esp_zigbee_nwk.h"
#include "zboss_api.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "led_strip.h"
#include "nvs_flash.h"
#include "wifi_log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_system.h"

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
#define PROBE_GROUP_COUNT (sizeof(PROBE_GROUPS) / sizeof(PROBE_GROUPS[0]))

/* The SDK's APS callback reports a groupcast as a plain broadcast and drops
 * the group id, so the group cannot be read off the frame. Giving each group
 * its own endpoint puts the channel back in a field that does survive:
 * a frame for 21659 is delivered to endpoint 2 and nowhere else. */
#define PROBE_EP_OF_CHANNEL(i) ((uint8_t)(PROBE_ENDPOINT + (i)))
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

/* The wheel is one control for two parameters, so a click swaps which one it
 * drives. BILRESA sends alternating On/Off for a single click, a
 * manufacturer-specific arrow command for a double click, and MoveToLevel
 * while rotating. See bilresa-e2490-reference.md section 7.
 *
 * Which gesture does what is runtime state, not compile-time state: the
 * serial console below rebinds it, so trying a different mapping does not
 * mean rebuilding and reflashing, which would cost the Touchlink pairing. */

typedef enum {
    ACT_NONE = 0,   /* ignore the gesture */
    ACT_MODE,       /* swap between hue and brightness */
    ACT_ONOFF,      /* toggle the light */
    ACT_ACTIVE,     /* drive whichever parameter is currently selected */
    ACT_HUE,        /* always drive hue */
    ACT_BRIGHT,     /* always drive brightness */
    ACT_SAT,        /* always drive saturation */
    ACT_BLINK,      /* always drive blink rate */
    ACT_COUNT,
} light_action_t;

typedef enum { GEST_CLICK = 0, GEST_DOUBLE, GEST_SCROLL, GEST_COUNT } gesture_t;

typedef enum { LIGHT_MODE_BRIGHT = 0, LIGHT_MODE_HUE } light_mode_t;

/* Packed so it can go to NVS as one blob and survive the reset that the
 * capture script does on every connect. */
typedef struct {
    uint8_t map[GEST_COUNT];
    uint8_t chan[3];    /* scroll action per BILRESA channel, 1-3 */
    uint8_t mode;
    uint8_t on;
    uint16_t hue;       /* degrees, 0-359 */
    uint8_t sat;
    uint8_t bright;
    uint8_t blink;      /* 0 steady, otherwise faster as it rises */
} light_cfg_t;

static light_cfg_t s_cfg = {
    .map = {[GEST_CLICK] = ACT_MODE,
            [GEST_DOUBLE] = ACT_ONOFF,
            [GEST_SCROLL] = ACT_ACTIVE},
    /* The remote's channel is the only per-gesture context we get, so give
     * each one its own parameter rather than sharing one wheel. */
    .chan = {ACT_BRIGHT, ACT_HUE, ACT_BLINK},
    .mode = LIGHT_MODE_BRIGHT,
    .on = 1,
    .hue = 30,
    .sat = 255,
    .bright = 160,
};

static volatile bool s_light_seen;      /* a command has arrived */
static volatile int s_mode_blips;
static volatile int s_rx_blip;         /* one white flash per command */
static TickType_t s_last_blip;
static volatile uint16_t s_last_dst = 0xffff;   /* NWK destination last seen */
/* The remote is the Touchlink initiator, so it takes 0x0001 and the probe
 * 0x0002. Tracked anyway rather than hardcoded, in case a later pairing
 * lands differently. */
static volatile uint16_t s_remote_addr = 0x0001;
static volatile uint16_t s_last_group;          /* group of the last frame, 0 if none */
static volatile int s_last_channel = -1;        /* 0-2, from the endpoint */
static volatile bool s_rxlog = true;
static volatile uint16_t s_filter;      /* 0 = accept any destination */

static void report_network(const char *when);
static void regroup_now(void);
static void report_network_key(const char *when);
static void try_key_sequence(uint8_t param);

static const char *ACT_NAMES[ACT_COUNT] = {
    "none", "mode", "onoff", "active", "hue", "bright", "sat", "blink",
};
static const char *GEST_NAMES[GEST_COUNT] = {"click", "double", "scroll"};

#define LIGHT_NVS_NS  "light"
#define LIGHT_NVS_KEY "cfg"

static void light_save(void)
{
    nvs_handle_t h;
    if (nvs_open(LIGHT_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, LIGHT_NVS_KEY, &s_cfg, sizeof(s_cfg));
    nvs_commit(h);
    nvs_close(h);
}

static void light_load(void)
{
    nvs_handle_t h;
    if (nvs_open(LIGHT_NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    size_t len = sizeof(s_cfg);
    light_cfg_t tmp;
    if (nvs_get_blob(h, LIGHT_NVS_KEY, &tmp, &len) == ESP_OK &&
        len == sizeof(tmp)) {
        bool sane = tmp.mode <= LIGHT_MODE_HUE && tmp.hue < 360;
        for (int g = 0; g < GEST_COUNT; g++)
            if (tmp.map[g] >= ACT_COUNT) sane = false;
        for (int c = 0; c < 3; c++)
            if (tmp.chan[c] >= ACT_COUNT) sane = false;
        if (sane) s_cfg = tmp;
    }
    nvs_close(h);
}

static void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v,
                       uint8_t *r, uint8_t *g, uint8_t *b)
{
    const uint8_t region = h / 60;
    const uint16_t rem = (h - region * 60) * 255 / 60;
    const uint8_t p = (uint16_t)v * (255 - s) / 255;
    const uint8_t q = (uint16_t)v * (255 - (uint32_t)s * rem / 255) / 255;
    const uint8_t t = (uint16_t)v * (255 - (uint32_t)s * (255 - rem) / 255) / 255;
    switch (region % 6) {
    case 0:  *r = v; *g = t; *b = p; break;
    case 1:  *r = q; *g = v; *b = p; break;
    case 2:  *r = p; *g = v; *b = t; break;
    case 3:  *r = p; *g = q; *b = v; break;
    case 4:  *r = t; *g = p; *b = v; break;
    default: *r = v; *g = p; *b = q; break;
    }
}

static void light_render(void)
{
    if (!s_cfg.on) {
        led_set(0, 0, 0);
        return;
    }
    uint8_t r, g, b;
    hsv_to_rgb(s_cfg.hue, s_cfg.sat, s_cfg.bright, &r, &g, &b);
    led_set(r, g, b);
}

static void light_swap_mode(void)
{
    s_cfg.mode = (s_cfg.mode == LIGHT_MODE_HUE) ? LIGHT_MODE_BRIGHT
                                                : LIGHT_MODE_HUE;
    s_mode_blips = 2;
    ESP_LOGW(TAG, "LIGHT mode=%s",
             s_cfg.mode == LIGHT_MODE_HUE ? "hue" : "bright");
}

/* Rotating sends a ramp of MoveToLevel steps across the transition, so
 * driving the parameter straight from the attribute gives smooth movement. */
static void light_set_param(light_action_t act, uint8_t level)
{
    if (level > 254) level = 254;   /* the remote can send an out-of-spec 255 */
    if (act == ACT_ACTIVE)
        act = (s_cfg.mode == LIGHT_MODE_HUE) ? ACT_HUE : ACT_BRIGHT;
    switch (act) {
    case ACT_HUE:    s_cfg.hue = (uint16_t)level * 359 / 254; break;
    case ACT_SAT:    s_cfg.sat = level; break;
    case ACT_BRIGHT: s_cfg.bright = level; break;
    case ACT_BLINK:  s_cfg.blink = level; break;
    default: break;
    }
}

/* Rate limited: rotating sends about ten level steps a second, and a flash on
 * every one of them reads as a strobe rather than as confirmation. */
static void light_blip(void)
{
    const TickType_t now = xTaskGetTickCount();
    if (now - s_last_blip < pdMS_TO_TICKS(250)) return;
    s_last_blip = now;
    s_rx_blip = 1;
}

static void light_gesture(gesture_t g, uint8_t level)
{
    light_action_t act = s_cfg.map[g];
    const int ch = s_last_channel;
    if (g == GEST_SCROLL && ch >= 0) act = s_cfg.chan[ch];
    /* filter 0 means act on everything; otherwise only on that group, which
     * is how a single BILRESA channel gets isolated once the remote actually
     * groupcasts rather than broadcasts. */
    if (s_filter && s_last_group != s_filter) {
        ESP_LOGW(TAG, "LIGHT ignored, group=%u filter=%u",
                 s_last_group, s_filter);
        return;
    }
    s_light_seen = true;
    light_blip();
    switch (act) {
    case ACT_NONE:  break;
    case ACT_MODE:  light_swap_mode(); break;
    case ACT_ONOFF:
        s_cfg.on = !s_cfg.on;
        ESP_LOGW(TAG, "LIGHT %s", s_cfg.on ? "on" : "off");
        break;
    default: light_set_param(act, level); break;
    }
}

static int action_from_name(const char *s)
{
    for (int i = 0; i < ACT_COUNT; i++)
        if (strcmp(s, ACT_NAMES[i]) == 0) return i;
    return -1;
}

static void light_status(void)
{
    printf("light on=%d mode=%s hue=%u sat=%u bright=%u blink=%u\n", s_cfg.on,
           s_cfg.mode == LIGHT_MODE_HUE ? "hue" : "bright", s_cfg.hue,
           s_cfg.sat, s_cfg.bright, s_cfg.blink);
    for (int c = 0; c < 3; c++)
        printf("chan %d (group %u) -> %s\n", c + 1, PROBE_GROUPS[c],
               ACT_NAMES[s_cfg.chan[c]]);
    for (int g = 0; g < GEST_COUNT; g++)
        printf("map %s -> %s\n", GEST_NAMES[g], ACT_NAMES[s_cfg.map[g]]);
}

static void console_help(void)
{
    printf("commands:\n"
           "  status                     show light state and gesture map\n"
           "  on | off | toggle          light power\n"
           "  hue <0-359>                set hue\n"
           "  sat <0-255>                set saturation\n"
           "  bright <0-255>             set brightness\n"
           "  hsv <h> <s> <v>            set all three\n"
           "  mode hue|bright            which parameter the wheel drives\n"
           "  chan <1-3> <action>        scroll role per remote channel\n"
           "  blink <0-254>              0 steady, higher is faster\n"
           "  map <click|double|scroll> <none|mode|onoff|active|hue|bright|sat>\n"
           "  blip                       flash white once, to test the LED\n"
           "zigbee:\n"
           "  pair                       arm Touchlink target, then 4 presses\n"
           "  steer | open [secs]        plain join / permit joining\n"
           "  net                        pan, channel, short address, key\n"
           "  groups | group <id>        group membership, one endpoint per channel\n"
           "  filter <any|group>         only act on that group, eg 21658\n"
           "  radio <11-26>              primary channel for next commissioning\n"
           "  keyseq <n|off>             switch key sequence, or stop the sweep\n"
           "  identify [secs] [ep]       send Identify to the remote\n"
           "  effect [id] [variant] [ep] send Identify Trigger Effect\n"
           "  rxlog on|off               per-frame logging\n"
           "  factory yes | reboot       wipe the pairing / restart\n"
           "  save | load | defaults     gesture map and colour in NVS\n"
           "  help\n");
}

static void console_line(char *line)
{
    char *save = NULL;
    const char *cmd = strtok_r(line, " \t", &save);
    if (!cmd) return;
    const char *a1 = strtok_r(NULL, " \t", &save);
    const char *a2 = strtok_r(NULL, " \t", &save);
    const char *a3 = strtok_r(NULL, " \t", &save);

    if (!strcmp(cmd, "help")) {
        console_help();
    } else if (!strcmp(cmd, "status")) {
        light_status();
    } else if (!strcmp(cmd, "on") || !strcmp(cmd, "off")) {
        s_cfg.on = (cmd[1] == 'n');
        s_light_seen = true;
    } else if (!strcmp(cmd, "toggle")) {
        s_cfg.on = !s_cfg.on;
        s_light_seen = true;
    } else if (!strcmp(cmd, "hue") && a1) {
        s_cfg.hue = (uint16_t)(atoi(a1) % 360);
        s_light_seen = true;
    } else if (!strcmp(cmd, "sat") && a1) {
        s_cfg.sat = (uint8_t)atoi(a1);
        s_light_seen = true;
    } else if (!strcmp(cmd, "bright") && a1) {
        s_cfg.bright = (uint8_t)atoi(a1);
        s_light_seen = true;
    } else if (!strcmp(cmd, "hsv") && a1 && a2 && a3) {
        s_cfg.hue = (uint16_t)(atoi(a1) % 360);
        s_cfg.sat = (uint8_t)atoi(a2);
        s_cfg.bright = (uint8_t)atoi(a3);
        s_light_seen = true;
    } else if (!strcmp(cmd, "mode") && a1) {
        s_cfg.mode = !strcmp(a1, "hue") ? LIGHT_MODE_HUE : LIGHT_MODE_BRIGHT;
    } else if (!strcmp(cmd, "map") && a1 && a2) {
        int g = -1, act = action_from_name(a2);
        for (int i = 0; i < GEST_COUNT; i++)
            if (!strcmp(a1, GEST_NAMES[i])) g = i;
        if (g < 0 || act < 0) {
            printf("bad map, try: map scroll active\n");
        } else {
            s_cfg.map[g] = (uint8_t)act;
            printf("map %s -> %s\n", GEST_NAMES[g], ACT_NAMES[act]);
        }
    } else if (!strcmp(cmd, "pair")) {
        if (esp_zb_lock_acquire(pdMS_TO_TICKS(500))) {
            esp_zb_bdb_start_top_level_commissioning(
                ESP_ZB_BDB_MODE_TOUCHLINK_TARGET);
            esp_zb_lock_release();
            printf("touchlink target armed, 4 rapid presses on the remote\n");
        } else {
            printf("stack busy\n");
        }
    } else if (!strcmp(cmd, "steer")) {
        if (esp_zb_lock_acquire(pdMS_TO_TICKS(500))) {
            esp_zb_bdb_start_top_level_commissioning(
                ESP_ZB_BDB_MODE_NETWORK_STEERING);
            esp_zb_lock_release();
            printf("network steering started\n");
        }
    } else if (!strcmp(cmd, "open")) {
        const int secs = a1 ? atoi(a1) : PERMIT_JOIN_S;
        if (esp_zb_lock_acquire(pdMS_TO_TICKS(500))) {
            esp_zb_bdb_open_network((uint8_t)secs);
            esp_zb_lock_release();
            printf("network open for %d s\n", secs);
        }
    } else if (!strcmp(cmd, "identify")) {
        /* Whether a Zigbee peer can drive the remote's own LEDs. Over Matter
         * this is closed: Identify takes command 0 only and IdentifyType is
         * None. The remote sends Identify itself when a channel binds, so the
         * reverse direction is worth asking. */
        esp_zb_zcl_identify_cmd_t c = {
            .zcl_basic_cmd = {
                .dst_addr_u.addr_short = s_remote_addr,
                .dst_endpoint = (uint8_t)(a2 ? atoi(a2) : 1),
                .src_endpoint = PROBE_ENDPOINT,
            },
            .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
            .identify_time = (uint16_t)(a1 ? atoi(a1) : 3),
        };
        if (esp_zb_lock_acquire(pdMS_TO_TICKS(500))) {
            const uint8_t tsn = esp_zb_zcl_identify_cmd_req(&c);
            esp_zb_lock_release();
            printf("identify %u s to 0x%04x ep %u, tsn %u\n", c.identify_time,
                   s_remote_addr, c.zcl_basic_cmd.dst_endpoint, tsn);
        } else {
            printf("stack busy\n");
        }
    } else if (!strcmp(cmd, "effect")) {
        esp_zb_zcl_identify_trigger_effect_cmd_t c = {
            .zcl_basic_cmd = {
                .dst_addr_u.addr_short = s_remote_addr,
                .dst_endpoint = (uint8_t)(a3 ? atoi(a3) : 1),
                .src_endpoint = PROBE_ENDPOINT,
            },
            .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
            .effect_id = (uint8_t)(a1 ? strtol(a1, NULL, 0) : 0),
            .effect_variant = (uint8_t)(a2 ? strtol(a2, NULL, 0) : 0),
        };
        if (esp_zb_lock_acquire(pdMS_TO_TICKS(500))) {
            const uint8_t tsn = esp_zb_zcl_identify_trigger_effect_cmd_req(&c);
            esp_zb_lock_release();
            printf("effect 0x%02x variant 0x%02x to 0x%04x ep %u, tsn %u\n",
                   c.effect_id, c.effect_variant, s_remote_addr,
                   c.zcl_basic_cmd.dst_endpoint, tsn);
        } else {
            printf("stack busy\n");
        }
    } else if (!strcmp(cmd, "net")) {
        report_network("now");
        report_network_key("now");
        printf("last dst=0x%04x group=%u filter=%u\n", s_last_dst, s_last_group,
               s_filter);
    } else if (!strcmp(cmd, "groups")) {
        if (esp_zb_lock_acquire(pdMS_TO_TICKS(500))) {
            regroup_now();
            esp_zb_lock_release();
            printf("groups normalised, one per endpoint\n");
        } else {
            printf("stack busy\n");
        }
    } else if (!strcmp(cmd, "group") && a1) {
        const uint16_t id = (uint16_t)atoi(a1);
        esp_err_t e = ESP_FAIL;
        if (esp_zb_lock_acquire(pdMS_TO_TICKS(500))) {
            e = esp_zb_aps_group_table_add_group(id, PROBE_ENDPOINT);
            esp_zb_lock_release();
        }
        printf("group %u: %s\n", id, esp_err_to_name(e));
    } else if (!strcmp(cmd, "filter") && a1) {
        s_filter = strcmp(a1, "any") ? (uint16_t)atoi(a1) : 0;
        printf("filter=0x%04x (%u)\n", s_filter, s_filter);
    } else if (!strcmp(cmd, "chan") && a1 && a2) {
        const int c = atoi(a1), act = action_from_name(a2);
        if (c < 1 || c > 3 || act < 0) {
            printf("bad chan, try: chan 3 blink\n");
        } else {
            s_cfg.chan[c - 1] = (uint8_t)act;
            printf("chan %d (group %u) -> %s\n", c, PROBE_GROUPS[c - 1],
                   ACT_NAMES[act]);
        }
    } else if (!strcmp(cmd, "blink") && a1) {
        s_cfg.blink = (uint8_t)atoi(a1);
        s_light_seen = true;
    } else if (!strcmp(cmd, "radio") && a1) {
        const int ch = atoi(a1);
        if (ch < 11 || ch > 26) {
            printf("channel must be 11-26\n");
        } else if (esp_zb_lock_acquire(pdMS_TO_TICKS(500))) {
            esp_zb_set_primary_network_channel_set(1l << ch);
            esp_zb_lock_release();
            printf("primary channel set to %d, effective at next commissioning\n",
                   ch);
        }
    } else if (!strcmp(cmd, "keyseq") && a1) {
        if (!strcmp(a1, "off")) {
            for (uint8_t s = 0; s < 4; s++)
                esp_zb_scheduler_alarm_cancel(try_key_sequence, s);
            printf("automatic key sequence sweep cancelled\n");
        } else {
            try_key_sequence((uint8_t)atoi(a1));
        }
    } else if (!strcmp(cmd, "rxlog") && a1) {
        s_rxlog = !strcmp(a1, "on");
        printf("rxlog %s\n", s_rxlog ? "on" : "off");
    } else if (!strcmp(cmd, "factory")) {
        if (a1 && !strcmp(a1, "yes")) {
            printf("factory reset, this drops the pairing\n");
            fflush(stdout);
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_zb_factory_reset();
        } else {
            printf("this wipes the Touchlink pairing, confirm with: factory yes\n");
        }
    } else if (!strcmp(cmd, "reboot")) {
        printf("rebooting\n");
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
    } else if (!strcmp(cmd, "blip")) {
        s_last_blip = 0;
        light_blip();
        s_light_seen = true;
    } else if (!strcmp(cmd, "save")) {
        light_save();
        printf("saved\n");
    } else if (!strcmp(cmd, "load")) {
        light_load();
        light_status();
    } else if (!strcmp(cmd, "defaults")) {
        s_cfg.map[GEST_CLICK] = ACT_MODE;
        s_cfg.map[GEST_DOUBLE] = ACT_ONOFF;
        s_cfg.map[GEST_SCROLL] = ACT_ACTIVE;
        light_status();
    } else {
        printf("unknown: %s (try help)\n", cmd);
    }
}

/* Read through the driver rather than stdin: the ZBOSS stack destabilises the
 * USB-Serial-JTAG and the blocking ROM path is what drops out under load. */
static void console_task(void *arg)
{
    char line[128];
    int n = 0;
    while (true) {
        uint8_t ch;
        int got = usb_serial_jtag_read_bytes(&ch, 1, pdMS_TO_TICKS(100));
        if (got != 1) continue;
        if (ch == '\r' || ch == '\n') {
            if (n) {
                line[n] = 0;
                console_line(line);
                n = 0;
            }
            continue;
        }
        if (n < (int)sizeof(line) - 1) line[n++] = (char)ch;
    }
}

static void console_start(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    if (usb_serial_jtag_driver_install(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "console: driver install failed");
        return;
    }
    usb_serial_jtag_vfs_use_driver();
    light_load();
    xTaskCreate(console_task, "console", 4096, NULL, 3, NULL);
    ESP_LOGW(TAG, "console ready, type help");
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
        } else if (s_rx_blip > 0) {
            s_rx_blip = 0;
            led_set(255, 255, 255);
            vTaskDelay(pdMS_TO_TICKS(35));
            light_render();
            vTaskDelay(pdMS_TO_TICKS(35));
        } else if (s_mode_blips > 0) {
            /* Blink so a mode swap is visible even at a steady colour. */
            s_mode_blips--;
            led_set(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(70));
            light_render();
            vTaskDelay(pdMS_TO_TICKS(110));
        } else if (s_light_seen) {
            if (s_cfg.blink && s_cfg.on) {
                /* 254 maps to about 60 ms on, 1 to about a second. */
                const int half = 1000 - (int)s_cfg.blink * 37 / 10;
                led_set(0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(half < 60 ? 60 : half));
                light_render();
                vTaskDelay(pdMS_TO_TICKS(half < 60 ? 60 : half));
            } else {
                light_render();
                vTaskDelay(pdMS_TO_TICKS(20));
            }
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
        if (!s_light_seen) s_cmd_flashes += 2;
        ESP_LOGW(TAG, "ATTR  ep=%d cluster=0x%04x (%s) attr=0x%04x len=%d",
                 m->info.dst_endpoint, m->info.cluster,
                 cluster_name(m->info.cluster), m->attribute.id,
                 m->attribute.data.size);
        uint32_t v = 0;
        if (m->attribute.data.value && m->attribute.data.size <= 4) {
            memcpy(&v, m->attribute.data.value, m->attribute.data.size);
            ESP_LOGW(TAG, "      value=%lu", (unsigned long)v);
        }
        if (m->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL &&
            m->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID) {
            light_gesture(GEST_SCROLL, (uint8_t)v);
        }
        /* On/Off is handled in raw_command: the remote alternates on and off,
         * and a repeat of the value it already set raises no attribute
         * change at all, so the click would be lost here. */
        break;
    }
    case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID: {
        /* IKEA sends its arrow buttons as manufacturer-specific commands,
         * which arrive here rather than as attribute writes. */
        const esp_zb_zcl_custom_cluster_command_message_t *m = message;
        if (!s_light_seen) s_cmd_flashes += 2;
        ESP_LOGW(TAG, "CUSTOM ep=%d cluster=0x%04x (%s) cmd=0x%02x size=%d",
                 m->info.dst_endpoint, m->info.cluster,
                 cluster_name(m->info.cluster), m->info.command.id,
                 m->data.size);
        /* The arrow commands are the only manufacturer-specific traffic this
         * remote sends, and a double click is what emits them. */
        light_gesture(GEST_DOUBLE, 0);
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
/* A groupcast rides inside an NWK broadcast, so the ZCL header's dst_addr
 * shows 0xfffd and the group id is only visible here, in the APS indication.
 * This is the field that carries the BILRESA channel. */
/* The remote sends every Groups command to endpoint 1, so binding a channel
 * clears endpoint 1's membership and adds that channel's group there. That
 * breaks the one-group-per-endpoint layout the channel attribution depends
 * on, so put it back after any Groups traffic. */
static void regroup_alarm(uint8_t param)
{
    regroup_now();
}

static bool aps_indication(esp_zb_apsde_data_ind_t ind)
{
    const int ch = (int)ind.dst_endpoint - PROBE_ENDPOINT;
    if (ind.src_short_addr && ind.src_short_addr != 0xffff)
        s_remote_addr = ind.src_short_addr;
    s_last_channel = (ch >= 0 && ch < (int)PROBE_GROUP_COUNT) ? ch : -1;
    s_last_group = (s_last_channel >= 0) ? PROBE_GROUPS[s_last_channel] : 0;
    if (ind.cluster_id == ESP_ZB_ZCL_CLUSTER_ID_GROUPS) {
        esp_zb_scheduler_alarm_cancel(regroup_alarm, 0);
        esp_zb_scheduler_alarm(regroup_alarm, 0, 1500);
    }
    if (s_rxlog) {
        ESP_LOGW(TAG, "APS   src=0x%04x dst=0x%04x mode=0x%02x ep=%d chan=%d "
                      "group=%u cluster=0x%04x lqi=%d",
                 ind.src_short_addr, ind.dst_short_addr, ind.dst_addr_mode,
                 ind.dst_endpoint, s_last_channel + 1, s_last_group,
                 ind.cluster_id, ind.lqi);
    }
    return false;   /* not consumed, normal processing continues */
}

static void regroup_now(void)
{
    for (size_t i = 0; i < PROBE_GROUP_COUNT; i++) {
        for (size_t e = 0; e < PROBE_GROUP_COUNT; e++) {
            if (e == i) continue;
            esp_zb_aps_group_table_remove_group(PROBE_GROUPS[i],
                                                PROBE_EP_OF_CHANNEL(e));
        }
        const esp_err_t r = esp_zb_aps_group_table_add_group(
            PROBE_GROUPS[i], PROBE_EP_OF_CHANNEL(i));
        ESP_LOGW(TAG, "regroup: %u on ep %d: %s", PROBE_GROUPS[i],
                 PROBE_EP_OF_CHANNEL(i), esp_err_to_name(r));
    }
}

static bool raw_command(uint8_t bufid)
{
    const zb_zcl_parsed_hdr_t *h = ZB_BUF_GET_PARAM(bufid, zb_zcl_parsed_hdr_t);
    if (h) {
        const uint16_t dst = h->addr_data.common_data.dst_addr;
        const uint8_t fc = h->addr_data.common_data.fc;
        /* APS frame control bits 2-3 are the delivery mode. A groupcast is
         * carried inside an NWK broadcast, so the NWK destination alone
         * cannot tell a group frame from a plain broadcast. */
        static const char *DELIVERY[4] = {"unicast", "indirect", "bcast", "group"};
        const uint8_t mode = (fc >> 2) & 0x3;
        s_last_dst = dst;
        if (h->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
            (h->cmd_id == ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID ||
             h->cmd_id == ESP_ZB_ZCL_CMD_ON_OFF_ON_ID ||
             h->cmd_id == ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID)) {
            light_gesture(GEST_CLICK, 0);
        }
        if (!s_rxlog) return false;
        ESP_LOGW(TAG, "RAW   src=0x%04x dst=0x%04x %s (fc=0x%02x) ep %d->%d "
                      "cluster=0x%04x cmd=0x%02x",
                 h->addr_data.common_data.source.u.short_addr, dst,
                 DELIVERY[mode], fc,
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
 * hold. This switches the local sequence once, on request from the console.
 * It used to run automatically after every pairing and chain through 0, 1 and
 * 2, which changed the key out from under a working link and stopped
 * reception dead. Leave it manual. */
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
        regroup_now();
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

    /* Endpoints 2 and 3 carry the same light clusters as endpoint 1, so a
     * groupcast for channel 2 or 3 has somewhere to be delivered. */
    for (size_t i = 1; i < PROBE_GROUP_COUNT; i++) {
        esp_zb_on_off_light_cfg_t extra_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
        esp_zb_cluster_list_t *extra = esp_zb_on_off_light_clusters_create(&extra_cfg);
        esp_zb_level_cluster_cfg_t extra_level = {.current_level = 128};
        esp_zb_cluster_list_add_level_cluster(
            extra, esp_zb_level_cluster_create(&extra_level),
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        const esp_zb_endpoint_config_t epc = {
            .endpoint = PROBE_EP_OF_CHANNEL(i),
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
            .app_device_version = 0,
        };
        esp_zb_ep_list_add_ep(ep, extra, epc);
    }

    esp_zb_device_register(ep);
    esp_zb_core_action_handler_register(action_handler);
    esp_zb_touchlink_action_check_register(touchlink_allow);
    esp_zb_raw_command_handler_register(raw_command);
    esp_zb_aps_data_indication_handler_register(aps_indication);
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
    console_start();
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
