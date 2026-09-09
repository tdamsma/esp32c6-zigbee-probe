/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

/* Bring up Wi-Fi station mode and ship every log line out over UDP.
 *
 * The Zigbee stack destabilises the C6's USB-Serial-JTAG console, so serial
 * logging dies seconds after boot. This keeps a usable log. */
void wifi_log_start(void);
