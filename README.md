# ESP32-C6 Zigbee coordinator probe

This firmware runs a Zigbee coordinator on an ESP32-C6 and logs the endpoints,
clusters, commands and timing of messages sent by a remote.

It forms a network, keeps it open for joining, and logs decoded ZCL commands.
It was built to investigate an IKEA BILRESA in its undocumented Zigbee mode,
using an M5Stack NanoC6. The probe also works with other remotes. BILRESA
measurements are documented in
[ikea-bilresa-e2490](https://github.com/tdamsma/ikea-bilresa-e2490).

This document was drafted with Claude and OpenAI models. Hardware observations
and measurements are documented below and in the linked device notes.
I’m sharing this in the hope that it’s useful to others.

## USB serial connection loss

On the board used here, the console uses the ESP32-C6's built-in USB-Serial-JTAG
interface. With the Zigbee stack running, the connection was lost a few seconds
after boot:

```
ERROR: read failed: [Errno 6] Device not configured
```

Three capture attempts recorded nothing past the bootloader. The same board
streamed serial output for minutes with Wi-Fi/BLE firmware using the same
`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG` setting. These tests suggest the failure
is associated with the ZBOSS stack, though the cause has not been established.

The probe provides Wi-Fi logging and LED indications to allow testing when
USB serial is unavailable.

### Wi-Fi logging

ESP-IDF's software coexistence support schedules Wi-Fi and 802.15.4 access to
the shared radio, so in principle the probe can send logs over Wi-Fi while the
Zigbee stack runs.

This does not work on this SDK version. With `CONFIG_ZB_RADIO_NATIVE=y` the
Wi-Fi receiver finds zero access points and every connect fails with reason 201,
NO_AP_FOUND, even when the Zigbee task is never started. Espressif rates single
chip Wi-Fi plus Zigbee as supported but unstable and recommends a dual SoC
design for gateways. The code is kept because it is complete and works in a
build without Zigbee, but on this hardware the LED indications below remain the
practical way to see what is happening.

Note also that UDP broadcast does not cross subnets, so set
`PROBE_LOG_UDP_TARGET` when the listener is on another network.

Set your network in `idf.py menuconfig` under Zigbee probe, then listen:

```sh
nc -ul 5555
```

Log lines are broadcast by default, so no host address needs configuring. Set
`PROBE_LOG_UDP_TARGET` to a specific address if your network blocks broadcast.
Leaving the SSID empty disables Wi-Fi logging at compile time and uses serial.

The `esp_log` hook runs inside every `ESP_LOG` call, including calls from the
Wi-Fi stack. To avoid blocking or recursive logging, it formats messages in a
stack buffer and pushes them to a queue without logging or allocating memory.
It drops lines when the queue is full. Scan results are allocated on the heap,
because twenty `wifi_ap_record_t` entries overflow the main task stack and
trigger a stack protection fault.

Unrelated to Zigbee, this board needs `CONFIG_ESP_PHY_RF_CAL_FULL=y` for Wi-Fi
to receive at all. With the default partial calibration a scan found zero access
points while BLE scanning found twenty devices, and a stock ESP-IDF scan example
behaved identically until full calibration was enabled. Full calibration costs
about 100 ms of boot time and is set in `sdkconfig.defaults`.

Wi-Fi logging increased the application size from 622,640 to 1,240,208 bytes,
leaving 36% of the partition free.

### LED indications

The on-board LED indicates network and command activity without a log connection.

| LED | Meaning |
|---|---|
| Slow blue breathe | Network formed and open, no device joined |
| 3 × green | A device joined, or a Touchlink request arrived |
| 1 × white | A ZCL command arrived (button or wheel) |

Later serial sessions lasted about 16 seconds, allowing decoded commands to be
captured. The LED also confirmed that pairing had succeeded.

## Coordinator configuration

```
role            ESP_ZB_DEVICE_TYPE_COORDINATOR
channel         20  (mask 15/20/25/26, between Wi-Fi channels 1/6/11)
endpoint        1, HA on/off light with Level cluster added as a server
permit join     open, re-armed every 160 s
touchlink       target mode active, RSSI threshold -90  (not used in observed joins)
```

The probe advertises on/off and level server clusters so the remote can find a
binding target during Find & Bind. A coordinator without matching clusters
provides no target for these commands.

`esp_zb_on_off_light_ep_create()` already includes the Scenes cluster. Adding it
again fails with "cluster ID:0x5 is already existed".

## Destination group addresses

The incoming attribute-write callback provides `status`, `dst_endpoint` and
`cluster`, but omits the APS destination address. An integration using only this
callback cannot distinguish a remote's channels by their destination group.

A remote bound through Touchlink sends groupcasts to a fixed group ID for each
channel. BILRESA uses 21658, 21659 and 21660. Read the destination address from
the raw buffer to capture it:

```c
static bool raw_command(uint8_t bufid)
{
    const zb_zcl_parsed_hdr_t *h = ZB_BUF_GET_PARAM(bufid, zb_zcl_parsed_hdr_t);
    uint16_t dst = h->addr_data.common_data.dst_addr;   /* group id for a groupcast */
    return false;  /* not consumed -- normal processing continues */
}
esp_zb_raw_command_handler_register(raw_command);
```

The probe logs this address in a `RAW` line alongside each decoded command.
Returning `false` allows the stack to continue processing the command normally.

### Touchlink

The full procedure and evidence are in
[TOUCHLINK-INVESTIGATION.md](TOUCHLINK-INVESTIGATION.md).

Touchlink was tested against an IKEA BILRESA E2490 and does not reach a usable
link. The findings are worth recording because each stage fails differently.

A coordinator is the wrong role. While the probe formed its own network as
coordinator, the remote's Touchlink scan produced nothing at all, and the
callback registered with `esp_zb_touchlink_action_check_register()` never fired.
A remote that is off network does not join an existing network during Touchlink.
It creates one and adopts the target, which a coordinator cannot accept. Set
`PROBE_ROLE_ROUTER` to stay factory new and joinable.

As a router the handshake starts correctly:

```
*** TOUCHLINK request, action=1 -- allowing ***
signal 15 BDB Touchlink Network (ESP_OK)
group 21658 on endpoint 1: ESP_OK
```

It then fails at the network key. Every frame from the remote is rejected:

```
NLME status 0x12 from 0x0001    ZB_NWK_COMMAND_STATUS_BAD_KEY_SEQUENCE_NUMBER
```

repeating every few seconds, and the remote's LED keeps blinking because its
initiator never sees the exchange complete. No ZCL traffic reaches the
application, so the group addressing described above was never exercised
against real traffic.

Touchlink encrypts the transferred network key with either the certification
key, index 15, which is public and intended for testing, or the master key,
index 4, which is shared by certified Touchlink devices. The SDK advertises both
by default and prefers the certification key, because priority follows the
higher bit. Key handling is therefore the remaining suspect, but status 0x12
reports a network security state problem and does not identify which key was
selected. Installing a master key with `PROBE_TOUCHLINK_MASTER_KEY`, which makes
the probe advertise the master key alone, is untested.

Note also that the remote must be factory reset first. While it was commissioned
over Matter, the four press Touchlink sequence produced nothing.

## Network startup sequence

```
SKIP_STARTUP        -> start_top_level_commissioning(INITIALIZATION)
DEVICE_FIRST_START  -> if factory new: start(NETWORK_FORMATION)
                       else:           open_network()
FORMATION (ok)      -> open_network() + start(TOUCHLINK_TARGET)
```

Wait for network formation to complete before calling
`esp_zb_bdb_open_network()`. Calling it earlier logs "Device not in a network,
failed to open network" and leaves the network closed to joining.

## Build and run

The build requires ESP-IDF 5.4.x. The esp-zigbee-sdk components are pinned in
`probe/main/idf_component.yml`.

```sh
. $IDF_PATH/export.sh
cd probe
idf.py set-target esp32c6
idf.py -p /dev/ttyACM0 flash monitor
```

Adjust the LED pins at the top of `probe/main/main.c` for your board. If it has
no LED, remove the LED code and use Wi-Fi or serial logging.

## Related

- [ikea-bilresa-e2490](https://github.com/tdamsma/ikea-bilresa-e2490) documents the remote's Matter endpoints, events, sleep behaviour, and Zigbee mode.
- [esp32c6-matter-thread-controller](https://github.com/tdamsma/esp32c6-matter-thread-controller) provides the Matter controller firmware used for the BILRESA measurements.

## Licence

Apache-2.0.
