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

A remote bound through Touchlink sends groupcasts to a fixed group ID for each
channel. BILRESA uses 21658, 21659 and 21660. Recovering that ID is harder than
it looks, because the SDK does not hand it to the application anywhere.

The attribute-write callback provides `status`, `dst_endpoint` and `cluster`,
and omits addressing entirely. The raw command handler provides
`zb_zcl_parsed_hdr_t`, whose `addr_data.common_data.dst_addr` is 0xfffd for a
groupcast, since an APS groupcast travels inside a network layer broadcast. The
APS data indication provides `esp_zb_apsde_data_ind_t`, which reports
`dst_addr_mode=0x02` and `dst_short_addr=0xfffd` for these frames, the group ID
having already been resolved away.

What the parsed header does carry is the APS frame control, so a groupcast can
at least be told apart from a real broadcast. Bits 2 and 3 of `fc` are the
delivery mode, where 3 means group:

```c
static bool raw_command(uint8_t bufid)
{
    const zb_zcl_parsed_hdr_t *h = ZB_BUF_GET_PARAM(bufid, zb_zcl_parsed_hdr_t);
    uint8_t mode = (h->addr_data.common_data.fc >> 2) & 0x3;  /* 3 = group */
    return false;  /* not consumed -- normal processing continues */
}
esp_zb_raw_command_handler_register(raw_command);
```

To recover the channel itself, give each group its own endpoint, 21658 on
endpoint 1, 21659 on endpoint 2 and 21660 on endpoint 3. Group delivery is
resolved against the APS group table before the indication is raised, so
`dst_endpoint` identifies the channel. The probe logs this in an `APS` line as
`ep=2 chan=2 group=21659`.

Note that the remote sends its Groups commands to endpoint 1 whenever a channel
is bound, starting with Remove All Groups, which destroys this layout. The probe
re-asserts it 1.5 seconds after any Groups cluster traffic.

### Touchlink

The full procedure and evidence are in
[TOUCHLINK-INVESTIGATION.md](TOUCHLINK-INVESTIGATION.md).

Touchlink works against an IKEA BILRESA E2490 and gives per channel control
with no hub. Four things are needed, and the first two were what blocked it for
a long time.

**Router role.** A coordinator is the wrong role. While the probe formed its own
network as coordinator, the remote's Touchlink scan produced nothing at all, and
the callback registered with `esp_zb_touchlink_action_check_register()` never
fired. A remote that is off network does not join an existing network during
Touchlink. It creates one and adopts the target, which a coordinator cannot
accept. Keep `PROBE_ROLE_ROUTER` enabled.

**No automatic key sequence switching.** Every frame being rejected with

```
NLME status 0x12 from 0x0001    ZB_NWK_COMMAND_STATUS_BAD_KEY_SEQUENCE_NUMBER
```

was caused by the probe's own diagnostic. An alarm scheduled from
`TOUCHLINK_TARGET_FINISHED` swept the local key sequence through 0, 1 and 2,
which moved it away from the one the remote uses. `keyseq <n>` on the console is
now a one shot for deliberate use, and nothing switches the key on its own. The
switch persists across a reboot, so a board left in this state stays broken
until the sequence is put back.

**The ZLL master key, advertised alone.** `PROBE_TOUCHLINK_MASTER_KEY` set to
`9F55************************EE31` makes the probe advertise the master key,
index 4, on its own, so the selected key index is unambiguous. Commissioning
completes on this path, though it has not been shown that the default, which
advertises both keys, would fail.

**One endpoint per group**, as described above, which is what makes the channel
readable.

The remote must be factory reset first. While it was commissioned over Matter,
the four press Touchlink sequence produced nothing. A reset remote can only bind
channel 1; channel 2 becomes selectable once channel 1 is bound, and channel 3
once channel 2 is.

### Serial console

The probe carries a line based console on the USB-Serial-JTAG port, so the light
behaviour and the Zigbee state can be changed at runtime without reflashing,
which matters because reflashing is cheap but re-pairing is not. Type `help` for
the list. It reads through the `usb_serial_jtag` driver rather than `stdin`,
since the blocking ROM path is what drops out under ZBOSS load.

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
