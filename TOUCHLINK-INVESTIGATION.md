# The Zigbee route with an IKEA BILRESA: procedure and result

A record of exactly what was done, in order, and what each step showed, for both
ways of reaching the remote over Zigbee. This document exists so the work is not
repeated blindly, and so anyone continuing knows precisely where the wall is.

## Summary

The remote has two Zigbee paths and neither delivers per channel control.

Plain Zigbee join works and delivers commands, but a coordinator cannot tell the
three channels apart, because the channel is only visible in the group address
of a groupcast and a joined remote does not use one.

Touchlink would fix that. It reaches network setup and then fails during secured
communication. The handshake completes on our side, the target joins, group
membership succeeds, and then every frame from the remote is rejected with bad
key sequence number. Explicit configuration of a production master key is the
obvious next thing to try and has not been tested.

## Goal

The remote sends its wheel and button commands as groupcasts to one fixed group
per channel, 21658, 21659 and 21660. A coordinator sees a flat command stream
with no channel information, because the channel is only visible in the APS
destination address. A Touchlink bound target would receive those groupcasts and
could recover the channel, giving three separately controllable channels locally
with no hub and with real dimming.

## Equipment and versions

- ESP32-C6 with 4 MB flash, built in ceramic antenna
- ESP-IDF v5.4.2, esp-zigbee-lib and esp-zboss-lib 1.6.x
- IKEA BILRESA E2490, firmware 1.8.7

## The other Zigbee path: plain join, verified working but channel blind

Entering plain Zigbee mode is factory reset, four rapid presses of the system
button, then eight more. The LEDs go dark, which is success. The remote then
joins an ordinary coordinator with permit join open, and no Touchlink is
involved. This was verified with this probe and produced usable traffic:

- wheel turn on Level cluster 0x0008, absolute values, one update per 101 ms
- wheel click on On/Off cluster 0x0006, alternating On and Off
- a Level value of zero immediately before every click, restored about 100 ms
  later, which will glitch any naive mapping of Level onto an output
- nothing at all from the button that selects the channel

The last point is the problem. All three channels produce the same stream, so a
coordinator cannot separate them. That is what motivated the Touchlink attempt
below.

## Step 1: coordinator as Touchlink target

Configuration was the probe's default: `ESP_ZB_DEVICE_TYPE_COORDINATOR`, network
formed on a channel from the mask, permit join open, and
`esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_TOUCHLINK_TARGET)`
with `esp_zb_zdo_touchlink_set_rssi_threshold(-90)`.

The stack reported target mode active:

```
signal 14 BDB Touchlink Target (ESP_OK)
```

The remote was factory reset, then the system button was pressed four times
rapidly with the remote held against the board, several times over several
minutes.

Result: nothing. The callback registered with
`esp_zb_touchlink_action_check_register()` never fired once, in any capture.

## Step 2: why the coordinator role is wrong

A remote that is not on a network does not join an existing network during
Touchlink. It starts a network of its own and adopts the target into it. A
coordinator that has already formed a network cannot be adopted, so it never
answers a scan request in a way that leads anywhere.

The target therefore has to be factory new and joinable. A `PROBE_ROLE_ROUTER`
option was added, which selects `ESP_ZB_DEVICE_TYPE_ROUTER` and, instead of
forming a network, starts Touchlink target mode directly and waits.

The stored network must be cleared when changing role, otherwise the stack
restores the previous coordinator network from NVS and ignores the change:

```sh
idf.py -p /dev/ttyACM0 erase-flash
idf.py -p /dev/ttyACM0 flash
```

This was also how an unrelated puzzle was explained, where a channel mask change
appeared to have no effect because the old network was being restored.

## Step 3: the remote must be factory reset

Observed separately: while the remote was commissioned over Matter and Thread,
the four press sequence produced nothing at a coordinator. After a factory reset
Touchlink behaved as described below. The remote holds one transport at a time,
so Zigbee experiments cost the Matter commissioning.

## Step 4: router role, handshake starts

With the router role the remote's scan is answered and the handshake proceeds:

```
router role: waiting as a touchlink target, do not expect a network of our own
signal 14 BDB Touchlink Target (ESP_OK)
*** TOUCHLINK request, action=0 -- allowing ***
signal 54 NWK Permit Join (ESP_OK)
signal 15 BDB Touchlink Network (ESP_OK)
signal 16 BDB Touchlink Target Finished (ESP_OK)
```

The ESP flashed its LED, so from our side this looked like success. The remote
kept blinking, which means its initiator never considered the exchange complete.

A network layer signal then repeated every seven seconds or so, and in bursts
when the wheel was turned. No ZCL command ever reached the application.

## Step 5: instrumenting the failure

Two additions were made.

The network status code was logged, using
`esp_zb_zdo_signal_nwk_status_indication_params_t`, which carries a status byte
and the originating address.

The endpoint was added to all three group ids on
`ESP_ZB_BDB_SIGNAL_TOUCHLINK_TARGET_FINISHED`, using
`esp_zb_aps_group_table_add_group()`, because a device silently ignores
groupcasts for groups it is not a member of.

Result:

```
*** TOUCHLINK request, action=1 -- allowing ***
signal 15 BDB Touchlink Network (ESP_OK)
group 21658 on endpoint 1: ESP_OK
group 21659 on endpoint 1: ESP_OK
group 21660 on endpoint 1: ESP_OK
NLME status 0x12 from 0x0001
```

Group membership succeeds. The status code decodes, via
`zboss_api_nwk.h`, as:

```
#define ZB_NWK_COMMAND_STATUS_BAD_KEY_SEQUENCE_NUMBER 0x12U
```

Address 0x0001 is the remote, which is the coordinator of the network it created
during Touchlink. So the remote is transmitting, we are receiving, and the
network layer rejects every frame because it is encrypted with a key sequence
number we do not have.

## Step 6: offering both Touchlink keys

Touchlink encrypts the transferred network key with one of two keys. The
certification key, index 15, is public and intended for testing. The master key,
index 4, is a secret shared by certified Touchlink devices. If the two sides do
not agree, the target joins with a network key it cannot use, which produces
exactly the observed symptom.

The probe was changed to offer both:

```c
esp_zb_zdo_touchlink_set_key_bitmask(ESP_ZB_TOUCHLINK_MASTER_KEY |
                                     ESP_ZB_TOUCHLINK_CERTIFICATION_KEY);
```

Rebuilt, erased, reflashed, and retested. The result was identical, including
the repeating `NLME status 0x12`.

This step proved nothing. The SDK header states that both keys are already in
the bitmask by default, so the call changed no behaviour, and priority follows
the higher bit, which means the certification key at bit 15 outranks the master
key at bit 4. The call also does not supply master key bytes, which the SDK does
not contain. Advertising the master key alone, with a key installed through
`esp_zb_zdo_touchlink_set_master_key()`, is a different configuration and is
still untested.

## Step 7: installing a master key and advertising it alone

`PROBE_TOUCHLINK_MASTER_KEY` was added, which installs a key with
`esp_zb_zdo_touchlink_set_master_key()` and then calls
`esp_zb_zdo_touchlink_set_key_bitmask(ESP_ZB_TOUCHLINK_MASTER_KEY)` so that only
the master key is advertised and the selected key is unambiguous.

Both sides were reset, the probe logged `master key installed, advertising
master key only`, and the procedure from step 4 was repeated.

The result was unchanged, including `NLME status 0x12` repeating.

This does not settle the question. The key value used was not verified against a
published source, and the log only demonstrates that the string parsed into 16
bytes and that the bitmask was applied. A wrong key and a correct key produce the
same symptom in this setup, because in both cases the target joins and then
rejects frames. The master key path therefore remains untested with a confirmed
key value.

## Step 8: inspecting the key we hold, and sweeping the sequence number

With the master key cleared and the SDK defaults restored, the probe was changed
to read the network key eight seconds after Touchlink using
`esp_zb_secur_primary_network_key_get()`, report whether it is degenerate, and
then apply sequence numbers 0, 1 and 2 with
`esp_zb_secur_network_key_switch()` at ten second intervals.

The key is plausible material, not zeros:

```
network key starts e1454514, switching to seq 0
key switch to seq 0: ESP_OK
```

This is the most useful single observation in the investigation. The Touchlink
key transfer produces a real looking key, which argues against the idea that the
key exchange itself is failing.

The sequence sweep was inconclusive. Errors stopped after the first switch, but
at the same time the remote resumed sending scan requests:

```
62972  NLME status 0x12
69656  NLME status 0x12
69854  key switch to seq 0
83978  TOUCHLINK request, action=0
92255  TOUCHLINK request, action=0
100502 TOUCHLINK request, action=0
```

The quiet period is explained by the remote no longer sending post commissioning
traffic, not necessarily by the key switch. The remote abandons the result and
restarts its search, which matches its LED continuing to blink throughout.

## Conclusion

Touchlink reaches network setup and fails during secured communication.
Commissioning completes on the target side, the device joins, and group
membership succeeds, after which every frame from the remote is rejected with
NWK status 0x12, bad key sequence number.

The roles, the channel, proximity, group membership and the receive path are all
ruled out, since each was changed or verified independently. Key handling is the
remaining candidate, and it is not proven. Status 0x12 reports a network
security state problem. It does not identify which key was selected, nor show
that particular key bytes are wrong.

Installing a master key and advertising it alone was exercised with an
unverified key value, so it discriminates nothing. The value could not be
confirmed against a published source.

The key the target ends up holding is plausible material rather than a
degenerate value, which argues against a failed key exchange.

The sequence number sweep was inconclusive, because the remote changed behaviour
at the same time.

The honest state is that Touchlink negotiates, the target joins with usable
looking key material, the initiator is never satisfied and resumes scanning, and
the few frames that do arrive after commissioning are rejected with status 0x12.
The cause is not identified.

Further progress needs an over the air capture with a second radio, so that the
negotiated key index, the network key transfer and the security headers of the
rejected frames can be read directly rather than inferred. Everything short of
that has been guesswork, and this document is a record of which guesses were
eliminated.

## What remains untested

Because no traffic ever reached the application, the group addressing path was
never exercised against real commands. The raw command handler that reads the
APS destination from `zb_zcl_parsed_hdr_t.addr_data.common_data.dst_addr` is
written and compiles, but whether the group id appears there for this remote's
groupcasts is unverified.

Anyone resuming with a working key should start there, turn the wheel on each of
the three channels, and check whether the `RAW` lines show 0x549a, 0x549b and
0x549c.

## Where to resume

The next experiment, in order:

1. Set `PROBE_TOUCHLINK_MASTER_KEY` to the production master key. The probe
   installs it and then advertises the master key only, so the selected key is
   unambiguous.
2. Keep `PROBE_ROLE_ROUTER` enabled, erase flash on the board and factory reset
   the remote, so neither side carries stale network state.
3. Repeat step 4 and watch two things: whether the remote's LED stops blinking,
   and whether `NLME status 0x12` disappears.
4. If 0x12 persists, capture the exchange over the air and check which key index
   was negotiated before drawing conclusions about the key bytes.
5. If commands start arriving, turn the wheel on each of the three channels and
   check whether the `RAW` lines show destination groups 0x549a, 0x549b and
   0x549c.

Note that steps 3 and 5 are separate results. Successful commissioning would not
by itself demonstrate that the three channels are distinguishable at the
application layer.

## The alternative that does work

Matter over Thread exposes the three channels as nine endpoints and needs no
secret keys. See
[ikea-bilresa-e2490](https://github.com/tdamsma/ikea-bilresa-e2490) and
[esp32c6-matter-thread-controller](https://github.com/tdamsma/esp32c6-matter-thread-controller).
