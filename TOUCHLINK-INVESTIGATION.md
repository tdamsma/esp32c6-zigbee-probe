# The Zigbee route with an IKEA BILRESA: procedure and result

A record of exactly what was done, in order, and what each step showed, for both
ways of reaching the remote over Zigbee. This document exists so the work is not
repeated blindly, and so anyone continuing knows precisely where the wall is.

## Summary

Touchlink works, and the three channels are separately identifiable. This was
achieved on 2026-09-10 and is described in step 10. Steps 1 to 9 below are the
earlier record, kept because they show which explanations were eliminated and
because two of the conclusions they reached turned out to be wrong.

The working configuration is a router role target, the ZLL master key
advertised alone, no automatic key sequence switching, and one endpoint per
BILRESA group. With that in place a factory new pairing produces no rejected
frames at all, and a wheel command on channel 1, 2 or 3 arrives on endpoint 1,
2 or 3 respectively.

Two findings from the earlier steps were mistaken. The key sequence sweep was
not a neutral diagnostic, it was the cause of the rejections. The conclusion
that the remote broadcasts rather than groupcasts came from reading only the
network layer destination, which is 0xfffd for both.

Plain Zigbee join is still channel blind and that has not changed. The channel
is visible only in the group address of a groupcast, and a joined remote does
not use one.

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

Confirmed upstream after the fact:
[esp-zigbee-sdk#781](https://github.com/espressif/esp-zigbee-sdk/issues/781)
records the same coordinator-role failure on ESP-IDF v5.5.2, and Espressif
maintainer xieqinan confirms that Touchlink only works on distributed networks,
pointing at the SDK's distributed-network Touchlink example. So this is by
design, not a bug that a later SDK will fix.

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

## Step 9: a clean run from a genuinely factory new board

Steps 4 to 8 all shared a defect that was only found afterwards. The board was
reset between attempts but its flash was never erased, so it carried network
state from the previous Touchlink into the next one. Instrumenting the network
identity at boot made this visible:

```
run A  before touchlink: pan 0x5609 ext_pan 9c139efffecc0afc channel 15 short 0x0002
run B  before touchlink: pan 0x4f65 ext_pan 9c139efffecc0afc channel 15 short 0x0002
```

A target that is not factory new takes a different Touchlink path than a fresh
one, so those runs are weaker evidence than they looked. In run B the target
signal returned `ESP_FAIL` and `ESP_ZB_BDB_SIGNAL_TOUCHLINK_TARGET_FINISHED`
never fired at all, which means the group registrations and the key sequence
sweep of step 8 never executed in that run.

The run was repeated after `idf.py erase-flash` and a reflash, with the remote
factory reset as well. The baseline was then genuinely factory new:

```
before touchlink: pan 0xffff ext_pan 0000000000000000 channel 255 short 0xfffe
```

From that state Touchlink completed properly:

```
TOUCHLINK request, action=0 -- allowing
touchlink network: pan 0xbbef ext_pan 9c139efffecc0afc channel 20 short 0x0002
target finished:   pan 0xbbef ext_pan 9c139efffecc0afc channel 20 short 0x0002
group 21658 on endpoint 1: ESP_OK
group 21659 on endpoint 1: ESP_OK
group 21660 on endpoint 1: ESP_OK
```

Three things follow from those identity lines.

The channel is 20, while the probe's own mask is channels 25 and 26. The remote
chose it, and it differs from the 15 seen in the earlier runs, so nothing was
cached from a previous attempt.

Our short address is 0x0002 and the rejected frames come from 0x0001, which is
the ordinary initiator and target pair. The two sides are on one network with
sensible addresses. An earlier hypothesis, that Touchlink had left the probe on
a network of its own making with a separate key, is therefore wrong.

The extended PAN id equals the board's own EUI-64 even from a clean erase. That
is ZBOSS using its own address when asked to start the network, not evidence of
a second network.

The key material was fresh, different from the earlier run, and again not
degenerate. The full key was read back on a later boot, since flashing the
application without erasing leaves NVS intact:

```
before touchlink: network key 008cf15f...
```

The sequence sweep then ran to completion, which it had not done before:

```
network key starts 008cf15f, switching to seq 0
key switch to seq 0: ESP_OK
network key starts 008cf15f, switching to seq 1
key switch to seq 1: ESP_OK
network key starts 008cf15f, switching to seq 2
key switch to seq 2: ESP_OK
```

Throughout the run the remote's buttons were pressed repeatedly, so there was
real traffic to accept or reject at every setting. The result was 80 status
indications, every one of them 0x12 from 0x0001, no other status value, and not
a single frame ever decoded to a ZCL command.

This makes the sequence number sweep a clean negative rather than the
inconclusive result recorded in step 8. All three plausible sequence numbers
were applied, each switch was accepted by the stack, and the rejections
continued unchanged.

## Step 10: the working configuration

Four changes were needed. Each is independent, and the first two had to be in
place before the last two could be observed at all.

### 10.1 The key sequence sweep was the cause, not a diagnostic

Step 8 added `try_key_sequence()`, scheduled from
`ESP_ZB_BDB_SIGNAL_TOUCHLINK_TARGET_FINISHED` and chaining through sequence
numbers 0, 1 and 2 at ten second intervals. It was intended to test whether the
sequence number was the problem. It was the problem. The alarm re-armed on
every pairing and switched the local network key sequence away from the one the
remote uses, after which every frame was rejected with status 0x12.

The evidence is unambiguous. In one run the last lines before traffic stopped
were:

```
W (64962) after touchlink: pan 0xf2aa ext_pan 9c139efffecc0afc channel 25 short 0x0002
W (64963) network key starts 3f6f9c1e, switching to seq 2
W (64967) key switch to seq 2: ESP_OK
```

Reception stopped at that instant and did not return across a reboot, because
the switch persists. Issuing `keyseq 0` from the console restored it
immediately. Note that `esp_zb_secur_network_key_switch()` is a local
operation, so it succeeds regardless of what the remote is using.

The sweep is removed. A one shot `keyseq <n>` remains available from the
console for deliberate use.

### 10.2 The ZLL master key, advertised alone

`PROBE_TOUCHLINK_MASTER_KEY` is set to the ZLL Master Key `9F55************************EE31`,
posted by [MayaZigBee on 2015-03-22](https://xcancel.com/MayaZigBee/status/579723961661022209)
and [discussed on Hacker News](https://news.ycombinator.com/item?id=9249753)
the next day, where it was later redacted after a DMCA takedown request. It has
been republished continuously since and is built into ZigBee penetration
testing tools. `install_master_key()` installs it and calls
`esp_zb_zdo_touchlink_set_key_bitmask(ESP_ZB_TOUCHLINK_MASTER_KEY)` so the
master key is the only one advertised and the selected index is unambiguous.

Commissioning completes with this key, which answers the question step 7 could
not: the remote accepts the master key path.

Whether the key was required was left open here, because the SDK default was
never retried once the key sequence sweep of 10.1 was found to be the real
cause of the rejections. Step 11 closes it: the default fails.

### 10.3 A groupcast arrives with a network destination of 0xfffd

Steps up to here concluded that the remote broadcasts its commands, because the
destination read from `zb_zcl_parsed_hdr_t.addr_data.common_data.dst_addr` was
always 0xfffd rather than a group id. That conclusion was wrong.

An APS groupcast is carried inside a network layer broadcast, so 0xfffd is the
expected network destination for a groupcast and does not distinguish one from
a true broadcast. The delivery mode is in the APS frame control, bits 2 and 3
of `addr_data.common_data.fc`, where 0 is unicast, 2 is broadcast and 3 is
group. Every wheel command from a Touchlink bound BILRESA reads as mode 3:

```
RAW src=0x0001 dst=0xfffd group (fc=0x0c) ep 1->1 cluster=0x0008 cmd=0x00
```

The remote had been groupcasting the whole time.

### 10.4 The SDK does not expose the group id, so use one endpoint per group

Knowing the frame is group delivered is not the same as knowing which group.
`esp_zb_aps_data_indication_handler_register()` looked like the answer, since
`esp_zb_apsde_data_ind_t` carries `dst_addr_mode` and `dst_short_addr`. It is
not: for these frames the callback reports `dst_addr_mode=0x02`, a plain 16 bit
address, and `dst_short_addr=0xfffd`. The group id has already been discarded.

The field that does survive is `dst_endpoint`, because group delivery is
resolved against the APS group table before the indication is raised. Giving
each group its own endpoint therefore restores the channel:

| Channel | Group | Endpoint |
|---|---|---|
| 1 | 21658 | 1 |
| 2 | 21659 | 2 |
| 3 | 21660 | 3 |

Endpoints 2 and 3 are created with the same On/Off and Level server clusters as
endpoint 1, using `esp_zb_on_off_light_clusters_create()` and
`esp_zb_ep_list_add_ep()`. A groupcast to 21659 is then delivered to endpoint 2
and nowhere else.

### 10.5 The remote rewrites endpoint 1 on every bind

Binding a channel sends three unicast commands to endpoint 1: Groups Remove All
Groups (0x0004 command 0x04), Groups Add Group (0x0004 command 0x00), and
Identify Trigger Effect (0x0003 command 0x40).

Remove All Groups clears endpoint 1's membership, and Add Group then puts the
newly bound channel's group on endpoint 1. Both break the layout in 10.4:
channel 1 stops being received, and the newly bound channel is delivered to two
endpoints at once. `regroup_now()` restores one group per endpoint, and is
scheduled 1.5 seconds after any frame on the Groups cluster.

### 10.6 Result

With all three channels bound, wheel commands attribute correctly and no frame
is rejected:

```
(72077) APS src=0x0001 dst=0xfffd mode=0x02 ep=1 chan=1 group=21658 cluster=0x0008
(74141) APS src=0x0001 dst=0xfffd mode=0x02 ep=2 chan=2 group=21659 cluster=0x0008
(184667) APS src=0x0001 dst=0xfffd mode=0x02 ep=3 chan=3 group=21660 cluster=0x0008
```

Across that session there were 3 frames on channel 1, 13 on channel 2 and 5 on
channel 3, following the operator's channel switches in order, with zero
occurrences of NLME status 0x12.

Two smaller observations from the same work. The group follows the channel
currently selected on the remote, not the channel that was bound, so a selected
but unbound channel still groupcasts and is still received, because the target's
own group membership is what determines delivery. And a single click sends an
alternating On/Off command rather than Toggle, so a click that repeats the value
already held raises no attribute change: take the click from the command in the
raw handler, not from `ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID`.

## Conclusion

Touchlink delivers per channel control from a BILRESA E2490 with no hub. The
wall described in steps 1 to 9 was self inflicted: the key sequence sweep added
to diagnose the rejections was generating them.

The channel identity is recoverable, but not from the field the earlier steps
looked at. The group id is absent from both the parsed ZCL header and the APS
data indication. Mapping each group to its own endpoint and reading
`dst_endpoint` is what makes it observable.

The over the air capture with a second radio, recommended at the end of step 9,
was never needed.

## How to reproduce

1. Set `PROBE_TOUCHLINK_MASTER_KEY` to the ZLL Master Key `9F55************************EE31`, leaked by MayaZigBee in 2015, and
   keep `PROBE_ROLE_ROUTER` enabled.
2. `idf.py erase-flash` then `idf.py flash`. Resetting the board is not enough.
3. Factory reset the remote by holding the pair button for about ten seconds,
   releasing when the amber pulse starts.
4. Confirm the baseline boot line reads
   `pan 0xffff ext_pan 0000000000000000 channel 255 short 0xfffe`. If it does
   not, the run is invalid.
5. Four rapid presses of the pair button with the remote held against the board
   binds channel 1. The front bottom button then unlocks channel 2, and channel
   3 after that. A reset remote cannot select channel 2 until channel 1 is
   bound.
6. Turn the wheel on each channel and check the `APS` lines for
   `ep=1 chan=1 group=21658`, `ep=2 chan=2 group=21659` and
   `ep=3 chan=3 group=21660`.

The serial console added for this work drives all of it at runtime, so changing
the gesture mapping does not require a rebuild. `help` lists the commands.

## Step 11: the master key is required, tested 2026-09-11

Step 10.2 installed the ZLL master key and noted that this did not show the key
was needed, because the SDK default was never retried once the key sequence
sweep was found to be the real cause of the earlier rejections. That comparison
has now been made, with the sweep gone so it cannot confound the result.

`PROBE_TOUCHLINK_MASTER_KEY` empty, which leaves the SDK defaults advertising
both the certification key (index 15) and the master key (index 4) and
preferring the certification key because it has the higher bit. Otherwise the
procedure above, unchanged: erase-flash, a factory reset remote, and a verified
`pan 0xffff ext_pan 0000000000000000 channel 255 short 0xfffe` baseline.

Commissioning completes:

```
W (93068) *** TOUCHLINK request, action=0 -- allowing ***
W (96368) target finished: pan 0x29df ext_pan 9c139efffecc0afc channel 11 short 0x0002
W (96376) regroup: 21658 on ep 1: ESP_OK
```

and then nothing works:

```
W (97561)  NLME status 0x12 from 0x0001
W (104483) NLME status 0x12 from 0x0001
W (110766) NLME status 0x12 from 0x0001
...
```

Nine rejections over two minutes, zero APS frames, zero
attribute writes, no wheel command of any kind, and the remote's LEDs kept
blinking because its initiator never saw the exchange complete. That is the
step 7 failure reproduced exactly.

Restoring the key and repeating the same procedure on the same hardware
minutes later pairs and delivers traffic. The two runs differ in the key alone.

So the answer is that the master key is required, and the caveat attached to
10.2 is resolved. `c22114def...`, a second key that circulated from the same account in 2016,
was not tested either, because
the question it was being held in reserve for is now answered.

Note what this does not say. It does not say the certification key is rejected
outright, only that the SDK default, which advertises both and prefers the
certification key, fails against this remote. Advertising the certification key
alone was not tested.

## The alternative that does work

Matter over Thread exposes the three channels as nine endpoints and needs no
secret keys. See
[ikea-bilresa-e2490](https://github.com/tdamsma/ikea-bilresa-e2490) and
[esp32c6-matter-thread-controller](https://github.com/tdamsma/esp32c6-matter-thread-controller).
