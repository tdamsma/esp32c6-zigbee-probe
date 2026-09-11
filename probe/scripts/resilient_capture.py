"""Capture serial output across the drop-outs the Zigbee stack causes.

The ZBOSS stack destabilises the C6's USB-Serial-JTAG, so reads fail with
"Device not configured" a few seconds after boot. Reopen and continue instead
of exiting, so the log covers whatever the port is alive for.
"""
import argparse
import glob
import time
import serial

p = argparse.ArgumentParser()
p.add_argument('--seconds', type=float, default=240)
p.add_argument('--reset', action='store_true')
a = p.parse_args()

end = time.time() + a.seconds
first = True
while time.time() < end:
    ports = sorted(glob.glob('/dev/cu.usbmodem*'))
    if not ports:
        time.sleep(0.5)
        continue
    try:
        s = serial.Serial(ports[0], 115200, timeout=.05)
        if first and a.reset:
            s.setDTR(False); s.setRTS(True); time.sleep(.1)
            s.setRTS(False); time.sleep(.1)
        first = False
        while time.time() < end:
            data = s.read(4096)
            if data:
                print(data.decode(errors='replace'), end='', flush=True)
    except (serial.SerialException, OSError):
        try:
            s.close()
        except Exception:
            pass
        time.sleep(0.5)
