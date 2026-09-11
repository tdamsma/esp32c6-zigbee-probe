"""Two-way console for the Zigbee probe's light control.

The probe's serial port drops out under the ZBOSS stack, so reconnect the way
resilient_capture.py does, and re-send nothing: commands are typed again by
whoever is driving. Use --cmd for scripted one-shots, or no --cmd for an
interactive session where stdin lines are forwarded to the board.
"""
import argparse
import glob
import queue
import sys
import threading
import time

import serial

p = argparse.ArgumentParser()
p.add_argument('--seconds', type=float, default=20)
p.add_argument('--cmd', action='append', default=[],
               help='command to send, repeatable')
p.add_argument('--delay', type=float, default=0.4,
               help='seconds between queued commands')
p.add_argument('--reset', action='store_true')
p.add_argument('--quiet', action='store_true',
               help='only print lines from the light/console, not the stack')
a = p.parse_args()

outbox = queue.Queue()
for c in a.cmd:
    outbox.put(c)


def stdin_pump():
    for line in sys.stdin:
        outbox.put(line.rstrip('\n'))


if not a.cmd:
    threading.Thread(target=stdin_pump, daemon=True).start()

KEEP = ('LIGHT', 'APS ', 'light ', 'map ', 'chan ', 'console', 'commands:',
        'unknown:', 'saved', 'bad ', '  ')

end = time.time() + a.seconds
first = True
next_send = 0.0
buf = ''
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
                buf += data.decode(errors='replace')
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.rstrip('\r')
                    if not a.quiet or any(k in line for k in KEEP):
                        print(line, flush=True)
            now = time.time()
            if now >= next_send and not outbox.empty():
                cmd = outbox.get()
                s.write((cmd + '\n').encode())
                s.flush()
                print(f'> {cmd}', flush=True)
                next_send = now + a.delay
    except (serial.SerialException, OSError) as e:
        try:
            s.close()
        except Exception:
            pass
        time.sleep(0.5)
