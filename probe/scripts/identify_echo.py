"""Send Identify to the remote while it is awake, over Zigbee.

The BILRESA is a sleepy end device: a unicast sent while it is idle is held
for indirect delivery and expires with NLME status 0x06. It is awake around
its own traffic, so Identify has to answer a frame it just sent. The Matter
equivalent is bilresa_identify_echo.py.

Question being tested: can a Zigbee peer drive the remote's three LEDs? Over
Matter it cannot, because Identify accepts command 0 only and IdentifyType
reads None.

Reading and writing are split across threads, and the whole run is capped by
SIGALRM, because the ZBOSS stack destabilises the USB-Serial-JTAG port and a
read on a port that has gone away blocks forever. An earlier version of this
script hung that way and silently lost the experiment.
"""
import argparse
import glob
import os
import re
import signal
import sys
import threading
import time

import serial

p = argparse.ArgumentParser()
p.add_argument('--seconds', type=float, default=60)
p.add_argument('--identify-seconds', type=int, default=5)
p.add_argument('--effect', action='store_true',
               help='send Trigger Effect instead of plain Identify')
p.add_argument('--endpoint', type=int, default=1)
p.add_argument('--gap', type=float, default=2.0)
p.add_argument('--log', default='tmp/identify-echo.log')
a = p.parse_args()

CMD = (f'effect 0 0 {a.endpoint}' if a.effect
       else f'identify {a.identify_seconds} {a.endpoint}')
WAKE = re.compile(r'APS\s+src=0x0001')

os.makedirs(os.path.dirname(a.log) or '.', exist_ok=True)
log = open(a.log, 'w')
lock = threading.Lock()
state = {'awake': 0.0, 'port': None, 'sent': 0, 'stop': False}


def emit(line):
    with lock:
        log.write(line + '\n')
        log.flush()
    print(line, flush=True)


def reader():
    """Owns nothing but the read side. If it wedges, the clock still runs."""
    buf = ''
    while not state['stop']:
        s = state['port']
        if s is None:
            time.sleep(0.2)
            continue
        try:
            data = s.read(4096)
        except Exception:
            time.sleep(0.2)
            continue
        if not data:
            continue
        buf += data.decode(errors='replace')
        while '\n' in buf:
            line, buf = buf.split('\n', 1)
            line = line.rstrip('\r')
            emit(line)
            if WAKE.search(line):
                state['awake'] = time.time()


# Hard cap. The run is short and a hang is the one failure that costs the
# operator's time rather than just the data.
signal.signal(signal.SIGALRM, lambda *_: os._exit(3))
signal.alarm(int(a.seconds) + 20)

ports = sorted(glob.glob('/dev/cu.usbmodem*'))
if not ports:
    sys.exit('no serial port found')
port = serial.Serial(ports[0], 115200, timeout=0.2)
state['port'] = port
threading.Thread(target=reader, daemon=True).start()
port.write(b'rxlog on\n')

emit(f'=== {CMD} every {a.gap}s for {a.seconds}s on {ports[0]} ===')
end = time.time() + a.seconds
last = 0.0
while time.time() < end:
    now = time.time()
    if now - last >= a.gap:
        last = now
        # Send regardless of whether a wake was parsed: a continuously turned
        # wheel keeps the remote awake, and relying on the parse is what makes
        # this fragile. The awake flag is reported so a miss is visible.
        fresh = now - state['awake'] < 3.0
        try:
            port.write((CMD + '\n').encode())
            port.flush()
        except Exception as e:
            emit(f'!!! write failed: {e}')
            break
        state['sent'] += 1
        emit(f'>>> sent {CMD} (#{state["sent"]}, remote '
             f'{"awake" if fresh else "not seen recently"})')
    time.sleep(0.05)

state['stop'] = True
emit(f'=== sent {state["sent"]} commands ===')
log.close()
