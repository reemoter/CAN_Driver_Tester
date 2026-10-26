"""EPOS UDP bench console. Python standard library only; no automatic motion."""
import argparse
import csv
from collections import deque
from pathlib import Path
import queue
import secrets
import socket
import struct
import threading
import time

STATES = ['OFFLINE', 'MONITOR', 'ENABLING', 'ENABLED', 'STOPPING', 'STOPPED', 'ERROR', 'RESETTING']
ERRORS = ['NONE', 'SDO_TIMEOUT', 'SDO_ABORT', 'PROTOCOL', 'STATE_TIMEOUT', 'DRIVE',
          'HEARTBEAT', 'CAN', 'RX_OVERFLOW', 'LEASE', 'TX', 'PDO_TIMEOUT', 'REVERSAL_TIMEOUT']
RESULTS = ['OK', 'BAD_PACKET', 'BAD_COMMAND', 'BAD_INDEX', 'STATE_OR_BUSY',
           'VELOCITY_RANGE', 'RESERVED', 'STALE_SEQUENCE', 'OTHER_CONTROLLER']


def encode(sequence, command, index=0, value=0):
    return struct.pack('<4sIBBHi', b'ABS1', sequence, command, index, 0, value)


def direction_velocity(magnitude, direction):
    speed = int(magnitude)
    if speed < 0 or direction not in ('Forward (+)', 'Reverse (-)'):
        raise ValueError('Enter a nonnegative speed and select Forward (+) or Reverse (-).')
    value = -speed if direction == 'Reverse (-)' else speed
    if not -(2**31) <= value <= 2**31 - 1:
        raise ValueError('The selected signed velocity must fit 32 bits.')
    return value


def decode(data, sequence, command, index):
    if len(data) not in (56, 128, 168) or data[:4] != b'ABR1':
        raise ValueError('Invalid reply format')
    if struct.unpack_from('<I', data, 4)[0] != sequence or data[8:10] != bytes((command, index)):
        return None  # late reply from another request
    statusword, emcy, target, actual, abort, rx, tx, esr, uptime, lease = struct.unpack_from('<HHiiIIIIII', data, 16)
    result = dict(result=data[10], state=data[11], error=data[12], node=data[13],
                heartbeat=data[14], mode=struct.unpack_from('b', data, 15)[0],
                statusword=statusword, emcy=emcy, target=target, actual=actual,
                abort=abort, rx=rx, tx=tx, esr=esr, uptime=uptime, lease=lease,
                count=data[52], board_id=data[53], board_raw=data[54], board_flags=data[55])
    if len(data) >= 128:
        fields = struct.unpack_from('<iiHhiiiiIIIIHBBIIIIII', data, 56)
        names = ('drive_target', 'demand', 'controlword', 'torque', 'current_demand',
                 'current_actual', 'position', 'position_delta', 'valid', 'unsupported',
                 'position_ms', 'diag_abort', 'diag_object', 'diag_sub', 'reserved',
                 'current_ms', 'torque_ms', 'motor_ms', 'demand_ms', 'controlword_ms', 'drive_target_ms')
        result.update(zip(names, fields))
        result['reversal_supported'] = bool(data[103] & 0x80)
        result['reversal_pending'] = bool(data[103] & 1)
    if len(data) == 168:
        result.update(zip(('hall', 'pdo_ready', 'hall_ms', 'hall_edges', 'hall_skipped',
                           'hall_invalid', 'encoder_ppr', 'pole_pairs', 'position_unit', 'statusword_ms', 'pdo_rx'),
                          struct.unpack_from('<HHIIIIIIIII', data, 128)))
    return result


class Client:
    def __init__(self, host, port=2000, bind='0.0.0.0'):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind, 0))
        self.sock.connect((host, port))
        self.sequence = secrets.randbelow(0x7fffffff)

    def request(self, command, index=0, value=0):
        self.sequence = (self.sequence + 1) & 0xffffffff
        self.sock.send(encode(self.sequence, command, index, value))
        deadline = time.monotonic() + 0.2
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('UDP response timeout')
            self.sock.settimeout(remaining)
            reply = decode(self.sock.recv(2048), self.sequence, command, index)
            if reply is not None:
                return reply

    def close(self):
        self.sock.close()


def label(items, value):
    return items[value] if value < len(items) else str(value)


def validate_hand_sample(sample):
    if sample.get('state') != 1 or sample.get('statusword', 0) & 0x6f != 0x40:
        raise ValueError('Reset drivers, wait for MONITOR / Switch on disabled. Do not Enable.')
    if sample.get('valid', 0) & 0x201 != 0x201 or sample.get('hall') not in range(1, 7):
        raise ValueError('Fresh position and valid Hall pattern are required. Update MCU firmware if absent.')
    for stamp in ('hall_ms', 'position_ms', 'statusword_ms'):
        if (sample['motor_ms'] - sample.get(stamp, 0)) & 0xffffffff >= 200:
            raise ValueError('Telemetry is stale; wait for fresh disabled-state readings.')


def hand_turn_report(start, end):
    validate_hand_sample(start)
    validate_hand_sample(end)
    if start['node'] != end['node'] or not 0 < ((end['motor_ms'] - start['motor_ms']) & 0xffffffff) < 0x80000000:
        raise ValueError('Node/reset/time discontinuity; start measurement again.')
    if end['hall_edges'] < start['hall_edges']:
        raise ValueError('Counters reset; start measurement again.')
    delta = (end['position'] - start['position'] + 2**31) % 2**32 - 2**31
    edges = end['hall_edges'] - start['hall_edges']
    skipped = end['hall_skipped'] - start['hall_skipped']
    invalid = end['hall_invalid'] - start['hall_invalid']
    text = f'ONE physical turn (user measured): position delta={delta}; Hall transitions={edges}; skipped={skipped}; invalid={invalid}. '
    if skipped or invalid or edges == 0 or edges % 6:
        return text + 'Measurement inconclusive. Turn more slowly in one direction; do not change settings.'
    text += f'Pole-pair candidate={edges // 6} (configured {end["pole_pairs"]}). '
    if end.get('position_unit') == 0x00b50000 and end.get('valid', 0) & 4096:
        text += f'Encoder PPR candidate={abs(delta) / 4:g} (configured {end["encoder_ppr"]}); assumes motor-shaft incremental feedback. '
    else:
        text += 'Position scaling unconfirmed; PPR cannot be calculated. '
    return text + 'Candidate only: a precise full turn, no reversals/missed edges required. No settings were written.'


def gui(args):
    import tkinter as tk
    from tkinter import ttk
    root = tk.Tk()
    root.title('ABS UDP motor bench')
    root.geometry('1100x800')
    commands, events = queue.Queue(), queue.Queue()
    stop = threading.Event()
    samples = deque(maxlen=600)  # approximately the most recent 60 seconds
    measurement_start = None
    turn_phase = 0
    host = tk.StringVar(value=args.host)
    index = tk.StringVar(value='0')
    velocity = tk.StringVar(value='0')
    direction = tk.StringVar(value='Forward (+)')
    status = tk.StringVar(value='Disconnected. Connect performs status reads only.')
    settings = ttk.Frame(root, padding=10)
    settings.pack(fill='x')
    ttk.Label(settings, text='MCU IP').pack(side='left')
    ttk.Entry(settings, textvariable=host, width=18).pack(side='left', padx=5)
    ttk.Label(settings, text='Motor index (0/1)').pack(side='left')
    ttk.Entry(settings, textvariable=index, width=4).pack(side='left', padx=5)
    ttk.Label(settings, text='Speed magnitude (EPOS units / rpm)').pack(side='left')
    ttk.Entry(settings, textvariable=velocity, width=7).pack(side='left', padx=5)

    def submit(command, zero=False):
        try:
            i = int(index.get())
            v = direction_velocity(velocity.get(), direction.get()) if command == 2 and not zero else 0
            if i not in (0, 1) or not -(2**31) <= v <= 2**31 - 1:
                raise ValueError('Use motor index 0/1 and a valid 32-bit velocity.')
            commands.put((command, i, v))
        except ValueError as exc:
            status.set(str(exc))

    directions = ttk.Frame(root, padding=(10, 0))
    directions.pack(fill='x')
    ttk.Label(directions, text='Direction').pack(side='left')
    ttk.Combobox(directions, textvariable=direction, state='readonly', width=16,
                 values=('Forward (+)', 'Reverse (-)')).pack(side='left', padx=5)
    ttk.Button(directions, text='Set zero speed', command=lambda: submit(2, zero=True)).pack(side='left', padx=5)
    ttk.Label(directions, text='Select direction, then Set velocity. Reversal waits for standstill in MCU.').pack(side='left')

    buttons = ttk.Frame(root, padding=10)
    buttons.pack(fill='x')
    ttk.Button(buttons, text='Connect / status', command=lambda: commands.put(('connect', host.get(), 0))).pack(side='left')
    for text, cmd in [('Power ON', 4), ('Enable at zero', 1), ('Set velocity', 2),
                      ('QUICK STOP', 3), ('Power OFF', 5), ('Reset drivers', 7)]:
        ttk.Button(buttons, text=text, command=lambda c=cmd: submit(c)).pack(side='left', padx=2)
    ttk.Label(root, text='Enable starts automatic 100 ms keepalive. Link loss stops renewal; no automatic re-enable.',
              wraplength=730).pack(anchor='w', padx=10)
    ttk.Label(root, text='Velocity uses device units. Actual velocity is only sampled while ENABLED; other states may show stale data.',
              wraplength=730).pack(anchor='w', padx=10)
    ttk.Label(root, text='Reset drivers cycles the shared EPOS power (all motors). Stop first; re-enable manually after MONITOR.',
              wraplength=890).pack(anchor='w', padx=10)
    ttk.Label(root, textvariable=status, wraplength=730).pack(anchor='w', padx=10, pady=8)
    feedback = tk.StringVar(value='Feedback: waiting for extended diagnostics.')
    ttk.Label(root, textvariable=feedback, wraplength=990, justify='left').pack(anchor='w', padx=10, pady=8)

    def save_samples():
        if not samples:
            events.put(('log', 'CSV not saved: no extended diagnostic samples. Load the updated MCU firmware, then Connect / status.'))
            return
        try:
            folder = Path(__file__).resolve().parent / 'logs'
            folder.mkdir(parents=True, exist_ok=True)
            path = folder / f'abs_motor_{time.strftime("%Y%m%d_%H%M%S")}_{time.time_ns() % 1000000000:09d}.csv'
            with open(path, 'x', newline='', encoding='utf-8-sig') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(samples[0]))
                writer.writeheader()
                writer.writerows(list(samples))
            events.put(('log', f'Diagnostic samples saved: {path}'))
        except OSError as exc:
            events.put(('log', f'CSV save failed: {exc}'))

    ttk.Button(root, text='Save CSV to tools/logs (last ~60 s)', command=save_samples).pack(anchor='w', padx=10)
    def mark_turn(begin):
        nonlocal measurement_start, turn_phase
        try:
            if not samples:
                raise ValueError('Connect and wait for passive samples first.')
            sample = dict(samples[-1])
            validate_hand_sample(sample)
            if begin:
                measurement_start, turn_phase = sample, 1
                events.put(('log', 'Hand-turn START. Output disabled. Turn motor shaft exactly ONCE, slowly, one direction; do not Enable.'))
            else:
                if measurement_start is None:
                    raise ValueError('Click Begin one turn first.')
                events.put(('log', hand_turn_report(measurement_start, sample)))
                turn_phase = 2
        except ValueError as exc:
            events.put(('log', str(exc)))
    turn_buttons = ttk.Frame(root)
    turn_buttons.pack(anchor='w', padx=10)
    ttk.Button(turn_buttons, text='Begin one hand turn (disabled)', command=lambda: mark_turn(True)).pack(side='left')
    ttk.Button(turn_buttons, text='End one hand turn', command=lambda: mark_turn(False)).pack(side='left')
    output = tk.Text(root, height=19, state='disabled')
    output.pack(fill='both', expand=True, padx=10, pady=5)

    def worker():
        client, armed, selected = None, None, 0
        next_poll = 0
        while not stop.is_set():
            try:
                cmd, i, v = commands.get(timeout=0.02)
            except queue.Empty:
                cmd = None
            try:
                if cmd == 'connect':
                    if armed is not None:
                        events.put(('log', 'Stop the enabled motor before reconnecting.'))
                        continue
                    if client:
                        client.close()
                    client = None
                    client = Client(i, args.port, args.bind)
                    events.put(('log', 'Connected socket; querying MCU status (no motion).'))
                elif cmd is not None:
                    if not client:
                        events.put(('log', 'Connect first.'))
                        continue
                    if armed is not None and i != armed:
                        events.put(('log', 'Stop current motor before selecting another.'))
                        continue
                    selected = i
                    if cmd == 3:
                        armed = None
                    reply = client.request(cmd, i, v if cmd == 2 else 0)
                    detail = f' | requested velocity={v}' if cmd == 2 else ''
                    events.put(('log', f'Command {cmd}: {label(RESULTS, reply["result"])}{detail}'))
                    events.put(('status', reply))
                    if cmd == 1 and reply['result'] == 0:
                        armed = i
                    if cmd == 7 and reply['result'] == 0:
                        armed = None
                        events.put(('log', 'Shared driver reset accepted. Wait for MONITOR; no automatic enable.'))
                if client and time.monotonic() >= next_poll:
                    next_poll = time.monotonic() + 0.1
                    if armed is not None:
                        reply = client.request(6, armed)
                        if reply['result'] or reply['state'] not in (2, 3):
                            armed = None
                            events.put(('log', 'Keepalive stopped: motor not enabled/enabling.'))
                    reply = client.request(9, selected)
                    events.put(('status', reply))
            except (OSError, ValueError) as exc:
                previous, armed = armed, None
                events.put(('log', f'{exc}; keepalive OFF. Check target state before further commands.'))
                if client and previous is not None:
                    try:
                        client.request(3, previous)
                    except (OSError, ValueError):
                        pass
                next_poll = time.monotonic() + 1
        if client:
            if armed is not None:
                try:
                    client.request(3, armed)
                except (OSError, ValueError):
                    pass
            client.close()

    def refresh():
        nonlocal measurement_start, turn_phase
        while True:
            try:
                kind, value = events.get_nowait()
            except queue.Empty:
                break
            if kind == 'status':
                r = value
                if measurement_start is not None and turn_phase == 1 and (
                        r['state'] != 1 or r['statusword'] & 0x6f != 0x40 or
                        r['node'] != measurement_start['node']):
                    measurement_start, turn_phase = None, 0
                    events.put(('log', 'Hand-turn measurement invalidated: disabled-state/node changed. Start again after Reset drivers.'))
                status.set(f'Board {r["board_id"]} (configured) | Node {r["node"]}: {label(STATES,r["state"])} | error {label(ERRORS,r["error"])} | '
                           f'result {label(RESULTS,r["result"])}\n'
                           f'SW=0x{r["statusword"]:04X} mode={r["mode"]} target={r["target"]} actual={r["actual"]} '
                           f'CAN RX/TX={r["rx"]}/{r["tx"]} ESR=0x{r["esr"]:08X} abort=0x{r["abort"]:08X}')
                if 'valid' in r:
                    samples.append(dict(pc_time=time.time(), turn_phase=turn_phase, **r))
                    def sample(key, bit, stamp=None):
                        if r['unsupported'] & bit:
                            return 'N/A (unsupported)'
                        if not r['valid'] & bit:
                            return 'not sampled'
                        age = f' [{(r["motor_ms"] - r[stamp]) & 0xffffffff} ms old]' if stamp else ''
                        return str(r[key]) + age
                    feedback.set(
                        f'Drive target rpm: {sample("drive_target",16,"drive_target_ms")} | '
                        f'Demand rpm: {sample("demand",32,"demand_ms")} | '
                        f'Controlword: {sample("controlword",64,"controlword_ms")}\n'
                        f'Current demand / actual [mA]: {sample("current_demand",2)} / '
                        f'{sample("current_actual",4,"current_ms")} | '
                        f'Torque [per mille rated, calculated]: {sample("torque",8,"torque_ms")}\n'
                        f'Position [device units]: {sample("position",1,"position_ms")} | '
                        f'Position delta/sample: {sample("position_delta",1)}\n'
                        f'Warning: {bool(r["statusword"] & 0x80)} | Internal limit: '
                        f'{bool(r["statusword"] & 0x800)} | Halt: '
                        f'{bool(r["controlword"] & 0x100) if r["valid"] & 64 else "not sampled"} | '
                        f'Optional diagnostic abort: 0x{r["diag_abort"]:08X} '
                        f'at 0x{r["diag_object"]:04X}:{r["diag_sub"]:02X}')
                    feedback.set(feedback.get() + '\n' +
                                 ('Direction change: waiting for zero-speed feedback' if r['reversal_pending'] else
                                  'Direction change: none pending' if r['reversal_supported'] else
                                  'Reversal interlock unavailable: update MCU firmware'))
                    if 'hall' in r:
                        feedback.set(feedback.get() + '\n' +
                                     f'Hall={r["hall"]:03b} edges={r["hall_edges"]} skipped={r["hall_skipped"]} invalid={r["hall_invalid"]} | '
                                     f'Configured PPR={r["encoder_ppr"]}, pole pairs={r["pole_pairs"]}, position unit=0x{r["position_unit"]:08X}' +
                                     '\n' + f'PDO configured={bool(r["pdo_ready"])} | TPDO received={r["pdo_rx"]}')
                elif r['result'] == 2:
                    feedback.set('Extended diagnostics unsupported: update MCU firmware.')
            else:
                output.configure(state='normal')
                output.insert('end', time.strftime('%H:%M:%S ') + value + '\n')
                if int(output.index('end-1c').split('.')[0]) > 250:
                    output.delete('1.0', '100.0')
                output.see('end')
                output.configure(state='disabled')
        root.after(50, refresh)

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()
    root.protocol('WM_DELETE_WINDOW', lambda: (stop.set(), root.destroy()))
    refresh()
    root.mainloop()
    thread.join(timeout=1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.0.24', help='Firmware BOARD_CONFIG_ID: 1=.23, 2=.24 (default), 3=.25, 4=.26')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--bind', default='0.0.0.0', help='Optional PC Ethernet IPv4 address')
    parser.add_argument('--status', action='store_true', help='Read-only single status request; no GUI')
    args = parser.parse_args()
    if args.status:
        client = Client(args.host, args.port, args.bind)
        try:
            print(client.request(0))
        finally:
            client.close()
    else:
        gui(args)


if __name__ == '__main__':
    main()
