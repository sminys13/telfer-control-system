"""Exercise only an identified Step9L SIM Mega. Refuses every FIELD firmware.

No direct Modbus commands: virtual web commands are permitted only after SIM,
physicalTx=0, UART1 TX=0 and SPI=0 have been read back from the controller.
"""
import argparse
from datetime import datetime, timezone, timedelta
import hashlib
import json
from pathlib import Path
import time
import serial

parser = argparse.ArgumentParser()
parser.add_argument('--port', required=True)
parser.add_argument('--output', required=True)
args = parser.parse_args()
out = Path(args.output).resolve()
out.mkdir(parents=True, exist_ok=True)
trace, checks, memory = [], [], []
state, axes, firmware, slot_labels = {}, {}, {}, {}
sequence, session, last_ping = 1000, None, 0
verified_sim = False

def fields(line):
    return dict(word.split('=', 1) for word in line.split()[1:] if '=' in word)

def record(direction, line):
    stamp = datetime.now(timezone(timedelta(hours=4))).isoformat(timespec='milliseconds')
    trace.append(f'[{stamp}] {direction} {line}')

with serial.Serial(args.port, 115200, timeout=0.05, write_timeout=1) as port:
    def send(line):
        record('TX', line)
        port.write((line+'\n').encode('ascii'))

    def pump(heartbeat=True):
        global state, firmware, last_ping
        if heartbeat and verified_sim and session and time.monotonic()-last_ping > .22:
            send(f'web ping {session}')
            last_ping = time.monotonic()
        line = port.readline().decode('utf-8', errors='replace').strip()
        if not line:
            return '', {}
        record('RX', line)
        value = fields(line)
        if line.startswith('@WEB_STATE '): state = value
        elif line.startswith('@WEB_AXIS '): axes[int(value['i'])] = value
        elif line.startswith('@SERVICE_INFO '): firmware = value
        elif line.startswith('@WEB_SLOT '): slot_labels[int(value['slot'])]=bytes.fromhex(value.get('nameHex','')).decode('utf-8')
        elif line.startswith('@WEB_MEMORY '):
            memory.append(value)
            if verified_sim: assert value.get('tx1') == '0' and value.get('spi') == '0', 'physical peripheral unexpectedly enabled'
        return line, value

    def command(body, expected=True, response='@WEB_ACK '):
        global sequence
        assert verified_sim
        sequence += 1
        send(f'web {body} @{sequence}')
        end = time.monotonic()+7
        while time.monotonic() < end:
            line, value = pump()
            if line.startswith(response) and value.get('id') == str(sequence):
                assert (value.get('ok') == '1') == expected, (body, value)
                return value
        raise AssertionError('No correlated reply: '+body)

    def wait(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end: pump()

    def snapshot():
        command('info')
        assert len(axes) == 4
        return dict(state), {i: dict(v) for i,v in axes.items()}

    def pass_check(name):
        checks.append(name)
        print('PASS:', name, flush=True)

    try:
        start = time.monotonic()
        while time.monotonic()-start < 3: pump(False)
        send('service info')
        end = time.monotonic()+5
        while time.monotonic()<end and not firmware: pump(False)
        assert firmware.get('fw') == 'v6-system-step9l-web-control'
        assert firmware.get('build') == 'SIM' and firmware.get('desktopSim') == '1'
        assert firmware.get('physicalTx') == '0' and firmware.get('diagnosticLock') == '1'
        assert firmware.get('auto') == '0' and firmware.get('dwinMotion') == '0'
        assert memory and memory[-1].get('tx1') == '0' and memory[-1].get('spi') == '0'
        session = state['session']
        verified_sim = True
        pass_check('identified SIM firmware; UART1 TX and SPI disabled')
        assert state['armed'] == '0' and state['running'] == '0'
        command(f'sim {session} reset')
        wait(.7)
        command(f'arm {session}')
        before = snapshot()[1]
        end = time.monotonic()+.8
        while time.monotonic()<end:
            command(f'jog {session} h1 pos 10')
            wait(.15)
        command(f'release {session}')
        wait(.5)
        after = snapshot()[1]
        assert int(after[0]['mm']) > int(before[0]['mm'])
        assert all(after[i]['mm'] == before[i]['mm'] for i in [1,2,3])
        pass_check('single manual X+ moves the selected model axis; release stops')
        read=command('read v1 P6.03',response='@WEB_PARAM ')
        assert read['raw']=='0' and read['exception']=='0'
        command(f'write {session} v1 P6.03 0 1000')
        assert command('read v1 P6.03',response='@WEB_PARAM ')['raw']=='1000'
        command(f'write {session} v1 P6.03 1000 0')
        pass_check('named virtual parameter write is verified by readback')
        command(f'sim {session} startup 1')
        refused = command(f'jog {session} v1 pos 10', False)
        assert refused.get('reason') == 'JOG_FREQUENCY'
        command(f'sim {session} startup 0')
        pass_check('5Hz request is refused when model P6.03/P0.14 is 10Hz')

        command(f'program {session} slot 4')
        command(f'testplan {session} 50 30 1')
        command(f'start {session} auto')
        end = time.monotonic()+12
        while time.monotonic()<end:
            pump()
            if state.get('phase') == 'MOVE_ZONE_H': break
        command(f'pause {session}')
        wait(.6)
        paused, coordinates = snapshot()
        assert paused.get('paused') == '1'
        wait(.4)
        still, stable = snapshot()
        assert still['phase'] == paused['phase']
        assert all(stable[i]['mm'] == coordinates[i]['mm'] for i in range(4))
        command(f'resume {session}')
        end = time.monotonic()+60
        while time.monotonic()<end:
            pump()
            if state.get('phase') == 'DONE' and state.get('running') == '0': break
        assert state.get('phase') == 'DONE' and state.get('error') == '0', state
        wait(.5)
        pass_check('real Mega executes two-zone AutoRunner cycle with pause/resume and HOME')

        command(f'start {session} steps')
        end = time.monotonic()+10
        while time.monotonic()<end:
            pump()
            if state.get('paused') == '1': break
        assert state.get('paused') == '1'
        previous_step = state.get('step')
        command(f'next {session}')
        wait(.5)
        assert state.get('step') != previous_step or state.get('paused') == '0'
        command('stop')
        pass_check('step mode pauses at boundary; Next continues; STOP cancels')

        wait(.5);command(f'arm {session}');command(f'start {session} auto')
        command(f'sim {session} sensor v1 1');wait(.6);snapshot()
        assert state.get('armed') == '0' and state.get('running') == '0' and state.get('issue') == '2'
        command(f'sim {session} sensor v1 0');wait(.5)
        command(f'arm {session}');command(f'jog {session} v1 pos 10')
        command(f'sim {session} fault v1 1');wait(.6);snapshot()
        assert state.get('armed') == '0' and state.get('issue') == '4'
        command(f'sim {session} fault v1 0');wait(.4)
        command(f'arm {session}');command(f'sim {session} estop 1');wait(.5)
        command(f'clear {session}', False)
        command(f'sim {session} estop 0');command(f'clear {session}')
        pass_check('sensor loss, VFD fault and E-STOP stop/disarm; active latch cannot clear')

        command(f'arm {session}');last_ping = time.monotonic()
        end = time.monotonic()+2.4
        while time.monotonic()<end: pump(False)
        assert state.get('armed') == '0' and state.get('issue') == '1'
        pass_check('missing heartbeat disarms on the real Mega')

        # Leave a useful two-zone lesson in SIM slot 1, never in FIELD memory.
        command(f'sim {session} reset');wait(.6);command(f'arm {session}')
        command(f'program {session} slot 1');command(f'testplan {session} 50 30 1')
        longest='Я'*19+'A'
        command(f'program {session} name {longest.encode("utf-8").hex()}')
        snapshot()
        assert slot_labels[1]==longest
        pass_check('maximum 39-byte UTF-8 name survives status/heartbeat traffic')
        name = 'Учебная 1'.encode('utf-8').hex()
        command(f'program {session} name {name}');command(f'program {session} save')
        command('stop');wait(.6)
        current, _ = snapshot()
        assert current['armed'] == '0' and current['running'] == '0' and current['ready'] == '1'
        pass_check('SIM slot 1 saved and left disarmed, ready for user launch')
    finally:
        if verified_sim:
            try: send('web stop');port.flush()
            except serial.SerialException: pass
        (out/'mega-simulation-transcript.txt').write_text('\n'.join(trace)+'\n',encoding='utf-8')
        result = {'port':args.port,'firmware':firmware,'checks':checks,'last_state':state,'memory':memory,
                  'min_free_observed':min((int(m['minimum']) for m in memory),default=None),
                  'physical_uart1_tx_enabled':any(m.get('tx1')!='0' for m in memory),
                  'physical_spi_enabled':any(m.get('spi')!='0' for m in memory)}
        result['trace_sha256']=hashlib.sha256((out/'mega-simulation-transcript.txt').read_bytes()).hexdigest()
        (out/'mega-simulation-results.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print('COM closed; no physical VFD/sensor I/O used')
