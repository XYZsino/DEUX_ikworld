"""Session-bound Newland serial reader. No ROS dependency or device access at import."""
from dataclasses import dataclass
from collections import deque
import math
import re

START = b'\x01T\x04'
STOP = b'\x01P\x04'
DEFAULT_PATTERN = r'(?:[0-9]{8}|[0-9]{12,14})'


@dataclass(frozen=True)
class Session:
    identity: str
    track_id: str
    started: float
    expires: float


def find_port(port='', serial_number=''):
    if port:
        return port
    from serial.tools import list_ports
    matches = [p.device for p in list_ports.comports()
               if (p.vid, p.pid) == (0x1EAB, 0x3306)
               and (not serial_number or p.serial_number == serial_number)]
    if len(matches) != 1:
        raise RuntimeError(f'expected one Newland dongle (1eab:3306), found {len(matches)}; set port/serial_number')
    return matches[0]


def connect(port='', serial_number='', baudrate=115200):
    import serial
    # Nonblocking reads; bounded command writes. Linux exclusive ownership.
    return serial.Serial(find_port(port, serial_number), baudrate=baudrate,
                         timeout=0, write_timeout=.05, exclusive=True)


class BarcodeController:
    def __init__(self, emit, *, connector=None, simulated=False, enabled=False,
                 pattern=DEFAULT_PATTERN, quiet_s=.2, drain_timeout_s=1.,
                 scan_timeout_s=5., reconnect_s=1., max_bytes=4096):
        for value in (quiet_s, drain_timeout_s, scan_timeout_s, reconnect_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('scanner time limits must be positive and finite')
        if drain_timeout_s < quiet_s or not 1 <= max_bytes <= 65536:
            raise ValueError('invalid scanner drain/buffer limits')
        self.pattern = re.compile(pattern)
        if self.pattern.fullmatch(''):
            raise ValueError('barcode pattern must not accept empty values')
        self.emit, self.connector = emit, connector
        self.simulated, self.enabled = simulated, enabled
        self.quiet_s, self.drain_timeout_s = quiet_s, drain_timeout_s
        self.scan_timeout_s, self.reconnect_s, self.max_bytes = scan_timeout_s, reconnect_s, max_bytes
        self.port = None
        self.session = None
        self.buffer = bytearray()
        self.seen = deque(maxlen=256)
        self.latest_start = -1.
        self.last_ros = None
        self.retry_at = 0.
        self.last_rx = self.drain_start = self.wall_deadline = 0.
        self.state = 'idle' if simulated else ('disconnected' if enabled else 'disabled')
        self.reason = 'waiting for a fresh scan session'

    def status(self, state, reason):
        self.state, self.reason = state, reason

    def write(self, command):
        if self.port.write(command) != len(command):
            raise IOError('incomplete scanner command write')

    def stop(self):
        if self.port is not None:
            self.write(STOP)
            self.port.reset_input_buffer()
        self.buffer.clear()

    def disconnect(self, reason, wall):
        # Invalidate the session before reconnect. Never resume an old trigger.
        self.session = None
        self.buffer.clear()
        if self.port is not None:
            try:
                self.port.write(STOP)
            except Exception:
                pass
            try:
                self.port.close()
            except Exception:
                pass
        self.port = None
        self.retry_at = wall + self.reconnect_s
        self.status('disconnected', reason)

    def finish(self, state, reason, wall):
        self.session = None
        try:
            self.stop()
        except Exception as error:
            self.disconnect(f'stop failed: {error}', wall)
            return False
        self.status(state, reason)
        return True

    def check_clock(self, now, wall):
        if self.last_ros is not None and now < self.last_ros:
            self.finish('canceled', 'ROS clock moved backwards; new session required', wall)
            self.latest_start = -1.
        self.last_ros = now

    def request(self, session, active, now, wall):
        self.check_clock(now, wall)
        if not active:
            # A terminal lease may arrive before its active delivery (or on a
            # restarted subscriber). Never subsequently rearm that same ID.
            if session.identity and session.identity not in self.seen:
                self.seen.append(session.identity)
            if self.session and session.identity == self.session.identity:
                self.finish('canceled', 'scan session closed by BT', wall)
            return
        # Duplicate lease deliveries never clear buffers, extend deadlines, or retrigger.
        if session.identity in self.seen:
            return
        if (not session.identity or not session.track_id
                or not all(math.isfinite(x) for x in (session.started, session.expires))
                or not 0 < session.started <= now < session.expires
                or session.expires - session.started > 120
                or session.started <= self.latest_start):
            self.reason = 'invalid, expired, future or out-of-order scan session'
            return
        self.seen.append(session.identity)
        self.latest_start = session.started
        if not self.simulated and (not self.enabled or self.port is None):
            self.status('disabled' if not self.enabled else 'disconnected',
                        'scanner unavailable; request a new session after connection')
            return
        if not self.finish('idle', 'preparing new session', wall):
            return
        self.session = session
        if self.simulated:
            self.status('scanning', 'waiting for simulated reader')
        else:
            self.last_rx = self.drain_start = wall
            self.wall_deadline = wall + (session.expires - now)
            self.status('draining', 'discarding input until serial line is quiet')

    def valid(self, now, wall):
        if self.session is None:
            return False
        if now >= self.session.expires or (not self.simulated and wall >= self.wall_deadline):
            self.finish('timeout', 'scan deadline expired; no valid barcode', wall)
            return False
        return True

    def value(self, raw, now, wall):
        if self.state != 'scanning' or not self.valid(now, wall):
            return
        if len(raw) > self.max_bytes:
            self.finish('error', 'barcode exceeds byte limit', wall)
            return
        try:
            value = raw.decode('utf-8', errors='strict')
        except UnicodeDecodeError:
            self.reason = 'rejected undecodable scanner data'
            return
        if (not value or any(ord(c) < 32 or ord(c) == 127 for c in value)
                or value.upper() in ('ACK', 'NAK', 'OK', 'ERROR', 'FAIL')
                or not self.pattern.fullmatch(value)):
            self.reason = 'rejected control/ACK or data outside barcode pattern'
            return
        session = self.session
        # Stop first. A stop I/O failure is diagnosed, not treated as successful control.
        if self.finish('read', 'barcode received; scanner stopped', wall):
            self.emit(session, value, now)

    def tick(self, now, wall):
        self.check_clock(now, wall)
        if self.simulated:
            self.valid(now, wall)
            return
        if not self.enabled:
            return
        if self.port is None:
            if wall < self.retry_at:
                return
            try:
                self.port = self.connector()
                self.stop()
                self.status('idle', 'connected; waiting for a fresh scan session')
            except Exception as error:
                self.disconnect(f'connection failed: {error}', wall)
            return
        try:
            data = self.port.read(self.max_bytes + 1)
            if data:
                self.last_rx = wall
            if self.session is None:
                return  # Unsolicited/late frames are drained even while idle.
            if not self.valid(now, wall):
                return
            if self.state == 'draining':
                if wall - self.drain_start >= self.drain_timeout_s:
                    self.finish('error', 'serial input did not become quiet before trigger', wall)
                elif not data and wall - self.last_rx >= self.quiet_s:
                    self.port.reset_input_buffer()
                    self.write(START)
                    self.wall_deadline = min(self.wall_deadline, wall + self.scan_timeout_s)
                    self.status('scanning', 'trigger sent; waiting for CR-terminated barcode')
                return
            if self.state != 'scanning':
                return
            self.buffer.extend(data)
            if len(self.buffer) > self.max_bytes:
                self.finish('error', 'serial frame overflow; session stopped', wall)
                return
            while b'\r' in self.buffer and self.session is not None:
                raw, _, remainder = self.buffer.partition(b'\r')
                self.buffer = bytearray(remainder)
                self.value(bytes(raw).lstrip(b'\n'), now, wall)
        except Exception as error:
            self.disconnect(f'serial I/O failed: {error}', wall)

    def close(self):
        self.session = None
        try:
            self.stop()
        finally:
            if self.port is not None:
                self.port.close()
                self.port = None
