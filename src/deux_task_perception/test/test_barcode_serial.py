"""Protocol/session tests with deterministic clocks and an in-memory serial peer."""
from types import SimpleNamespace
import pytest
from deux_task_perception.barcode_serial import BarcodeController, Session, START, STOP, find_port


class SerialPeer:
    def __init__(self):
        self.input = bytearray()
        self.commands = []
        self.closed = False
        self.fail_read = False
        self.fail_stop = False
        self.short_write = False

    def write(self, data):
        if self.fail_stop and data == STOP:
            raise IOError('stop disconnected')
        self.commands.append(data)
        return 1 if self.short_write else len(data)

    def read(self, size):
        if self.fail_read:
            raise IOError('USB disconnected')
        result = bytes(self.input[:size]); del self.input[:size]
        return result

    def reset_input_buffer(self):
        self.input.clear()

    def close(self):
        self.closed = True


def setup(**kwargs):
    peer, reads = SerialPeer(), []
    c = BarcodeController(lambda s, v, t: reads.append((s, v, t)), connector=lambda: peer, enabled=True, **kwargs)
    c.tick(100., 10.)
    assert c.state == 'idle' and peer.commands == [STOP]
    return c, peer, reads


def request(c, identity='a', now=100., wall=10.01, ttl=10.):
    s = Session(identity, 'bag', now, now + ttl)
    c.request(s, True, now, wall)
    return s


def scanning(c):
    c.tick(100.22, 10.23)
    assert c.state == 'scanning'


def test_fragmented_barcode_preserves_zeros_and_emits_only_once():
    c, p, reads = setup(); request(c); scanning(c)
    for data, dt in [(b'001234', .25), (b'5678901', .27)]:
        p.input.extend(data); c.tick(100 + dt, 10 + dt)
    assert not reads
    p.input.extend(b'\r\n2087231250396\r')
    c.tick(100.3, 10.3)
    assert [(x.identity, v, t) for x, v, t in reads] == [('a', '0012345678901', 100.3)]
    assert c.session is None and c.state == 'read' and p.commands[-1] == STOP
    p.input.extend(b'2087231250396\r'); c.tick(100.4, 10.4)
    assert len(reads) == 1


@pytest.mark.parametrize('raw', [b'\x06\r', b'\x15\r', b'OK\r', b'ACK\r', b'ERROR\r',
                                 b'~\x010000#ORTSET5000;\x03\r', b'\xff\r', b'123\r', b'\r'])
def test_ack_control_partial_invalid_frames_never_become_reads(raw):
    c, p, reads = setup(); request(c); scanning(c)
    p.input.extend(raw); c.tick(100.3, 10.3)
    assert not reads and c.state == 'scanning'
    p.input.extend(b'2087231250396\r'); c.tick(100.4, 10.4)
    assert len(reads) == 1


def test_new_session_discards_old_partial_and_late_input_during_drain():
    c, p, reads = setup(); request(c); scanning(c)
    p.input.extend(b'001234'); c.tick(100.3, 10.3)
    new = request(c, 'b', 100.4, 10.4)
    p.input.extend(b'5678901\r'); c.tick(100.5, 10.5)
    c.tick(100.65, 10.65); assert c.state == 'draining' and not reads
    c.tick(100.72, 10.72); assert c.state == 'scanning'
    p.input.extend(b'2087231250396\r'); c.tick(100.8, 10.8)
    assert len(reads) == 1 and reads[0][0] == new


def test_duplicate_or_stale_session_cannot_retrigger_or_cancel_new_scan():
    c, p, reads = setup(); first = request(c); scanning(c)
    commands = list(p.commands)
    c.request(first, True, 100.3, 10.3)
    assert p.commands == commands
    second = request(c, 'b', 100.4, 10.4)
    c.request(first, False, 100.5, 10.5)
    c.request(Session('old', 'bag', 99., 110.), True, 100.5, 10.5)
    assert c.session == second
    c.request(second, False, 100.6, 10.6)
    assert c.state == 'canceled' and c.session is None
    c.request(second, True, 100.7, 10.7)
    assert c.session is None


def test_closed_lease_arriving_before_active_delivery_cannot_start():
    c, p, reads = setup()
    session = Session('already-closed', 'bag', 100., 110.)
    c.request(session, False, 100.1, 10.1)
    c.request(session, True, 100.2, 10.2)
    c.tick(100.5, 10.5)
    assert c.session is None and START not in p.commands and not reads


@pytest.mark.parametrize('kind', ['ros_expiry', 'paused_ros', 'backwards', 'cancel_drain', 'overflow', 'noise'])
def test_bounded_stop_paths(kind):
    c, p, reads = setup(max_bytes=32)
    s = request(c)
    if kind == 'cancel_drain':
        c.request(s, False, 100.1, 10.1)
    elif kind == 'noise':
        for dt in [.2, .4, .6, .8, 1.1]:
            p.input.extend(b'x'); c.tick(100 + dt, 10 + dt)
    else:
        scanning(c)
        if kind == 'ros_expiry':c.tick(110., 10.4)
        if kind == 'paused_ros':c.tick(100.22, 16.)
        if kind == 'backwards':c.tick(1., 10.4)
        if kind == 'overflow':p.input.extend(b'x' * 33); c.tick(100.3, 10.3)
    assert c.session is None and not reads and p.commands[-1] == STOP
    if kind in ('cancel_drain', 'noise'):assert START not in p.commands


def test_io_failure_invalidates_session_and_requires_fresh_request_after_reconnect():
    c, p, reads = setup(); s = request(c); scanning(c)
    p.fail_read = True; c.tick(100.3, 10.3)
    assert c.state == 'disconnected' and p.closed and c.session is None
    replacement = SerialPeer(); c.connector = lambda: replacement
    c.tick(101.4, 11.4)
    assert c.state == 'idle' and replacement.commands == [STOP]
    c.request(s, True, 101.5, 11.5)
    assert c.session is None and START not in replacement.commands
    request(c, 'new', 101.6, 11.6); c.tick(101.9, 11.9)
    assert replacement.commands[-1] == START


def test_stop_failure_does_not_publish_success_and_short_write_rejected():
    c, p, reads = setup(); request(c); scanning(c)
    p.fail_stop = True; p.input.extend(b'2087231250396\r'); c.tick(100.3, 10.3)
    assert not reads and c.state == 'disconnected' and p.closed
    c, p, reads = setup(); request(c); p.short_write = True; c.tick(100.3, 10.3)
    assert c.state == 'disconnected' and not reads


def test_disabled_never_opens_or_discovers_a_device():
    calls = []
    c = BarcodeController(lambda *args: None, connector=lambda: calls.append(True))
    c.tick(100., 10.); request(c)
    assert c.state == 'disabled' and not calls


@pytest.mark.parametrize('session', [Session('', 'bag', 100., 110.), Session('a', '', 100., 110.),
    Session('a', 'bag', 101., 110.), Session('a', 'bag', 99., 100.), Session('a', 'bag', 100., 300.)])
def test_invalid_session_never_triggers(session):
    c, p, reads = setup(); c.request(session, True, 100., 10.1)
    assert c.session is None and START not in p.commands


def test_explicit_port_and_dongle_selection(monkeypatch):
    from serial.tools import list_ports
    def port(device, serial, vid=0x1EAB):
        return SimpleNamespace(device=device, vid=vid, pid=0x3306, serial_number=serial)
    monkeypatch.setattr(list_ports, 'comports', lambda: [port('/dev/fake0', 'A'), port('/dev/fake1', 'B')])
    assert find_port('/dev/pts/0') == '/dev/pts/0'
    assert find_port(serial_number='B') == '/dev/fake1'
    with pytest.raises(RuntimeError, match='found 2'):find_port()
    with pytest.raises(RuntimeError, match='found 0'):find_port(serial_number='missing')
