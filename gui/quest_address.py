"""Where is the Quest right now?  Ask the robot's NT server, not a ping scan.

WHY THIS EXISTS
---------------
The double-tap fix runs `adb -s <ip>:5802 shell am start ...`.  It used to take <ip> from
config.QUESTNAV_ADB_ADDRESS, which is the hard-coded .200 until someone presses "Ping Quest",
and the ping scan only overwrites it once it has finished walking .200, .201, ... - so a
double-tap during the scan (or any time after the Quest's DHCP address changed and nobody
re-pinged) went to the stale .200 while the log showed the scan pinging .201.  A ping reply
also only proves SOMETHING is at that address, not that it is the Quest.

Every NT4 server publishes the meta-topic `$clients`: a msgpack array of
{"id": "<identity>@<n>", "conn": "<ip>:<port>", "ver": ...}, one per connected client.  The
QuestNav app is an NT client of the robot, so its entry carries the IP the Quest is actually
talking from.  That is the address adb needs.
"""

import struct

ADB_PORT = 5802


def _unpack(buf: bytes, i: int):
    """Minimal msgpack decoder - just the types $clients uses (so no msgpack dependency)."""
    b = buf[i]
    i += 1
    if b <= 0x7f:                       # positive fixint
        return b, i
    if 0x80 <= b <= 0x8f:               # fixmap
        return _map(buf, i, b & 0x0f)
    if 0x90 <= b <= 0x9f:               # fixarray
        return _array(buf, i, b & 0x0f)
    if 0xa0 <= b <= 0xbf:               # fixstr
        n = b & 0x1f
        return buf[i:i + n].decode('utf-8', 'replace'), i + n
    if b == 0xc0:
        return None, i
    if b in (0xc2, 0xc3):
        return b == 0xc3, i
    sizes = {0xcc: 'B', 0xcd: '>H', 0xce: '>I', 0xcf: '>Q', 0xd0: 'b', 0xd1: '>h', 0xd2: '>i', 0xd3: '>q'}
    if b in sizes:
        fmt = sizes[b]
        n = struct.calcsize(fmt)
        return struct.unpack(fmt, buf[i:i + n])[0], i + n
    if b in (0xd9, 0xda, 0xdb):         # str8 / str16 / str32
        fmt = {0xd9: 'B', 0xda: '>H', 0xdb: '>I'}[b]
        h = struct.calcsize(fmt)
        n = struct.unpack(fmt, buf[i:i + h])[0]
        i += h
        return buf[i:i + n].decode('utf-8', 'replace'), i + n
    if b in (0xdc, 0xdd):               # array16 / array32
        h = 2 if b == 0xdc else 4
        return _array(buf, i + h, int.from_bytes(buf[i:i + h], 'big'))
    if b in (0xde, 0xdf):               # map16 / map32
        h = 2 if b == 0xde else 4
        return _map(buf, i + h, int.from_bytes(buf[i:i + h], 'big'))
    raise ValueError(f'unsupported msgpack byte 0x{b:02x}')


def _array(buf, i, n):
    out = []
    for _ in range(n):
        v, i = _unpack(buf, i)
        out.append(v)
    return out, i


def _map(buf, i, n):
    out = {}
    for _ in range(n):
        k, i = _unpack(buf, i)
        v, i = _unpack(buf, i)
        out[k] = v
    return out, i


def parse_clients(raw: bytes) -> list[tuple[str, str]]:
    """$clients bytes -> [(id, ip), ...].  Bad or empty data -> []."""
    try:
        clients, _ = _unpack(raw, 0)
    except (IndexError, ValueError, struct.error):
        return []
    out = []
    for c in clients if isinstance(clients, list) else []:
        if isinstance(c, dict) and isinstance(c.get('conn'), str):
            out.append((str(c.get('id', '')), c['conn'].rsplit(':', 1)[0]))
    return out


def quest_ip(raw: bytes) -> str | None:
    """IP of the QuestNav client, or None if it is not connected to the robot.
    The newest matching connection wins (stale ones are dropped by the server anyway)."""
    found = [ip for ident, ip in parse_clients(raw) if 'quest' in ident.lower()]
    return found[-1] if found else None
