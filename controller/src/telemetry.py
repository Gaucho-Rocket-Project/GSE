"""Parsing and durable CSV storage for ground UART telemetry records."""
from __future__ import annotations

import csv
from datetime import datetime, timezone
from pathlib import Path


def parse_telemetry(line: str) -> tuple[int, int, str] | None:
    parts = line.split(' ', 3)
    if len(parts) != 4 or parts[0] != 'TLM':
        return None
    try:
        sequence, uptime = int(parts[1]), int(parts[2])
    except ValueError:
        return None
    payload = parts[3]
    if not (0 <= sequence <= 0xffffffff and 0 <= uptime <= 0xffffffff and
            1 <= len(payload) <= 200 and all(' ' <= c <= '~' for c in payload)):
        return None
    return sequence, uptime, payload


class TelemetryLog:
    def __init__(self, path: Path) -> None:
        self.path = path

    def append(self, sample: tuple[int, int, str]) -> str:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now(timezone.utc).isoformat(timespec='milliseconds')
        with self.path.open('a', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            if stream.tell() == 0:
                writer.writerow(('received_utc', 'sequence', 'vehicle_uptime_ms', 'payload'))
            writer.writerow((timestamp, *sample))
        return timestamp
