import csv
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from telemetry import TelemetryLog, parse_telemetry


class TelemetryTests(unittest.TestCase):
    def test_parse_and_reject_invalid(self):
        self.assertEqual(parse_telemetry('TLM 42 1000 pressure,12.5'), (42, 1000, 'pressure,12.5'))
        for line in ('TLM a 1 data', 'TLM -1 1 data', 'TLM 1 2 ',
                     'TLM 1 2 ' + 'x' * 201, 'ACK 1 2 3', 'TLM 1 2 x\ty'):
            self.assertIsNone(parse_telemetry(line), line)

    def test_append_csv_quotes_and_only_one_header(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'data' / 'telemetry.csv'
            log = TelemetryLog(path)
            log.append((1, 30, 'a,"b"'))
            log.append((2, 40, 'next'))
            with path.open(newline='', encoding='utf-8') as stream:
                rows = list(csv.reader(stream))
            self.assertEqual(rows[0], ['received_utc', 'sequence', 'vehicle_uptime_ms', 'payload'])
            self.assertEqual(rows[1][1:], ['1', '30', 'a,"b"'])
            self.assertEqual(rows[2][1:], ['2', '40', 'next'])
            self.assertEqual(len(rows), 3)


if __name__ == '__main__':
    unittest.main()
