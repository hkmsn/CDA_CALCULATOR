"""Host-only unit tests for the hardware resilience serial parser."""

import unittest

from hardware_resilience_test import parse_capture


class CaptureParserTests(unittest.TestCase):
    def test_parses_rtc_telemetry(self):
        capture = parse_capture(
            "\r2026-07-22 12:17:57.00|1.20|1.1840|25.00|101325.0|"
            "3.20|0.01|NA|0.04|-0.003|0.0089|-0.0326|1.0030*   "
        )
        self.assertEqual(len(capture.telemetry), 1)
        self.assertEqual(capture.telemetry[0][0], "1.20")
        self.assertEqual(capture.telemetry[0][-1], "1.0030")

    def test_ignores_interleaved_incomplete_record(self):
        capture = parse_capture(
            "12:17:57:00|NA|NA|[W][Sensor] recovery|NA*\n"
            "12:17:59:00|NA|NA|NA|NA|NA|0.00|NA|NA|NA|NA|NA|NA*"
        )
        self.assertEqual(len(capture.telemetry), 1)

    def test_parses_health_counters(self):
        capture = parse_capture(
            "Health: writeFailures=2 failovers=1 logDrops=3 queueDrops=4 "
            "i2cRecoveries=5 imuRecoveries=6 truncations=7"
        )
        self.assertEqual(capture.health[-1]["queueDrops"], 4)
        self.assertEqual(capture.health[-1]["i2cRecoveries"], 5)


if __name__ == "__main__":
    unittest.main()
