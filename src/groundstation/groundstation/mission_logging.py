"""Per-session CSV logging of mission commands, events, and positions.

Writes mission telemetry to timestamped CSV files, one file per session.
Disabled by default; call :meth:`MissionCsvLogger.enable` to start a session.
"""

import csv
import logging
import os
import time
from datetime import datetime

logger = logging.getLogger(__name__)


class MissionCsvLogger:
    """Writes mission telemetry to timestamped CSV files (one session each)."""

    DEFAULT_LOG_DIR = "~/skyloop_logs"

    def __init__(self, log_dir: str = DEFAULT_LOG_DIR):
        self.log_dir = os.path.expanduser(log_dir)
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.commands_file = None
        self.events_file = None
        self.positions_file = None
        self.enabled = False

    def enable(self, log_dir: str = None):
        """Start a logging session, creating the CSV files with headers."""
        if log_dir:
            self.log_dir = os.path.expanduser(log_dir)

        self.enabled = True
        os.makedirs(self.log_dir, exist_ok=True)

        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.commands_file = os.path.join(self.log_dir, f"commands_{self.session_id}.csv")
        self.events_file = os.path.join(self.log_dir, f"events_{self.session_id}.csv")
        self.positions_file = os.path.join(self.log_dir, f"positions_{self.session_id}.csv")
        self._init_files()

        logger.info(f"📁 CSV logging enabled: {self.log_dir}")
        logger.info(f"   Commands: {self.commands_file}")
        logger.info(f"   Events: {self.events_file}")
        logger.info(f"   Positions: {self.positions_file}")

    def disable(self):
        """Stop logging (files are left as-is)."""
        self.enabled = False
        logger.info("📁 CSV logging disabled")

    def _init_files(self):
        """Write the header row to each CSV file."""
        with open(self.commands_file, 'w', newline='') as f:
            csv.writer(f).writerow(
                ['timestamp', 'datetime', 'namespace', 'command', 'arguments'])
        with open(self.events_file, 'w', newline='') as f:
            csv.writer(f).writerow(
                ['timestamp', 'datetime', 'namespace', 'state', 'message'])
        with open(self.positions_file, 'w', newline='') as f:
            csv.writer(f).writerow(
                ['timestamp', 'datetime', 'namespace', 'latitude', 'longitude',
                 'altitude', 'heading', 'battery', 'state'])

    def write_command(self, namespace: str, command: str, args: str):
        """Append a command row."""
        if not self.enabled or not self.commands_file:
            return
        try:
            with open(self.commands_file, 'a', newline='') as f:
                csv.writer(f).writerow(
                    [time.time(), datetime.now().isoformat(), namespace, command, args])
        except Exception as e:
            logger.error(f"Failed to write command to CSV: {e}")

    def write_event(self, namespace: str, state: str, message: str):
        """Append an event row."""
        if not self.enabled or not self.events_file:
            return
        try:
            with open(self.events_file, 'a', newline='') as f:
                csv.writer(f).writerow(
                    [time.time(), datetime.now().isoformat(), namespace, state, message])
        except Exception as e:
            logger.error(f"Failed to write event to CSV: {e}")

    def write_position(self, namespace: str, lat: float, lon: float, alt: float,
                       heading: float, battery: float, state: str):
        """Append a position row."""
        if not self.enabled or not self.positions_file:
            return
        try:
            with open(self.positions_file, 'a', newline='') as f:
                csv.writer(f).writerow([
                    time.time(),
                    datetime.now().isoformat(),
                    namespace,
                    f"{lat:.8f}",
                    f"{lon:.8f}",
                    f"{alt:.2f}",
                    f"{heading:.1f}",
                    f"{battery:.1f}",
                    state,
                ])
        except Exception as e:
            logger.error(f"Failed to write position to CSV: {e}")

    def get_files(self) -> dict:
        """Return the current session's file paths."""
        return {
            'commands': self.commands_file,
            'events': self.events_file,
            'positions': self.positions_file,
            'directory': self.log_dir,
        }
