"""Return-to-home (RTH) time prediction from battery-drain telemetry.

Linear regression over battery history to predict when DJI will trigger RTH.
Extracted verbatim from ``perpetual_monitor`` (behaviour unchanged); the only
edits are moving the numpy/time/csv/os imports to module level instead of the
former ``self.np``/``self._time`` class-attribute pattern.
"""

import csv
import os
import time
from datetime import datetime

import numpy as np


class DroneRTHPredictor:
    """Predicts when DJI will trigger RTH based on battery drain.

    Uses linear regression on battery history to predict when battery will reach
    the RTH trigger threshold (batteryNeededToGoHome + margin).

    DJI returns battery as integer steps, so we only keep the FIRST point
    of each battery level to avoid skewing the regression.

    This is a simplified version for ROS2 integration - receives data from
    DroneData.
    """

    MAX_POINTS = 100  # Max unique battery levels to keep (100% to 0%)
    RTH_TRIGGER_MARGIN = 2  # RTH triggers when battery <= batt_needed_rth + margin
    MIN_DATAPOINTS = 3  # Minimum battery points needed before using RTH predictor

    def __init__(self, namespace: str = "drone"):
        self.namespace = namespace

        # Data storage - only first point per battery level
        # Key: battery level (int), Value: (timestamp, batt_needed_rth)
        self.battery_points: dict = {}

        # Track max battery needed to go home (for conservative prediction)
        self.max_batt_needed_rth = 0.0

        # Track last seen battery level to detect changes
        self.last_battery_level = None

        # State
        self.is_active = False  # Only predict when drone is in MONITORING state
        self.start_time = None

        # CSV logging
        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None

    def _init_csv_logging(self):
        """Initialize CSV file for logging RTH predictor data."""
        if self.csv_file is not None:
            return  # Already initialized

        log_dir = os.path.expanduser("~/rth_predictor_logs")
        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_ns = self.namespace.replace("/", "_")
        self.csv_path = os.path.join(log_dir, f"rth_predictor_{safe_ns}_{timestamp}.csv")

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'elapsed_time',
            'battery_level',
            'batt_needed_to_go_home',
            'max_batt_needed_rth',
            'rth_threshold',
            'drain_rate_per_min',
            'predicted_rth_seconds',
            'data_points',
            'slope',
            'intercept'
        ])

    def _log_to_csv(self, battery: float, batt_needed: float):
        """Log current state to CSV file."""
        if not self.is_active or self.start_time is None:
            return

        if self.csv_writer is None:
            self._init_csv_logging()

        elapsed = time.time() - self.start_time

        # Get current prediction info
        times, batteries = self._get_regression_data()
        slope = 0.0
        intercept = 0.0
        drain_rate = 0.0
        predicted_rth = float('inf')
        rth_threshold = self.max_batt_needed_rth + self.RTH_TRIGGER_MARGIN

        if times is not None and len(times) >= 2:
            try:
                slope, intercept = np.polyfit(times, batteries, 1)
                drain_rate = -slope * 60  # %/min

                if slope < 0:
                    t_rth = (rth_threshold - intercept) / slope
                    predicted_rth = max(0.0, t_rth - elapsed)
            except:
                pass

        self.csv_writer.writerow([
            f"{elapsed:.2f}",
            f"{battery:.1f}",
            f"{batt_needed:.1f}",
            f"{self.max_batt_needed_rth:.1f}",
            f"{rth_threshold:.1f}",
            f"{drain_rate:.4f}",
            f"{predicted_rth:.1f}" if predicted_rth != float('inf') else "inf",
            len(self.battery_points),
            f"{slope:.6f}",
            f"{intercept:.2f}"
        ])
        self.csv_file.flush()  # Ensure data is written immediately

    def close_csv(self):
        """Close CSV file when done."""
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

    def reset(self):
        """Reset the predictor when drone state changes."""
        self.close_csv()

        self.battery_points.clear()
        self.max_batt_needed_rth = 0.0
        self.last_battery_level = None
        self.start_time = None
        self.is_active = False

    def update(self, battery: float, batt_needed_to_go_home: float, is_monitoring: bool):
        """Update predictor with new telemetry data.

        Only stores the FIRST timestamp for each battery level (integer).
        DJI returns battery as steps, so multiple readings at same level
        would skew the regression.

        Args:
            battery: Current battery percentage
            batt_needed_to_go_home: DJI's batteryNeededToGoHome value
            is_monitoring: Whether drone is currently in MONITORING state
        """
        # Start/stop tracking based on monitoring state
        if is_monitoring and not self.is_active:
            # Just started monitoring - reset and start fresh
            self.reset()
            self.is_active = True
            self.start_time = time.time()
        elif not is_monitoring and self.is_active:
            # Stopped monitoring - deactivate but keep data for reference
            self.is_active = False
            return

        if not self.is_active:
            return

        # Convert battery to integer (DJI uses integer steps)
        battery_int = int(battery)

        # Only store the FIRST point for each battery level
        if battery_int not in self.battery_points:
            elapsed = time.time() - self.start_time
            self.battery_points[battery_int] = (elapsed, batt_needed_to_go_home)

        self.last_battery_level = battery_int

        # Track maximum battery needed to go home (conservative approach)
        if batt_needed_to_go_home > self.max_batt_needed_rth:
            self.max_batt_needed_rth = batt_needed_to_go_home

        # Log data to CSV
        self._log_to_csv(battery, batt_needed_to_go_home)

    def _get_regression_data(self):
        """Get arrays of (timestamps, battery_levels) for regression.

        Returns:
            Tuple of (times_array, batteries_array) sorted by time,
            or (None, None) if insufficient data.
        """
        if len(self.battery_points) < 2:
            return None, None

        # Extract and sort by timestamp
        points = [(t, batt) for batt, (t, _) in self.battery_points.items()]
        points.sort(key=lambda x: x[0])  # Sort by timestamp

        times = np.array([p[0] for p in points])
        batteries = np.array([p[1] for p in points])

        return times, batteries

    def predict_rth_time(self) -> float:
        """Predict time until RTH is triggered in seconds.

        Uses only one point per battery level for accurate regression.

        Returns:
            Predicted seconds until RTH, or float('inf') if cannot predict.
        """
        if not self.is_active or self.start_time is None:
            return float('inf')

        # Get regression data (one point per battery level)
        times, batteries = self._get_regression_data()
        if times is None:
            return float('inf')

        # Linear regression: battery = slope*t + intercept
        try:
            slope, intercept = np.polyfit(times, batteries, 1)
        except:
            return float('inf')

        # slope should be negative (battery draining)
        if slope >= 0:
            return float('inf')  # Battery not draining

        # Use MAX battery needed to go home for conservative prediction
        # This ensures we don't underestimate when RTH will trigger
        rth_threshold = self.max_batt_needed_rth + self.RTH_TRIGGER_MARGIN

        # Current time (use actual current time relative to start, not last stored timestamp)
        current_time = time.time() - self.start_time

        # Find when battery line crosses threshold
        # battery(t) = slope * t + intercept = rth_threshold
        # t_rth = (rth_threshold - intercept) / slope
        t_rth = (rth_threshold - intercept) / slope

        # Time until RTH
        time_until_rth = t_rth - current_time

        return max(0.0, time_until_rth)

    def get_datapoints(self) -> int:
        """Number of datapoints collected for the RTH prediction regression."""
        return len(self.battery_points)

    def get_drain_rate(self) -> float:
        """Battery drain rate in %/second using one point per battery level."""
        times, batteries = self._get_regression_data()
        if times is None:
            return 0.0

        try:
            slope, intercept = np.polyfit(times, batteries, 1)
            return -slope  # Negative slope = positive drain rate
        except:
            return 0.0

    def get_debug_info(self) -> dict:
        """Detailed debug information about the prediction.

        Returns:
            Dict with all computation details for debugging.
        """
        # Get current battery level from the most recent point
        current_battery = self.last_battery_level if self.last_battery_level is not None else 0.0

        # Get current batt_needed_rth from the last stored point
        current_batt_needed = 0.0
        if self.battery_points and self.last_battery_level in self.battery_points:
            _, current_batt_needed = self.battery_points[self.last_battery_level]

        info = {
            'is_active': self.is_active,
            'data_points': len(self.battery_points),
            'current_battery': float(current_battery),
            'batt_needed_to_go_home': current_batt_needed,
            'max_batt_needed_to_go_home': self.max_batt_needed_rth,
            'rth_threshold': 0.0,
            'slope': 0.0,
            'intercept': 0.0,
            'drain_rate_per_min': 0.0,
            'elapsed_since_monitoring': 0.0,
            't_rth_absolute': 0.0,
            'predicted_rth_seconds': float('inf'),
            'dji_remaining_flight_time': 0.0,  # Will be filled by caller
        }

        if not self.is_active or self.start_time is None:
            return info

        # Get regression data (one point per battery level)
        times, batteries = self._get_regression_data()
        if times is None:
            return info

        try:
            slope, intercept = np.polyfit(times, batteries, 1)
        except:
            return info

        info['slope'] = slope
        info['intercept'] = intercept
        info['drain_rate_per_min'] = -slope * 60  # %/min

        # Use MAX battery needed to go home for conservative prediction
        rth_threshold = self.max_batt_needed_rth + self.RTH_TRIGGER_MARGIN
        info['rth_threshold'] = rth_threshold

        # Current time relative to start
        current_time = time.time() - self.start_time
        info['elapsed_since_monitoring'] = current_time

        if slope >= 0:
            return info  # Battery not draining

        # When does line cross threshold?
        t_rth = (rth_threshold - intercept) / slope
        info['t_rth_absolute'] = t_rth

        # Time until RTH
        time_until_rth = max(0.0, t_rth - current_time)
        info['predicted_rth_seconds'] = time_until_rth

        # Add chart data for visualization (convert to minutes for readability)
        # Battery scatter points: [[time_min, battery], ...]
        chart_battery_points = [[t / 60, b] for t, b in zip(times, batteries)]
        info['chart_battery_points'] = chart_battery_points

        # Regression line: from t=0 to t=t_rth (or current + 5min if t_rth is too far)
        t_end = min(t_rth, current_time + 300) if t_rth > 0 else current_time + 300  # Max 5min projection
        t_start = 0
        regression_line = [
            [t_start / 60, slope * t_start + intercept],
            [t_end / 60, slope * t_end + intercept]
        ]
        info['chart_regression_line'] = regression_line

        # RTH threshold line (horizontal) - using MAX threshold
        threshold_line = [
            [0, rth_threshold],
            [t_end / 60, rth_threshold]
        ]
        info['chart_threshold_line'] = threshold_line

        # RTH crossing point (where regression meets threshold)
        if t_rth > 0:
            info['chart_rth_point'] = [[t_rth / 60, rth_threshold]]
        else:
            info['chart_rth_point'] = []

        # Current time vertical marker (from battery level to 0)
        current_time_min = current_time / 60
        info['chart_current_time_line'] = [
            [current_time_min, 0],
            [current_time_min, 100]
        ]

        # Current position marker (where we are now on the chart)
        info['chart_current_point'] = [[current_time_min, current_battery]]

        return info
