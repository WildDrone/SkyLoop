"""
RTH Predictor - Predicts when DJI will trigger Return-To-Home based on battery drain.

This tool analyzes battery drain rate and the evolving batteryNeededToGoHome value
to predict when RTH will be automatically triggered.

Key insight from flight data:
- RTH triggers when: battery <= batteryNeededToGoHome + 2 (approximately)

Authors: Edouard G.A. Rolland
"""

import time
import sys
import os
import glob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from datetime import datetime
import pandas as pd

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from djiInterface import DJIInterface

# Configuration
MAX_POINTS = 600  # 10 minutes at 1Hz
UPDATE_INTERVAL = 1000  # milliseconds
RTH_TRIGGER_MARGIN = 2  # RTH triggers when battery <= batt_needed_rth + margin


class RTHPredictor:
    """Real-time RTH prediction based on battery drain and DJI's batteryNeededToGoHome."""
    
    def __init__(self, dji_interface):
        self.dji = dji_interface
        
        # Data storage
        self.timestamps = deque(maxlen=MAX_POINTS)
        self.battery_level = deque(maxlen=MAX_POINTS)
        self.batt_needed_rth = deque(maxlen=MAX_POINTS)
        self.batt_margin = deque(maxlen=MAX_POINTS)  # battery - batt_needed_rth
        self.distance_to_home = deque(maxlen=MAX_POINTS)
        self.altitude = deque(maxlen=MAX_POINTS)
        self.flight_mode = deque(maxlen=MAX_POINTS)
        
        # Predictions
        self.predicted_rth_time = deque(maxlen=MAX_POINTS)  # seconds until RTH
        self.battery_drain_rate = deque(maxlen=MAX_POINTS)  # %/second
        self.batt_needed_rth_rate = deque(maxlen=MAX_POINTS)  # rate of change of batt_needed_rth
        self.margin_rate = deque(maxlen=MAX_POINTS)  # rate of margin closing
        
        # RTH trigger tracking
        self.rth_triggered = False
        self.rth_trigger_time = None
        self.prediction_at_trigger = None
        
        self.start_time = time.time()
        
    def calculate_drain_rate(self):
        """Calculate battery drain rate in %/second using ALL points (linear regression)."""
        if len(self.timestamps) < 2:
            return 0.0
        
        # Use ALL data points for regression
        times = np.array(list(self.timestamps))
        batteries = np.array(list(self.battery_level))
        
        # Linear regression to get drain rate
        try:
            slope, intercept = np.polyfit(times, batteries, 1)
            return -slope  # Negative slope = positive drain rate
        except:
            return 0.0
    
    def calculate_batt_needed_rate(self):
        """Calculate rate of change of batteryNeededToGoHome in %/second using ALL points."""
        if len(self.timestamps) < 2:
            return 0.0
        
        # Use ALL data points for regression
        times = np.array(list(self.timestamps))
        batt_needed = np.array(list(self.batt_needed_rth))
        
        try:
            slope, intercept = np.polyfit(times, batt_needed, 1)
            return slope  # Rate of change (can be positive or negative)
        except:
            return 0.0
    
    def calculate_margin_rate(self):
        """Calculate rate of margin (battery - batt_needed_rth) change using ALL points."""
        if len(self.timestamps) < 2:
            return 0.0
        
        # Use ALL data points for regression
        times = np.array(list(self.timestamps))
        margins = np.array(list(self.batt_margin))
        
        try:
            slope, intercept = np.polyfit(times, margins, 1)
            return -slope  # Negative slope = margin closing (positive rate)
        except:
            return 0.0
    
    def predict_rth_time(self):
        """
        Predict time until RTH is triggered.
        
        Simple approach:
        1. Linear regression on ALL battery points to get drain rate
        2. Use CURRENT batt_needed_rth + 2 as the trigger threshold
        3. Find when battery line crosses the threshold
        
        RTH triggers when: battery <= batt_needed_rth + RTH_TRIGGER_MARGIN
        
        battery(t) = battery_0 + slope * t
        We want: battery(t_rth) = current_threshold
        t_rth = (current_threshold - battery_0) / slope
        time_until_rth = t_rth - current_time
        """
        if len(self.battery_level) < 2:
            return float('inf')
        
        # Get all data for regression
        times = np.array(list(self.timestamps))
        batteries = np.array(list(self.battery_level))
        
        # Linear regression on ALL points: battery = a*t + b
        try:
            slope, intercept = np.polyfit(times, batteries, 1)
        except:
            return float('inf')
        
        # slope should be negative (battery draining)
        if slope >= 0:
            return float('inf')  # Battery not draining
        
        # Current RTH threshold
        current_batt_needed = self.batt_needed_rth[-1]
        rth_threshold = current_batt_needed + RTH_TRIGGER_MARGIN
        
        # Current time
        current_time = times[-1]
        
        # Find when battery line crosses threshold
        # battery(t) = slope * t + intercept = rth_threshold
        # t_rth = (rth_threshold - intercept) / slope
        t_rth = (rth_threshold - intercept) / slope
        
        # Time until RTH
        time_until_rth = t_rth - current_time
        
        return max(0, time_until_rth)
    
    def get_battery_prediction_line(self):
        """Get the linear regression coefficients for battery prediction."""
        if len(self.battery_level) < 2:
            return None, None
        
        times = np.array(list(self.timestamps))
        batteries = np.array(list(self.battery_level))
        
        try:
            slope, intercept = np.polyfit(times, batteries, 1)
            return slope, intercept
        except:
            return None, None
    
    def update_data(self):
        """Fetch new telemetry data and update predictions."""
        telemetry = self.dji.getTelemetry()
        if not telemetry:
            return False
        
        elapsed = time.time() - self.start_time
        self.timestamps.append(elapsed)
        
        # Get current values
        battery = self.dji.getBatteryLevel()
        batt_rth = self.dji.getBatteryNeededToGoHome()
        distance = self.dji.getDistanceToHome()
        location = self.dji.getLocation()
        alt = location.get('altitude', 0) if location else 0
        mode = self.dji.getFlightMode()
        
        self.battery_level.append(battery)
        self.batt_needed_rth.append(batt_rth)
        self.batt_margin.append(battery - batt_rth)
        self.distance_to_home.append(distance)
        self.altitude.append(alt)
        self.flight_mode.append(mode)
        
        # Calculate rates and predictions
        drain_rate = self.calculate_drain_rate()
        batt_needed_rate = self.calculate_batt_needed_rate()
        margin_rate = self.calculate_margin_rate()
        predicted_time = self.predict_rth_time()
        
        self.battery_drain_rate.append(drain_rate)
        self.batt_needed_rth_rate.append(batt_needed_rate)
        self.margin_rate.append(margin_rate)
        self.predicted_rth_time.append(predicted_time)
        
        # Detect RTH trigger
        if not self.rth_triggered and 'GO_HOME' in str(mode).upper():
            self.rth_triggered = True
            self.rth_trigger_time = elapsed
            # Get prediction from a few seconds ago (before mode change)
            if len(self.predicted_rth_time) > 5:
                self.prediction_at_trigger = self.predicted_rth_time[-5]
            print(f"\n{'='*60}")
            print(f"RTH TRIGGERED at {elapsed:.1f}s!")
            print(f"  Battery: {battery}%")
            print(f"  Battery needed for RTH: {batt_rth}%")
            print(f"  Margin was: {battery - batt_rth}%")
            print(f"  Prediction was: {self.prediction_at_trigger:.1f}s before trigger" if self.prediction_at_trigger else "  No prediction available")
            print(f"{'='*60}\n")
        
        return True
    
    def create_plot(self):
        """Create the real-time prediction plot."""
        self.fig, axes = plt.subplots(3, 2, figsize=(16, 12))
        self.fig.suptitle('RTH Predictor - Real-time Battery & RTH Analysis', fontsize=14)
        
        # Plot 1: Battery levels and RTH threshold
        self.ax1 = axes[0, 0]
        self.ax1.set_ylabel('Battery %')
        self.ax1.set_ylim(0, 100)
        self.ax1.set_title('Battery vs RTH Threshold')
        self.ax1.grid(True, alpha=0.3)
        
        # Plot 2: Battery Margin (battery - batt_needed_rth)
        self.ax2 = axes[0, 1]
        self.ax2.set_ylabel('Margin %')
        self.ax2.set_title('Battery Margin (Battery - Needed for RTH)')
        self.ax2.axhline(y=RTH_TRIGGER_MARGIN, color='r', linestyle='--', label=f'RTH Trigger ({RTH_TRIGGER_MARGIN}%)')
        self.ax2.grid(True, alpha=0.3)
        
        # Plot 3: Predicted time until RTH
        self.ax3 = axes[1, 0]
        self.ax3.set_ylabel('Seconds')
        self.ax3.set_title('Predicted Time Until RTH')
        self.ax3.grid(True, alpha=0.3)
        
        # Plot 4: Drain rates
        self.ax4 = axes[1, 1]
        self.ax4.set_ylabel('%/second')
        self.ax4.set_title('Rate Analysis')
        self.ax4.grid(True, alpha=0.3)
        
        # Plot 5: Distance and Altitude
        self.ax5 = axes[2, 0]
        self.ax5.set_ylabel('Distance (m) / Altitude (m)')
        self.ax5.set_xlabel('Time (seconds)')
        self.ax5.set_title('Distance to Home & Altitude')
        self.ax5.grid(True, alpha=0.3)
        
        # Plot 6: Prediction info text
        self.ax6 = axes[2, 1]
        self.ax6.axis('off')
        self.ax6.set_title('Current Status')
        
        # Initialize lines
        self.lines = {}
        
        # Battery plot
        self.lines['battery'], = self.ax1.plot([], [], 'b-', linewidth=2, label='Battery Level')
        self.lines['batt_rth'], = self.ax1.plot([], [], 'r--', linewidth=2, label='Battery Needed RTH')
        self.lines['rth_threshold'], = self.ax1.plot([], [], 'r-', linewidth=1, alpha=0.5, label=f'RTH Trigger (+{RTH_TRIGGER_MARGIN}%)')
        self.ax1.legend(loc='upper right')
        
        # Margin plot
        self.lines['margin'], = self.ax2.plot([], [], 'g-', linewidth=2, label='Current Margin')
        self.ax2.legend(loc='upper right')
        
        # Prediction plot
        self.lines['prediction'], = self.ax3.plot([], [], 'purple', linewidth=2, label='Time to RTH')
        self.ax3.legend(loc='upper right')
        
        # Rates plot
        self.lines['drain_rate'], = self.ax4.plot([], [], 'b-', linewidth=2, label='Battery Drain Rate')
        self.lines['batt_needed_rate'], = self.ax4.plot([], [], 'orange', linewidth=2, label='Batt Needed RTH Rate')
        self.lines['margin_rate'], = self.ax4.plot([], [], 'green', linewidth=2, linestyle='--', label='Margin Closing Rate')
        self.ax4.legend(loc='upper right')
        
        # Distance/Altitude plot
        self.lines['distance'], = self.ax5.plot([], [], 'b-', linewidth=2, label='Distance to Home')
        self.lines['altitude'], = self.ax5.plot([], [], 'g-', linewidth=1.5, label='Altitude')
        self.ax5.legend(loc='upper right')
        
        # Status text
        self.status_text = self.ax6.text(0.1, 0.9, '', transform=self.ax6.transAxes,
                                          fontsize=12, verticalalignment='top', fontfamily='monospace',
                                          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.tight_layout()
        return self.fig
    
    def animate(self, frame):
        """Animation update function."""
        if not self.update_data():
            return list(self.lines.values())
        
        if len(self.timestamps) < 2:
            return list(self.lines.values())
        
        t = list(self.timestamps)
        
        # Update battery plot
        self.lines['battery'].set_data(t, list(self.battery_level))
        self.lines['batt_rth'].set_data(t, list(self.batt_needed_rth))
        rth_threshold = [b + RTH_TRIGGER_MARGIN for b in self.batt_needed_rth]
        self.lines['rth_threshold'].set_data(t, rth_threshold)
        
        # Update margin plot
        self.lines['margin'].set_data(t, list(self.batt_margin))
        
        # Update prediction plot (cap at 600 for display)
        predictions = [min(p, 600) for p in self.predicted_rth_time]
        self.lines['prediction'].set_data(t, predictions)
        
        # Update rates plot
        self.lines['drain_rate'].set_data(t, list(self.battery_drain_rate))
        self.lines['batt_needed_rate'].set_data(t, list(self.batt_needed_rth_rate))
        self.lines['margin_rate'].set_data(t, list(self.margin_rate))
        
        # Update distance/altitude plot
        self.lines['distance'].set_data(t, list(self.distance_to_home))
        self.lines['altitude'].set_data(t, list(self.altitude))
        
        # Update axes limits
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5]:
            ax.set_xlim(max(0, t[-1] - 300), t[-1] + 10)
            ax.relim()
            ax.autoscale_view(scalex=False)
        
        # Update status text
        if self.battery_level and self.batt_needed_rth:
            current_battery = self.battery_level[-1]
            current_batt_rth = self.batt_needed_rth[-1]
            current_margin = current_battery - current_batt_rth
            drain_rate = self.battery_drain_rate[-1] if self.battery_drain_rate else 0
            batt_rate = self.batt_needed_rth_rate[-1] if self.batt_needed_rth_rate else 0
            m_rate = self.margin_rate[-1] if self.margin_rate else 0
            prediction = self.predicted_rth_time[-1] if self.predicted_rth_time else float('inf')
            mode = self.flight_mode[-1] if self.flight_mode else 'N/A'
            distance = self.distance_to_home[-1] if self.distance_to_home else 0
            
            # Format prediction time
            if prediction == float('inf') or prediction > 3600:
                pred_str = "> 60 min"
            elif prediction > 60:
                pred_str = f"{prediction/60:.1f} min"
            else:
                pred_str = f"{prediction:.0f} sec"
            
            # Status color based on urgency
            if prediction < 30:
                status_color = 'red'
                urgency = "⚠️ IMMINENT RTH!"
            elif prediction < 60:
                status_color = 'orange'
                urgency = "⚠️ RTH SOON"
            elif prediction < 120:
                status_color = 'yellow'
                urgency = "⚡ Getting close"
            else:
                status_color = 'green'
                urgency = "✓ OK"
            
            status = f"""
CURRENT STATUS
{'='*35}
Flight Mode:     {mode}
Battery:         {current_battery}%
Needed for RTH:  {current_batt_rth}%
Margin:          {current_margin}% (trigger at {RTH_TRIGGER_MARGIN}%)
Distance Home:   {distance:.1f}m

RATES (per minute)
{'='*35}
Battery Drain:   {drain_rate*60:.2f}%/min
RTH Need Change: {batt_rate*60:+.2f}%/min
Margin Closing:  {m_rate*60:.2f}%/min

PREDICTION
{'='*35}
Time to RTH:     {pred_str}
Status:          {urgency}

{'RTH TRIGGERED!' if self.rth_triggered else ''}
"""
            self.status_text.set_text(status)
            
            # Update status box color
            if self.rth_triggered:
                self.status_text.set_bbox(dict(boxstyle='round', facecolor='red', alpha=0.8))
            elif prediction < 60:
                self.status_text.set_bbox(dict(boxstyle='round', facecolor='orange', alpha=0.8))
            else:
                self.status_text.set_bbox(dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
        
        return list(self.lines.values())
    
    def run(self):
        """Start the real-time prediction."""
        print("Starting RTH Predictor...")
        print("This tool predicts when RTH will be triggered based on battery drain.")
        print("Press Ctrl+C to stop.\n")
        
        self.create_plot()
        
        ani = animation.FuncAnimation(
            self.fig, self.animate, interval=UPDATE_INTERVAL, blit=False, cache_frame_data=False
        )
        
        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.save_results()
    
    def save_results(self):
        """Save prediction results."""
        if len(self.timestamps) == 0:
            print("\nNo data collected.")
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"rth_prediction_{timestamp}.txt"
        
        with open(filename, 'w') as f:
            f.write("=" * 70 + "\n")
            f.write(f"RTH PREDICTION REPORT - {timestamp}\n")
            f.write("=" * 70 + "\n\n")
            
            f.write("FLIGHT SUMMARY\n")
            f.write("-" * 40 + "\n")
            duration = self.timestamps[-1] if self.timestamps else 0
            f.write(f"  Duration:              {duration:.1f}s ({duration/60:.1f} min)\n")
            f.write(f"  Start battery:         {self.battery_level[0] if self.battery_level else 'N/A'}%\n")
            f.write(f"  End battery:           {self.battery_level[-1] if self.battery_level else 'N/A'}%\n")
            f.write(f"  RTH Triggered:         {'Yes' if self.rth_triggered else 'No'}\n")
            if self.rth_triggered:
                f.write(f"  RTH Trigger Time:      {self.rth_trigger_time:.1f}s\n")
                if self.prediction_at_trigger:
                    f.write(f"  Prediction Error:      {self.prediction_at_trigger:.1f}s before actual\n")
            f.write("\n")
            
            f.write("DRAIN RATE ANALYSIS\n")
            f.write("-" * 40 + "\n")
            if self.battery_drain_rate:
                avg_drain = np.mean(list(self.battery_drain_rate)) * 60
                f.write(f"  Avg battery drain:     {avg_drain:.2f}%/min\n")
            if self.batt_needed_rth_rate:
                avg_batt_rate = np.mean(list(self.batt_needed_rth_rate)) * 60
                f.write(f"  Avg RTH need change:   {avg_batt_rate:+.2f}%/min\n")
            f.write("\n")
            
            f.write("=" * 70 + "\n")
        
        print(f"\nResults saved to: {filename}")


def analyze_historical_data(show_plots=True):
    """Analyze historical CSV files to validate prediction model."""
    csv_files = sorted(glob.glob("rth_analysis_*.csv"))
    
    if not csv_files:
        print("No historical data files found.")
        return
    
    print(f"Found {len(csv_files)} historical flight files.\n")
    
    # Collect all flights with RTH events for comparison plot
    rth_flights = []
    
    for csv_file in csv_files:
        print(f"\n{'='*60}")
        print(f"Analyzing: {csv_file}")
        print("="*60)
        
        df = pd.read_csv(csv_file)
        
        # Find RTH trigger point
        rth_idx = None
        for i in range(1, len(df)):
            if 'GO_HOME' in str(df['flight_mode'].iloc[i]).upper() and 'GO_HOME' not in str(df['flight_mode'].iloc[i-1]).upper():
                rth_idx = i
                break
        
        if rth_idx is None:
            print("  No RTH event found in this flight.")
            continue
        
        # Get values at RTH trigger
        rth_time = df['time'].iloc[rth_idx]
        rth_battery = df['battery'].iloc[rth_idx]
        rth_batt_needed = df['batt_needed_rth'].iloc[rth_idx]
        margin_at_rth = rth_battery - rth_batt_needed
        
        # Store for comparison
        rth_flights.append({
            'file': csv_file,
            'df': df,
            'rth_idx': rth_idx,
            'rth_time': rth_time,
            'rth_battery': rth_battery,
            'rth_batt_needed': rth_batt_needed,
            'margin_at_rth': margin_at_rth
        })
        
        print(f"  RTH triggered at:      {rth_time:.1f}s")
        print(f"  Battery at RTH:        {rth_battery}%")
        print(f"  Battery needed RTH:    {rth_batt_needed}%")
        print(f"  Margin at RTH:         {margin_at_rth}%")
        
        # RTH threshold
        df['rth_threshold'] = df['batt_needed_rth'] + RTH_TRIGGER_MARGIN
        
        # Simple prediction: linear regression on ALL battery points up to current time
        print("\n  Prediction accuracy (linear regression on all battery points):")
        
        for lookback in [120, 60, 30, 15, 10, 5]:
            if rth_idx > lookback:
                # Get data at lookback point
                idx = rth_idx - lookback
                t = df['time'].iloc[idx]
                
                # Linear regression on ALL battery points up to this moment
                times = df['time'].iloc[:idx+1].values
                batteries = df['battery'].iloc[:idx+1].values
                
                if len(times) > 1:
                    slope, intercept = np.polyfit(times, batteries, 1)
                    
                    # Current RTH threshold at this point
                    current_threshold = df['rth_threshold'].iloc[idx]
                    
                    # Find when battery line crosses threshold
                    # battery(t) = slope * t + intercept = threshold
                    if slope < 0:  # Battery is draining
                        t_rth_predicted = (current_threshold - intercept) / slope
                        predicted_time = t_rth_predicted - t
                        actual_time = rth_time - t
                        error = predicted_time - actual_time
                        accuracy = (1 - abs(error) / actual_time) * 100 if actual_time > 0 else 0
                        drain_rate = -slope * 60  # %/min
                        print(f"    {lookback}s before RTH: Predicted {predicted_time:.1f}s, Actual {actual_time:.1f}s, Error {error:+.1f}s ({accuracy:.0f}% accurate) [drain: {drain_rate:.2f}%/min]")
    
    # Generate plots if we have RTH flights
    if show_plots and rth_flights:
        plot_historical_analysis(rth_flights)


def plot_historical_analysis(rth_flights):
    """Generate comprehensive plots for historical flight analysis."""
    
    n_flights = len(rth_flights)
    
    # Create a figure for each flight
    for i, flight in enumerate(rth_flights):
        df = flight['df']
        rth_idx = flight['rth_idx']
        rth_time = flight['rth_time']
        
        # RTH threshold
        df['rth_threshold'] = df['batt_needed_rth'] + RTH_TRIGGER_MARGIN
        
        # Calculate rolling predictions using simple linear regression on ALL points
        predictions = []
        regression_lines = []  # Store (slope, intercept) at each point
        
        for idx in range(len(df)):
            if idx < 5:
                predictions.append(np.nan)
                regression_lines.append((None, None))
                continue
            
            # Linear regression on ALL battery points up to this moment
            times = df['time'].iloc[:idx+1].values
            batteries = df['battery'].iloc[:idx+1].values
            
            try:
                slope, intercept = np.polyfit(times, batteries, 1)
                regression_lines.append((slope, intercept))
                
                if slope < 0:  # Battery is draining
                    current_threshold = df['rth_threshold'].iloc[idx]
                    current_time = df['time'].iloc[idx]
                    # Find when battery line crosses threshold
                    t_rth_predicted = (current_threshold - intercept) / slope
                    time_until_rth = t_rth_predicted - current_time
                    predictions.append(max(0, time_until_rth))
                else:
                    predictions.append(np.nan)
            except:
                predictions.append(np.nan)
                regression_lines.append((None, None))
        
        df['predicted_time_to_rth'] = predictions
        
        # Create figure with subplots
        fig, axes = plt.subplots(2, 2, figsize=(16, 10))
        fig.suptitle(f'Flight Analysis: {flight["file"]}\nRTH at {rth_time:.1f}s, Battery: {flight["rth_battery"]}%', fontsize=14)
        
        # Plot 1: Battery vs RTH Threshold with regression line
        ax1 = axes[0, 0]
        ax1.plot(df['time'], df['battery'], 'b-', linewidth=2, label='Battery Level')
        ax1.plot(df['time'], df['rth_threshold'], 'r-', linewidth=2, label=f'RTH Threshold (needed + {RTH_TRIGGER_MARGIN}%)')
        ax1.axvline(x=rth_time, color='green', linestyle=':', linewidth=2, label='RTH Triggered')
        
        # Add regression line at a few key points
        for check_idx in [rth_idx - 60, rth_idx - 30, rth_idx - 10]:
            if check_idx > 5 and check_idx < len(regression_lines):
                slope, intercept = regression_lines[check_idx]
                if slope is not None and slope < 0:
                    # Extend line to show predicted intersection
                    t_start = df['time'].iloc[0]
                    t_end = rth_time + 30
                    line_times = np.array([t_start, t_end])
                    line_batteries = slope * line_times + intercept
                    ax1.plot(line_times, line_batteries, '--', alpha=0.5, linewidth=1)
        
        # Show final regression line
        if rth_idx > 5:
            slope, intercept = np.polyfit(df['time'].iloc[:rth_idx].values, df['battery'].iloc[:rth_idx].values, 1)
            t_start = df['time'].iloc[0]
            t_end = rth_time + 50
            line_times = np.array([t_start, t_end])
            line_batteries = slope * line_times + intercept
            ax1.plot(line_times, line_batteries, 'b--', alpha=0.7, linewidth=2, label=f'Battery regression (slope: {slope*60:.2f}%/min)')
        
        ax1.set_ylabel('Battery %')
        ax1.set_xlabel('Time (s)')
        ax1.set_title('Battery Level vs RTH Threshold')
        ax1.legend(loc='upper right')
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(0, 100)
        
        # Plot 2: Predicted vs Actual time to RTH
        ax2 = axes[0, 1]
        df['actual_time_to_rth'] = rth_time - df['time']
        df.loc[df['time'] > rth_time, 'actual_time_to_rth'] = 0
        
        ax2.plot(df['time'], df['actual_time_to_rth'], 'b-', linewidth=2, label='Actual Time to RTH')
        ax2.plot(df['time'], df['predicted_time_to_rth'], 'orange', linewidth=2, label='Predicted Time to RTH')
        ax2.axvline(x=rth_time, color='green', linestyle=':', linewidth=2, label='RTH Triggered')
        ax2.set_ylabel('Seconds')
        ax2.set_xlabel('Time (s)')
        ax2.set_title('Predicted vs Actual Time to RTH')
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.3)
        ax2.set_ylim(0, min(500, rth_time * 1.2))
        
        # Plot 3: Prediction Error
        ax3 = axes[1, 0]
        df['prediction_error'] = df['predicted_time_to_rth'] - df['actual_time_to_rth']
        ax3.plot(df['time'], df['prediction_error'], 'purple', linewidth=2, label='Prediction Error')
        ax3.axhline(y=0, color='gray', linestyle='-', linewidth=1)
        ax3.axvline(x=rth_time, color='green', linestyle=':', linewidth=2, label='RTH Triggered')
        ax3.fill_between(df['time'], 0, df['prediction_error'], alpha=0.3, color='purple')
        ax3.set_ylabel('Error (seconds)')
        ax3.set_xlabel('Time (s)')
        ax3.set_title('Prediction Error (Predicted - Actual)')
        ax3.legend(loc='upper right')
        ax3.grid(True, alpha=0.3)
        
        # Plot 4: Battery drain rate evolution
        ax4 = axes[1, 1]
        drain_rates = []
        for idx in range(len(df)):
            if idx < 10:
                drain_rates.append(np.nan)
                continue
            times = df['time'].iloc[:idx+1].values
            batteries = df['battery'].iloc[:idx+1].values
            try:
                slope, _ = np.polyfit(times, batteries, 1)
                drain_rates.append(-slope * 60)  # Convert to %/min
            except:
                drain_rates.append(np.nan)
        
        df['drain_rate'] = drain_rates
        ax4.plot(df['time'], df['drain_rate'], 'b-', linewidth=2, label='Battery Drain Rate')
        ax4.axvline(x=rth_time, color='green', linestyle=':', linewidth=2, label='RTH Triggered')
        ax4.set_ylabel('Drain Rate (%/min)')
        ax4.set_xlabel('Time (s)')
        ax4.set_title('Battery Drain Rate (from cumulative regression)')
        ax4.legend(loc='upper right')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save the figure
        plot_filename = flight['file'].replace('.csv', '_analysis.png')
        plt.savefig(plot_filename, dpi=150, bbox_inches='tight')
        print(f"\nPlot saved: {plot_filename}")
    
    # Create comparison plot if multiple flights
    if n_flights > 1:
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle(f'RTH Analysis Comparison ({n_flights} flights)', fontsize=14)
        
        colors = plt.cm.tab10(np.linspace(0, 1, n_flights))
        
        # Plot 1: Battery at RTH trigger
        ax1 = axes[0, 0]
        for i, flight in enumerate(rth_flights):
            ax1.bar(i, flight['rth_battery'], color=colors[i], alpha=0.7, label=f"Flight {i+1}")
            ax1.bar(i, flight['rth_batt_needed'], color=colors[i], alpha=0.3)
        ax1.set_xticks(range(n_flights))
        ax1.set_xticklabels([f"Flight {i+1}" for i in range(n_flights)])
        ax1.set_ylabel('Battery %')
        ax1.set_title('Battery at RTH Trigger (dark) vs Needed (light)')
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Margin at RTH
        ax2 = axes[0, 1]
        margins = [f['margin_at_rth'] for f in rth_flights]
        ax2.bar(range(n_flights), margins, color=colors, alpha=0.7)
        ax2.axhline(y=np.mean(margins), color='red', linestyle='--', label=f'Avg: {np.mean(margins):.1f}%')
        ax2.set_xticks(range(n_flights))
        ax2.set_xticklabels([f"Flight {i+1}" for i in range(n_flights)])
        ax2.set_ylabel('Margin %')
        ax2.set_title('Margin at RTH Trigger')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Margin evolution aligned to RTH
        ax3 = axes[1, 0]
        for i, flight in enumerate(rth_flights):
            df = flight['df']
            df['margin'] = df['battery'] - df['batt_needed_rth']
            # Align time so RTH is at 0
            time_to_rth = flight['rth_time'] - df['time']
            ax3.plot(time_to_rth, df['margin'], color=colors[i], linewidth=2, label=f"Flight {i+1}")
        ax3.axhline(y=RTH_TRIGGER_MARGIN, color='red', linestyle='--', linewidth=2, label=f'RTH Trigger ({RTH_TRIGGER_MARGIN}%)')
        ax3.axvline(x=0, color='green', linestyle=':', linewidth=2, label='RTH Event')
        ax3.set_xlabel('Time to RTH (seconds, negative = before RTH)')
        ax3.set_ylabel('Margin %')
        ax3.set_title('Margin Evolution Aligned to RTH Event')
        ax3.legend(loc='upper left')
        ax3.set_xlim(-120, 30)
        ax3.grid(True, alpha=0.3)
        
        # Plot 4: RTH trigger times
        ax4 = axes[1, 1]
        rth_times = [f['rth_time'] for f in rth_flights]
        ax4.bar(range(n_flights), rth_times, color=colors, alpha=0.7)
        ax4.set_xticks(range(n_flights))
        ax4.set_xticklabels([f"Flight {i+1}" for i in range(n_flights)])
        ax4.set_ylabel('Time (seconds)')
        ax4.set_title('Time to RTH Trigger')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('rth_comparison.png', dpi=150, bbox_inches='tight')
        print(f"\nComparison plot saved: rth_comparison.png")
    
    plt.show()


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--analyze":
        # Analyze historical data
        analyze_historical_data(show_plots=True)
        return
    
    IP_RC = "10.184.11.117"  # Default IP
    
    if len(sys.argv) > 1:
        IP_RC = sys.argv[1]
    
    print(f"Connecting to drone at {IP_RC}...")
    dji = DJIInterface(IP_RC)
    
    print("Starting telemetry stream...")
    dji.startTelemetryStream()
    
    time.sleep(2)
    
    if not dji.getTelemetry():
        print("Warning: No telemetry data received. Make sure the drone is connected.")
    
    predictor = RTHPredictor(dji)
    
    try:
        predictor.run()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        dji.stopTelemetryStream()
        print("Done.")


if __name__ == '__main__':
    main()
