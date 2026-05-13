"""
Data Plotting Module - OPTIMIZED with Battery Monitoring
Handles real-time plotting of sensor data using PyQtGraph

OPTIMIZATIONS:
- Use numpy arrays instead of deques (faster array operations)
- Pre-allocate arrays for better memory performance
- Batch plot updates instead of updating each curve separately
- Reduced style update overhead

NEW in v2.3:
- Battery voltage to percentage conversion
- Battery status display with color coding
- Altitude-based vertical speed calculation (no drift!)
"""

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout
from PyQt5.QtCore import Qt
import pyqtgraph as pg
import numpy as np

class DataPlotter(QWidget):
    """
    OPTIMIZED Widget for displaying real-time plots of sensor data with battery monitoring
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Pre-allocate numpy arrays instead of deques
        self.max_points = 800
        self.current_index = 0
        self.data_count = 0
        
        # Pre-allocated arrays (much faster than deques)
        self.time_data = np.zeros(self.max_points, dtype=np.float32)
        self.vertical_speed = np.zeros(self.max_points, dtype=np.float32)
        self.temperature = np.zeros(self.max_points, dtype=np.float32)
        self.altitude = np.zeros(self.max_points, dtype=np.float32)
        
        # For altitude-based vertical speed calculation
        self.last_altitude = None
        self.current_vertical_speed = 0.0
        self.calibration_done = False
        self.drop_detected = False
        self.parachute_deployed = False
        self.landing_detected = False
        self.start_time = None
        self.last_timestamp = None
        
        # Battery voltage smoothing (moving average)
        self.battery_buffer_size = 12  # Number of samples to average
        self.battery_buffer = []  # Circular buffer for battery voltages
        
        # CSV Export: Store all received data packets
        self.csv_data = []  # List of dictionaries containing all telemetry data
        
        # Battery voltage to percentage lookup table (2S LiPo)
        # Based on the image provided
        self.battery_table = [
            (8.40, 100),  # 100%
            (8.40, 90),   # 90%
            (7.94, 80),   # 80%
            (7.84, 70),   # 70%
            (7.74, 60),   # 60%
            (7.66, 50),   # 50%
            (7.58, 40),   # 40%
            (7.50, 30),   # 30%
            (7.40, 20),   # 20%
            (7.20, 10),   # 10%
            (6.60, 5),    # 5%
            (6.00, 0)     # 0%
        ]
        
        self.init_ui()
        
    def voltage_to_percentage(self, voltage):
        """
        Convert battery voltage to percentage based on 2S LiPo discharge curve
        
        Args:
            voltage (float): Battery voltage in Volts
        Returns:
            int: Battery percentage (0-100)
        """
        if voltage is None:
            return None
        
        # Handle out of range voltages
        if voltage >= 8.40:
            return 100
        if voltage <= 6.00:
            return 0
        
        # Linear interpolation between points in the lookup table
        for i in range(len(self.battery_table) - 1):
            v_high, pct_high = self.battery_table[i]
            v_low, pct_low = self.battery_table[i + 1]
            
            if voltage >= v_low and voltage <= v_high:
                # Linear interpolation
                pct = pct_low + (voltage - v_low) * (pct_high - pct_low) / (v_high - v_low)
                return int(pct)
        
        return 0
    
    def get_battery_color(self, percentage):
        """
        Get color based on battery percentage
        
        Args:
            percentage (int): Battery percentage (0-100)
        Returns:
            str: Color string for stylesheet
        """
        if percentage is None:
            return "#888888"  # Gray for unknown
        elif percentage >= 80:
            return "#27ae60"  # Green (good)
        elif percentage >= 50:
            return "#f39c12"  # Yellow (moderate)
        elif percentage >= 20:
            return "#e67e22"  # Orange (low)
        else:
            return "#e74c3c"  # Red (critical)
    
    def get_gps_color(self, satellites):
        """
        Get color based on GPS satellite count
        
        Args:
            satellites (int): Number of visible GPS satellites
        Returns:
            str: Color string for stylesheet
        """
        if satellites is None or satellites == 0:
            return "#e74c3c"  # Red (no GPS)
        elif satellites >= 6:
            return "#27ae60"  # Green (excellent fix - 6+ sats)
        elif satellites >= 4:
            return "#f39c12"  # Yellow (good fix - 4-5 sats)
        else:
            return "#e67e22"  # Orange (poor fix - 1-3 sats)
    
    def get_smoothed_battery_voltage(self, new_voltage):
        """
        Calculate moving average of battery voltage to reduce ADC noise
        
        Args:
            new_voltage (float): New battery voltage reading
        Returns:
            float: Smoothed battery voltage (average of last 12 readings)
        """
        # Add new voltage to buffer
        self.battery_buffer.append(new_voltage)
        
        # Keep only last N samples (circular buffer)
        if len(self.battery_buffer) > self.battery_buffer_size:
            self.battery_buffer.pop(0)
        
        # Calculate and return mean
        return sum(self.battery_buffer) / len(self.battery_buffer)
    
    def export_to_csv(self, filename):
        """
        Export all received telemetry data to CSV file
        
        Args:
            filename (str): Path to CSV file
        Returns:
            tuple: (success: bool, message: str)
        """
        if len(self.csv_data) == 0:
            return (False, "No data to export")
        
        try:
            import csv
            
            # Define CSV headers (all fields from telemetry)
            headers = [
                'relative_time',      # Time since start (s)
                'timestamp',          # Absolute timestamp (s)
                'tx_timestamp_ms',    # Raw STM32 timestamp (ms)
                'accel_x',            # m/s²
                'accel_y',
                'accel_z',
                'gyro_x',             # °/s
                'gyro_y',
                'gyro_z',
                'roll',               # °
                'pitch',
                'yaw',
                'temperature',        # °C
                'altitude',           # m
                'vertical_speed',     # m/s (calculated)
                'latitude',           # °
                'longitude',          # °
                'satellites',         # count
                'flags_raw',          # binary (use flags_raw, not flags)
                'battery_voltage',    # V
                'rssi',               # dBm
                'snr'                 # dB
            ]
            
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=headers, extrasaction='ignore')
                writer.writeheader()
                
                for data_point in self.csv_data:
                    # Write only the fields that exist in headers
                    row = {key: data_point.get(key, '') for key in headers}
                    writer.writerow(row)
            
            return (True, f"Exported {len(self.csv_data)} data points to {filename}")
            
        except Exception as e:
            return (False, f"Export failed: {str(e)}")
        
    def init_ui(self):
        """Initialize the plotting interface with PyQtGraph"""
        layout = QVBoxLayout()
        layout.setSpacing(5)
        layout.setContentsMargins(5, 5, 5, 5)
        
        pg.setConfigOption('background', 'w')
        pg.setConfigOption('foreground', 'k')
        pg.setConfigOption('antialias', False)  # Disable for better performance
        
        # Create plots with optimized settings
        self.plot_vspeed = pg.PlotWidget(title="Vertical Speed (from Altitude)")
        self.plot_vspeed.showGrid(x=True, y=True, alpha=0.3)
        self.plot_vspeed.setLabel('left', 'Vertical Speed', units='m/s')
        self.plot_vspeed.setLabel('bottom', 'Time', units='s')
        self.plot_vspeed.setDownsampling(auto=True, mode='peak')  # Enable downsampling
        self.curve_vspeed = self.plot_vspeed.plot(pen=pg.mkPen('#2E86AB', width=2))
        layout.addWidget(self.plot_vspeed)
        
        self.plot_temp = pg.PlotWidget(title="Temperature")
        self.plot_temp.showGrid(x=True, y=True, alpha=0.3)
        self.plot_temp.setLabel('left', 'Temperature', units='°C')
        self.plot_temp.setLabel('bottom', 'Time', units='s')
        self.plot_temp.setDownsampling(auto=True, mode='peak')
        self.curve_temp = self.plot_temp.plot(pen=pg.mkPen('#FF8C00', width=2))
        layout.addWidget(self.plot_temp)
        
        self.plot_alt = pg.PlotWidget(title="Altitude")
        self.plot_alt.showGrid(x=True, y=True, alpha=0.3)
        self.plot_alt.setLabel('left', 'Altitude', units='m')
        self.plot_alt.setLabel('bottom', 'Time', units='s')
        self.plot_alt.setDownsampling(auto=True, mode='peak')
        self.curve_alt = self.plot_alt.plot(pen=pg.mkPen('#9400D3', width=2))
        layout.addWidget(self.plot_alt)
        
        info_panel = self.create_info_panel()
        layout.addWidget(info_panel)
        self.setLayout(layout)
        
    def create_info_panel(self):
        panel = QWidget()
        panel.setStyleSheet("QWidget { background-color: #f5f5f5; border-radius: 5px; }")
        layout = QVBoxLayout()
        layout.setContentsMargins(10, 10, 10, 10)
        
        title = QLabel("📊 TELEMETRY & FLIGHT PHASES")
        title.setStyleSheet("font-size: 14px; font-weight: bold; color: #333; background: none;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # First row: Speed, Gyro, Temp, Alt
        telemetry_layout1 = QHBoxLayout()
        self.vspeed_label = QLabel("Vert. Speed: --- m/s")
        self.gyro_label = QLabel("Gyro: --- °/s")
        self.temp_label = QLabel("Temp: --- °C")
        self.alt_label = QLabel("Alt: --- m")
        
        telemetry_style = """
            QLabel {
                padding: 8px 12px;
                background-color: white;
                border: 2px solid #ddd;
                border-radius: 5px;
                font-weight: bold;
                font-size: 11px;
                color: #333;
            }
        """
        self.vspeed_label.setStyleSheet(telemetry_style)
        self.gyro_label.setStyleSheet(telemetry_style)
        self.temp_label.setStyleSheet(telemetry_style)
        self.alt_label.setStyleSheet(telemetry_style)
        
        telemetry_layout1.addWidget(self.vspeed_label)
        telemetry_layout1.addWidget(self.gyro_label)
        telemetry_layout1.addWidget(self.temp_label)
        telemetry_layout1.addWidget(self.alt_label)
        layout.addLayout(telemetry_layout1)
        
        # Second row: Battery and GPS Satellites
        telemetry_layout2 = QHBoxLayout()
        self.battery_label = QLabel("Battery: ---%")
        self.gps_label = QLabel("GPS: --- sats")  # NEW: GPS satellite count
        
        self.battery_label.setStyleSheet(telemetry_style)
        self.gps_label.setStyleSheet(telemetry_style)
        
        telemetry_layout2.addWidget(self.battery_label)
        telemetry_layout2.addWidget(self.gps_label)
        layout.addLayout(telemetry_layout2)
        
        flags_layout = QGridLayout()
        flags_layout.setSpacing(10)
        self.flag_calibration = self.create_flag_label("GO FOR LAUNCH")
        self.flag_drop = self.create_flag_label("ASCENSION")
        self.flag_parachute = self.create_flag_label("DROP")
        self.flag_landing = self.create_flag_label("RECOVERY")
        flags_layout.addWidget(self.flag_calibration, 0, 0)
        flags_layout.addWidget(self.flag_drop, 0, 1)
        flags_layout.addWidget(self.flag_parachute, 1, 0)
        flags_layout.addWidget(self.flag_landing, 1, 1)
        layout.addLayout(flags_layout)
        
        panel.setLayout(layout)
        return panel
        
    def create_flag_label(self, text):
        label = QLabel(text)
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("""
            QLabel {
                background-color: #e74c3c;
                color: white;
                font-weight: bold;
                font-size: 12px;
                padding: 12px;
                border-radius: 5px;
                border: 3px solid #c0392b;
            }
        """)
        return label
        
    def set_flag(self, flag_name, active):
        """OPTIMIZED: Cache flag styles to avoid recreating them"""
        flag_map = {
            'calibration': (self.flag_calibration, 'calibration_done'),
            'drop': (self.flag_drop, 'drop_detected'),
            'parachute': (self.flag_parachute, 'parachute_deployed'),
            'landing': (self.flag_landing, 'landing_detected')
        }
        
        if flag_name.lower() in flag_map:
            label, attr = flag_map[flag_name.lower()]
            old_value = getattr(self, attr, None)
            
            # Only update if state changed
            if old_value != active:
                setattr(self, attr, active)
                if active:
                    label.setStyleSheet("""
                        QLabel {
                            background-color: #27ae60;
                            color: white;
                            font-weight: bold;
                            font-size: 12px;
                            padding: 12px;
                            border-radius: 5px;
                            border: 3px solid #229954;
                        }
                    """)
                else:
                    label.setStyleSheet("""
                        QLabel {
                            background-color: #e74c3c;
                            color: white;
                            font-weight: bold;
                            font-size: 12px;
                            padding: 12px;
                            border-radius: 5px;
                            border: 3px solid #c0392b;
                        }
                    """)
        
    def update_data(self, data):
        """OPTIMIZED: Use numpy array operations with altitude-based vertical speed"""
        if self.start_time is None:
            self.start_time = data['timestamp']
            self.last_timestamp = data['timestamp']
            self.last_altitude = data.get('altitude', 0)
            
        current_time = data['timestamp'] - self.start_time
        current_altitude = data.get('altitude', 0)
        
        # Calculate vertical speed from altitude change (much more reliable!)
        if self.last_altitude is not None and self.last_timestamp is not None:
            dt = data['timestamp'] - self.last_timestamp
            
            if dt > 0:  # Avoid division by zero
                # Vertical speed = change in altitude / change in time
                altitude_change = current_altitude - self.last_altitude
                self.current_vertical_speed = altitude_change / dt
                
                # Optional: Apply smoothing to reduce noise
                # self.current_vertical_speed = 0.7 * self.current_vertical_speed + 0.3 * (altitude_change / dt)
        
        # Store complete data packet for CSV export
        data_copy = data.copy()
        data_copy['vertical_speed'] = self.current_vertical_speed
        data_copy['relative_time'] = current_time
        self.csv_data.append(data_copy)
        
        # Use circular buffer approach with numpy arrays
        idx = self.current_index % self.max_points
        self.time_data[idx] = current_time
        self.vertical_speed[idx] = self.current_vertical_speed
        self.temperature[idx] = data.get('temperature', 0)
        self.altitude[idx] = current_altitude
        
        self.current_index += 1
        self.data_count = min(self.data_count + 1, self.max_points)
        
        # Update for next iteration
        self.last_altitude = current_altitude
        self.last_timestamp = data['timestamp']
        
        # Update plots - now uses pre-allocated arrays
        self.update_plots()
        
        # Update labels - cache gyro magnitude calculation
        gyro_mag = np.sqrt(data['gyro_x']**2 + data['gyro_y']**2 + data['gyro_z']**2)
        
        # Batch text updates to reduce redraws
        self.vspeed_label.setText(f"Vert. Speed: {self.current_vertical_speed:.2f} m/s")
        self.gyro_label.setText(f"Gyro: {gyro_mag:.2f} °/s")
        self.temp_label.setText(f"Temp: {data.get('temperature', 0):.1f} °C")
        self.alt_label.setText(f"Alt: {data.get('altitude', 0):.1f} m")
        
        # Update battery display with color coding (smoothed values)
        battery_voltage = data.get('battery_voltage', None)
        if battery_voltage is not None:
            # Get smoothed voltage (moving average of last 12 readings)
            smoothed_voltage = self.get_smoothed_battery_voltage(battery_voltage)
            
            # Convert smoothed voltage to percentage
            battery_pct = self.voltage_to_percentage(smoothed_voltage)
            battery_color = self.get_battery_color(battery_pct)
            
            self.battery_label.setText(f"Battery: {battery_pct}%")
            self.battery_label.setStyleSheet(f"""
                QLabel {{
                    padding: 8px 12px;
                    background-color: {battery_color};
                    border: 2px solid {battery_color};
                    border-radius: 5px;
                    font-weight: bold;
                    font-size: 11px;
                    color: white;
                }}
            """)
        else:
            self.battery_label.setText("Battery: ---")
            self.battery_label.setStyleSheet("""
                QLabel {
                    padding: 8px 12px;
                    background-color: white;
                    border: 2px solid #ddd;
                    border-radius: 5px;
                    font-weight: bold;
                    font-size: 11px;
                    color: #333;
                }
            """)
        
        # NEW: Update GPS satellite count display with color coding
        satellites = data.get('satellites', None)
        if satellites is not None:
            gps_color = self.get_gps_color(satellites)
            
            self.gps_label.setText(f"GPS: {satellites} sats")
            self.gps_label.setStyleSheet(f"""
                QLabel {{
                    padding: 8px 12px;
                    background-color: {gps_color};
                    border: 2px solid {gps_color};
                    border-radius: 5px;
                    font-weight: bold;
                    font-size: 11px;
                    color: white;
                }}
            """)
        else:
            self.gps_label.setText("GPS: --- sats")
            self.gps_label.setStyleSheet("""
                QLabel {
                    padding: 8px 12px;
                    background-color: white;
                    border: 2px solid #ddd;
                    border-radius: 5px;
                    font-weight: bold;
                    font-size: 11px;
                    color: #333;
                }
            """)
        
    def update_plots(self):
        """OPTIMIZED: Use array slicing instead of converting deques"""
        if self.data_count == 0:
            return
        
        # Get valid data range (circular buffer handling)
        if self.data_count < self.max_points:
            # Haven't filled buffer yet
            time_slice = self.time_data[:self.data_count]
            vspeed_slice = self.vertical_speed[:self.data_count]
            temp_slice = self.temperature[:self.data_count]
            alt_slice = self.altitude[:self.data_count]
        else:
            # Buffer is full, need to reorder
            idx = self.current_index % self.max_points
            time_slice = np.concatenate([self.time_data[idx:], self.time_data[:idx]])
            vspeed_slice = np.concatenate([self.vertical_speed[idx:], self.vertical_speed[:idx]])
            temp_slice = np.concatenate([self.temperature[idx:], self.temperature[:idx]])
            alt_slice = np.concatenate([self.altitude[idx:], self.altitude[:idx]])
        
        # Single setData call per curve (no array copies)
        self.curve_vspeed.setData(time_slice, vspeed_slice)
        self.curve_temp.setData(time_slice, temp_slice)
        self.curve_alt.setData(time_slice, alt_slice)
        
    def clear_data(self):
        """OPTIMIZED: Fast array zeroing"""
        self.time_data.fill(0)
        self.vertical_speed.fill(0)
        self.temperature.fill(0)
        self.altitude.fill(0)
        self.current_index = 0
        self.data_count = 0
        
        self.start_time = None
        self.last_timestamp = None
        self.last_altitude = None
        self.current_vertical_speed = 0.0
        
        # Clear battery smoothing buffer
        self.battery_buffer = []
        
        # Clear CSV export data
        self.csv_data = []
        
        self.set_flag('calibration', False)
        self.set_flag('drop', False)
        self.set_flag('parachute', False)
        self.set_flag('landing', False)
        
        # Clear plots
        self.curve_vspeed.setData([], [])
        self.curve_temp.setData([], [])
        self.curve_alt.setData([], [])
        
        # Reset displays
        self.battery_label.setText("Battery: ---")
        self.gps_label.setText("GPS: --- sats")