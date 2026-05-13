"""
LoRa Data Handler Module - v2.3 with Battery Voltage
Uses STM32 tx_timestamp_ms for plotting timeline
Parses 16-field packets with battery voltage
"""

import time
from PyQt5.QtCore import QObject, pyqtSignal, QTimer
from SX127x.LoRa import LoRa
from SX127x.constants import MODE, BW, CODING_RATE
from SX127x.board_config import BOARD

# Flag bit definitions - MUST MATCH main.c EXACTLY
FLAG_GO_FOR_LAUNCH  = (1 << 0)  # 0x01 - Bit 0
FLAG_ASCENSION      = (1 << 1)  # 0x02 - Bit 1
FLAG_DROP           = (1 << 2)  # 0x04 - Bit 2
FLAG_RECOVERY       = (1 << 3)  # 0x08 - Bit 3


class LoRaDataReceiver(QObject):
    """
    LoRa receiver that emits Qt signals when data is received
    Integrates LoRa hardware with Qt GUI using signal/slot mechanism
    """
    # Signal emitted when new telemetry data is received (dict of sensor values)
    data_received = pyqtSignal(dict)
    # Signal emitted on connection status change
    connection_status = pyqtSignal(bool, str)
    
    def __init__(self, verbose=False):
        # Initialize QObject parent
        super().__init__()
        
        # Configure GPIO and LoRa module
        BOARD.setup()
        BOARD.SpiDev()
        
        # Create LoRa receiver instance
        self.lora = LoRaReceiver(verbose=verbose)
        self.lora.data_callback = self.on_lora_data_received
        
        self.running = False
        
    def configure_and_start(self):
        """Configure LoRa parameters and start receiving"""
        try:
            # Check hardware version
            version = self.lora.get_version()
            print(f"SX1276 Hardware Version: {hex(version)}")
            
            if version != 0x12:
                self.connection_status.emit(False, "Cannot communicate with SX1276. Check SPI wiring!")
                return False
            
            # Configure LoRa parameters (matching transmitter settings)
            self.lora.configure()
            
            # Emit success status
            self.connection_status.emit(True, "LoRa receiver configured and ready")
            
            # Start receiving in continuous mode
            self.lora.set_mode(MODE.RXCONT)
            self.running = True
            
            print("LoRa receiver started - waiting for packets...")
            print("Protocol v2.3: GO_FOR_LAUNCH → ASCENSION → DROP → RECOVERY")
            print("Using tx_timestamp_ms for plot timeline + Battery Voltage monitoring")
            return True
            
        except Exception as e:
            self.connection_status.emit(False, f"LoRa configuration error: {str(e)}")
            return False
            
    def on_lora_data_received(self, data_dict):
        """
        Callback when LoRa data is received
        Emits Qt signal with parsed data
        """
        if data_dict:
            self.data_received.emit(data_dict)
            
    def stop(self):
        """Stop LoRa receiver"""
        self.running = False
        if hasattr(self, 'lora'):
            self.lora.set_mode(MODE.SLEEP)
        BOARD.teardown()
        self.connection_status.emit(False, "LoRa receiver stopped")


class LoRaReceiver(LoRa):
    """
    Custom LoRa receiver class that extends SX127x.LoRa
    Handles data reception and parsing for satellite telemetry with calibration packet
    """
    
    def __init__(self, verbose=False):
        # Call parent constructor
        super(LoRaReceiver, self).__init__(verbose)
        
        # Put module to Sleep for safe initialization
        self.set_mode(MODE.SLEEP)
        # Map DIO0 to RxDone interrupt
        self.set_dio_mapping([0,0,0,0,0,0])
        
        # Callback function for received data
        self.data_callback = None
        
    def configure(self):
        """Configure LoRa parameters to match transmitter settings"""
        print("Configuring LoRa parameters...")
        self.set_mode(MODE.STDBY)

        # 1. FIFO Management: Dedicate the full 256 bytes to RX
        self.set_fifo_rx_base_addr(0x00)
        self.set_fifo_addr_ptr(0x00)
        
        # 2. PHY parameters (match transmitter configuration)
        self.set_freq(869.53)           # 869.53 MHz
        self.set_spreading_factor(7)   # SF10
        self.set_bw(BW.BW125)           # 125 kHz Bandwidth
        self.set_coding_rate(CODING_RATE.CR4_8) # Max error correction
        self.set_preamble(8)            # Preamble length
        self.set_implicit_header_mode(False) # Explicit mode

        # 3. Synchronization & Optimization
        self.set_low_data_rate_optim(True)
        self.set_sync_word(0x12)        # Standard Private Sync Word
        self.set_invert_iq(0)           # Standard IQ
        
        # 4. Hardware Gain & Safety
        self.set_ocp_trim(100)          # 100mA TX protection
        self.set_lna(lna_gain=1, lna_boost_hf=0b11) # Max Gain (GAIN.G1 = 1)
        self.set_agc_auto_on(True)      # Auto gain control
        
        print("LoRa configuration complete.")
        
    def on_rx_done(self):
        """
        Called when a packet is received (DIO0 interrupt)
        Parses the payload and calls the data callback
        """
        
        # Capture metrics
        irq_flags = self.get_irq_flags()
        hop_channel = self.get_hop_channel()
        pkt_rssi = self.get_pkt_rssi_value()
        pkt_snr = self.get_pkt_snr_value()
        
        # Adjust SNR if needed
        if pkt_snr > 15:
            pkt_snr = pkt_snr - 64
            
        # Read frequency error
        freq_error = self.get_fei()
        
        # CRC Check
        crc_on_payload = hop_channel['crc_on_payload']
        if crc_on_payload == 1 and irq_flags['crc_error'] == 1:
            print(f"CRC Error! RSSI: {pkt_rssi} dBm | SNR: {pkt_snr} dB")
            self.clear_irq_flags(RxDone=1, PayloadCrcError=1)
            return
        
        # Clear interrupt flags
        self.clear_irq_flags(RxDone=1, PayloadCrcError=1)
        
        # Read payload
        payload = self.read_payload(nocheck=True)
        
        if payload:
            # Filter: only include printable ASCII characters
            data_str = ''.join([chr(c) for c in payload if 31 < c < 127])
            
            # Print received packet header
            print(f"\n{'='*80}")
            print(f"[RX] {data_str.strip()}")
            print(f"RSSI: {pkt_rssi} dBm | SNR: {pkt_snr} dB | Freq Error: {freq_error} Hz")
            
            # Parse the data
            parsed_data = self.parse_packet(data_str)
            
            # Add RSSI and SNR to parsed data
            if parsed_data:
                parsed_data['rssi'] = pkt_rssi
                parsed_data['snr'] = pkt_snr
            
            # Print parsed data with labels
            if parsed_data:
                self.print_parsed_data(parsed_data)
            
            print(f"{'='*80}\n")
            
            if parsed_data and self.data_callback:
                self.data_callback(parsed_data)
        
        # Reset for next packet
        self.set_mode(MODE.SLEEP)
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        
    def parse_packet(self, packet_str):
        """
        Parse incoming packet - handles both calibration and telemetry packets
        
        Args:
            packet_str (str): Raw packet string from LoRa
        Returns:
            dict: Parsed data or None if parsing fails
        """
        try:
            # Check if this is a calibration packet
            if packet_str.startswith('CAL,'):
                return self.parse_calibration_packet(packet_str)
            else:
                return self.parse_telemetry_packet(packet_str)
                
        except (ValueError, IndexError) as e:
            print(f"Parse error: {e}")
            return None
    
    def parse_calibration_packet(self, packet_str):
        """
        Parse calibration packet
        Format: "CAL,<calibration_time_ms>,<timestamp_ms>"
        
        Args:
            packet_str (str): Calibration packet string
        Returns:
            dict: Parsed calibration data
        """
        values = [x.strip() for x in packet_str.split(',')]
        
        if len(values) >= 3 and values[0] == 'CAL':
            calibration_time_ms = int(values[1])
            tx_timestamp_ms = int(values[2])
            
            # Print calibration info
            print(f"\n{'*'*80}")
            print(f"🎯 CALIBRATION PACKET RECEIVED")
            print(f"{'*'*80}")
            print(f"Calibration Duration: {calibration_time_ms} ms ({calibration_time_ms/1000:.2f} seconds)")
            print(f"TX Timestamp: {tx_timestamp_ms} ms")
            print(f"{'*'*80}\n")
            
            return {
                'packet_type': 'calibration',
                'calibration_time_ms': calibration_time_ms,
                'calibration_time_sec': calibration_time_ms / 1000.0,
                'tx_timestamp_ms': tx_timestamp_ms,
                'timestamp': tx_timestamp_ms / 1000.0,  # Use STM32 time in seconds
                # Set GO_FOR_LAUNCH flag active after calibration
                'flag_go_for_launch': True,   # → "GO FOR LAUNCH" panel
                'flag_ascension': False,      # → "ASCENSION" panel
                'flag_drop': False,           # → "DROP" panel
                'flag_recovery': False        # → "RECOVERY" panel
            }
        else:
            print("Invalid calibration packet format")
            return None
    
    def print_parsed_data(self, data):
        """
        Print parsed telemetry data with labeled fields
        
        Args:
            data (dict): Parsed telemetry data
        """
        if data.get('packet_type') == 'calibration':
            # Calibration packet already printed in parse_calibration_packet
            return
        
        print(f"\n📊 PARSED TELEMETRY DATA:")
        print(f"{'─'*80}")
        
        # Accelerometer
        print(f"📏 Accel:  X={data.get('accel_x', 0):>7.2f} m/s²  |  "
              f"Y={data.get('accel_y', 0):>7.2f} m/s²  |  "
              f"Z={data.get('accel_z', 0):>7.2f} m/s²")
        
        # Gyroscope
        print(f"🔄 Gyro:   X={data.get('gyro_x', 0):>7.2f} °/s   |  "
              f"Y={data.get('gyro_y', 0):>7.2f} °/s   |  "
              f"Z={data.get('gyro_z', 0):>7.2f} °/s")
        
        # Orientation
        print(f"🧭 Orient: Roll={data.get('roll', 0):>6.1f}°  |  "
              f"Pitch={data.get('pitch', 0):>6.1f}°  |  "
              f"Yaw={data.get('yaw', 0):>6.1f}°")
        
        # Environment
        print(f"🌡️  Temp: {data.get('temperature', 0):>5.1f}°C  |  "
              f"📍 Alt: {data.get('altitude', 0):>7.1f} m")
        
        # GPS with satellite count
        satellites = data.get('satellites', 0)
        sat_icon = "🛰️" if satellites >= 4 else "⚠️"  # 4+ sats = good GPS fix
        print(f"{sat_icon}  GPS:   Lat={data.get('latitude', 0):>10.6f}°  |  "
              f"Lon={data.get('longitude', 0):>10.6f}°  |  "
              f"Sats={satellites}")
        
        # Timing
        print(f"⏱️  Time:  {data.get('tx_timestamp_ms', 0):>6} ms  "
              f"({data.get('timestamp', 0):.3f} s)")
        
        # Battery
        battery_voltage = data.get('battery_voltage', None)
        if battery_voltage is not None:
            print(f"🔋 Battery: {battery_voltage:.2f} V")
        
        # Flags
        flags = data.get('flags_raw', 0)
        flag_str = []
        if data.get('flag_go_for_launch'):
            flag_str.append("GO_FOR_LAUNCH")
        if data.get('flag_ascension'):
            flag_str.append("ASCENSION")
        if data.get('flag_drop'):
            flag_str.append("DROP")
        if data.get('flag_recovery'):
            flag_str.append("RECOVERY")
        
        if flag_str:
            print(f"🚩 Flags:  {' | '.join(flag_str)} (0x{flags:02X})")
        else:
            print(f"🚩 Flags:  None (0x{flags:02X})")
        
        print(f"{'─'*80}")
    
    def parse_telemetry_packet(self, packet_str):
        """
        Parse telemetry packet with timestamp, satellite count, and battery voltage (v2.3)
        Format: accel_x,...,longitude,satellites,flags,timestamp,battery_voltage
        
        Args:
            packet_str (str): Telemetry packet string
        Returns:
            dict: Parsed telemetry data with battery voltage and GPS satellites
        """
        values = [x.strip() for x in packet_str.split(',')]
        
        # v2.3: Expecting 17 values (13 sensor + satellites + flags + timestamp + battery_voltage)
        if len(values) >= 17:
            # Parse sensor values
            data = {
                'packet_type': 'telemetry',
                'accel_x': float(values[0]),
                'accel_y': float(values[1]),
                'accel_z': float(values[2]),
                'gyro_x': float(values[3]),
                'gyro_y': float(values[4]),
                'gyro_z': float(values[5]),
                'roll': float(values[6]),
                'pitch': float(values[7]),
                'yaw': float(values[8]),
                'temperature': float(values[9]),
                'altitude': float(values[10]),
                'latitude': float(values[11]),
                'longitude': float(values[12])
            }
            
            # Parse GPS satellite count (NEW!)
            satellites = int(values[13])
            data['satellites'] = satellites
            
            # Parse flags byte
            flags = int(values[14])
            data['flags_raw'] = flags
            
            # Parse TX timestamp (from STM32)
            tx_timestamp_ms = int(values[15])
            data['tx_timestamp_ms'] = tx_timestamp_ms
            
            # Parse battery voltage
            battery_voltage = float(values[16])
            data['battery_voltage'] = battery_voltage
            
            # Use STM32 timestamp for plotting timeline
            data['timestamp'] = tx_timestamp_ms / 1000.0
            
            # Decode individual flags matching main.c bit definitions
            data['flag_go_for_launch'] = bool(flags & FLAG_GO_FOR_LAUNCH)  # Bit 0 → "GO FOR LAUNCH"
            data['flag_ascension'] = bool(flags & FLAG_ASCENSION)          # Bit 1 → "ASCENSION"
            data['flag_drop'] = bool(flags & FLAG_DROP)                    # Bit 2 → "DROP"
            data['flag_recovery'] = bool(flags & FLAG_RECOVERY)            # Bit 3 → "RECOVERY"
            
            return data
        
        elif len(values) >= 16:
            # v2.3 old: Backwards compatibility - 16 values (no satellite count)
            print("Warning: Received v2.3 packet without satellite count (16 values)")
            # Parse without satellite count - set default value
            data = {
                'packet_type': 'telemetry',
                'accel_x': float(values[0]),
                'accel_y': float(values[1]),
                'accel_z': float(values[2]),
                'gyro_x': float(values[3]),
                'gyro_y': float(values[4]),
                'gyro_z': float(values[5]),
                'roll': float(values[6]),
                'pitch': float(values[7]),
                'yaw': float(values[8]),
                'temperature': float(values[9]),
                'altitude': float(values[10]),
                'latitude': float(values[11]),
                'longitude': float(values[12]),
                'satellites': 0,  # Not available in old format
                'flags_raw': int(values[13]),
                'tx_timestamp_ms': int(values[14]),
                'battery_voltage': float(values[15]),
                'timestamp': int(values[14]) / 1000.0
            }
            
            # Decode flags
            flags = data['flags_raw']
            data['flag_go_for_launch'] = bool(flags & FLAG_GO_FOR_LAUNCH)
            data['flag_ascension'] = bool(flags & FLAG_ASCENSION)
            data['flag_drop'] = bool(flags & FLAG_DROP)
            data['flag_recovery'] = bool(flags & FLAG_RECOVERY)
            
            return data
        
        elif len(values) >= 15:
            # v2.2: Backwards compatibility - 15 values (no battery voltage)
            print("Warning: Received v2.2 packet without battery voltage (15 values)")
            return None
            
        elif len(values) >= 14:
            # v2.1: Backwards compatibility - 14 values (no timestamp)
            print("Warning: Received v2.1 packet without timestamp (14 values)")
            return None
            
        elif len(values) >= 13:
            # v1.0: Old format - 13 values (no flags, no timestamp)
            print("Warning: Received v1.0 packet (13 values)")
            return None
            
        else:
            print(f"Incomplete data: expected 17 values, got {len(values)}")
            return None