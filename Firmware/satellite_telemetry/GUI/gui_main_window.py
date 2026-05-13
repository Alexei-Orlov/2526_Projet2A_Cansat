"""
Main GUI Window Module - v2.2 CORRECTED
Processes flight phase flags from LoRa telemetry
Flag mapping: GO FOR LAUNCH, ASCENSION, DROP, RECOVERY
"""

from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                            QPushButton, QStatusBar, QMessageBox, QSplitter, QFileDialog)
from PyQt5.QtCore import Qt
from lora_data_handler import LoRaDataReceiver
from data_plotter import DataPlotter
from satellite_3d import Satellite3DView
from map_view import MapView
from datetime import datetime

class SatelliteGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        self.lora_receiver = LoRaDataReceiver(verbose=False)
        self.lora_receiver.data_received.connect(self.on_data_received)
        self.lora_receiver.connection_status.connect(self.on_connection_status)
        
        self.init_ui()
        self.start_lora_receiver()
        
    def init_ui(self):
        self.setWindowTitle("Satellite Telemetry - Flight Phases v2.2")
        self.setGeometry(50, 50, 1800, 1000)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        main_splitter = QSplitter(Qt.Horizontal)
        left_splitter = QSplitter(Qt.Vertical)
        
        self.satellite_3d = Satellite3DView()
        left_splitter.addWidget(self.satellite_3d)
        
        self.map_view = MapView()
        left_splitter.addWidget(self.map_view)
        
        left_splitter.setSizes([400, 400])
        main_splitter.addWidget(left_splitter)
        
        self.data_plotter = DataPlotter()
        main_splitter.addWidget(self.data_plotter)
        
        main_splitter.setSizes([600, 1200])
        main_layout.addWidget(main_splitter)
        
        button_layout = QHBoxLayout()
        
        self.stop_btn = QPushButton("Stop LoRa Receiver")
        self.stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                font-weight: bold;
                padding: 6px 15px;
            }
            QPushButton:hover { background-color: #da190b; }
        """)
        self.stop_btn.clicked.connect(self.stop_and_close)
        button_layout.addWidget(self.stop_btn)
        
        self.clear_btn = QPushButton("Clear Data")
        self.clear_btn.setStyleSheet("padding: 6px 15px;")
        self.clear_btn.clicked.connect(self.clear_all_data)
        button_layout.addWidget(self.clear_btn)
        
        self.export_btn = QPushButton("Export to CSV")
        self.export_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                font-weight: bold;
                padding: 6px 15px;
            }
            QPushButton:hover { background-color: #0b7dda; }
        """)
        self.export_btn.clicked.connect(self.export_csv)
        button_layout.addWidget(self.export_btn)
        
        button_layout.addStretch()
        
        self.status_indicator = QPushButton("● Initializing...")
        self.status_indicator.setEnabled(False)
        self.status_indicator.setStyleSheet("""
            QPushButton {
                background-color: #ff9800;
                color: white;
                font-weight: bold;
                border: none;
                padding: 6px 15px;
            }
        """)
        button_layout.addWidget(self.status_indicator)
        
        main_layout.addLayout(button_layout)
        
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Initializing...")
        
    def start_lora_receiver(self):
        success = self.lora_receiver.configure_and_start()
        if not success:
            QMessageBox.critical(self, "LoRa Error", "Failed to start LoRa receiver")
            self.close()
            
    def stop_and_close(self):
        reply = QMessageBox.question(self, "Stop", "Stop LoRa and close?", 
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.cleanup()
            self.close()
            
    def on_data_received(self, data):
        """
        Process telemetry including flight phase flags
        
        CORRECTED Flag mapping matching main.c:
        ==========================================
        Bit 0 (FLAG_GO_FOR_LAUNCH) → flag_go_for_launch → "GO FOR LAUNCH" panel
        Bit 1 (FLAG_ASCENSION)     → flag_ascension     → "ASCENSION" panel
        Bit 2 (FLAG_DROP)          → flag_drop          → "DROP" panel
        Bit 3 (FLAG_RECOVERY)      → flag_recovery      → "RECOVERY" panel
        
        data_plotter.py expects these set_flag() calls:
        - set_flag('calibration', ...) → "GO FOR LAUNCH"
        - set_flag('drop', ...)        → "ASCENSION"
        - set_flag('parachute', ...)   → "DROP"
        - set_flag('landing', ...)     → "RECOVERY"
        """
        # Update plots with telemetry data
        self.data_plotter.update_data(data)
        
        # Update 3D satellite orientation
        self.satellite_3d.update_orientation(
            data['roll'], 
            data['pitch'], 
            data['yaw']
        )
        
        # Update map position
        self.map_view.update_position(
            data.get('latitude', 0), 
            data.get('longitude', 0)
        )
        
        # CORRECTED: Update flight phase flags with proper key mapping
        # The lora_data_handler parses flags into: flag_go_for_launch, flag_ascension, flag_drop, flag_recovery
        # But data_plotter expects: 'calibration', 'drop', 'parachute', 'landing'
        # So we map them correctly:
        
        if 'flag_go_for_launch' in data:
            # Bit 0: GO_FOR_LAUNCH → 'calibration' → "GO FOR LAUNCH" panel
            self.data_plotter.set_flag('calibration', data['flag_go_for_launch'])
        
        if 'flag_ascension' in data:
            # Bit 1: ASCENSION → 'drop' → "ASCENSION" panel
            self.data_plotter.set_flag('drop', data['flag_ascension'])
        
        if 'flag_drop' in data:
            # Bit 2: DROP → 'parachute' → "DROP" panel
            self.data_plotter.set_flag('parachute', data['flag_drop'])
        
        if 'flag_recovery' in data:
            # Bit 3: RECOVERY → 'landing' → "RECOVERY" panel
            self.data_plotter.set_flag('landing', data['flag_recovery'])
        
    def on_connection_status(self, connected, message):
        self.status_bar.showMessage(message)
        
        if connected:
            self.status_indicator.setText("● LoRa Active")
            self.status_indicator.setStyleSheet("""
                QPushButton {
                    background-color: #4CAF50;
                    color: white;
                    font-weight: bold;
                    border: none;
                    padding: 6px 15px;
                }
            """)
        else:
            self.status_indicator.setText("● LoRa Inactive")
            self.status_indicator.setStyleSheet("""
                QPushButton {
                    background-color: #f44336;
                    color: white;
                    font-weight: bold;
                    border: none;
                    padding: 6px 15px;
                }
            """)
            
    def clear_all_data(self):
        reply = QMessageBox.question(self, "Clear", "Clear all data?", 
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.data_plotter.clear_data()
    
    def export_csv(self):
        """Export telemetry data to CSV file"""
        # Generate default filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f"telemetry_data_{timestamp}.csv"
        
        # Open file dialog
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Export Telemetry Data",
            default_filename,
            "CSV Files (*.csv);;All Files (*)"
        )
        
        if filename:
            # Ensure .csv extension
            if not filename.endswith('.csv'):
                filename += '.csv'
            
            # Call export method from data_plotter
            success, message = self.data_plotter.export_to_csv(filename)
            
            if success:
                QMessageBox.information(self, "Export Successful", message)
                self.status_bar.showMessage(f"Exported to {filename}")
            else:
                QMessageBox.warning(self, "Export Failed", message)
                self.status_bar.showMessage("Export failed")
    
    def cleanup(self):
        print("Cleaning up...")
        self.lora_receiver.stop()
        if hasattr(self, 'map_view'):
            self.map_view.cleanup()
            
    def closeEvent(self, event):
        self.cleanup()
        event.accept()