"""
Main entry point for the Satellite Telemetry GUI Application
Launches main window with LoRa receiver on Raspberry Pi
"""

import sys
import os

# Force venv site-packages to take priority over system packages.
# This ensures the venv's pyqtgraph 0.14.0 is used instead of the
# system's 0.13.1 which causes a drawLines TypeError with this PyQt5 version.
import site
venv_site = os.path.join(os.path.dirname(os.path.dirname(sys.executable)), 'lib',
                         f'python{sys.version_info.major}.{sys.version_info.minor}',
                         'site-packages')
if venv_site not in sys.path:
    sys.path.insert(0, venv_site)

from PyQt5.QtWidgets import QApplication, QMessageBox

from gui_main_window import SatelliteGUI

def main():
    """
    Main function to start the application
    Directly launches the main telemetry window with LoRa receiver
    """
    app = QApplication(sys.argv)
    app.setApplicationName("Satellite Telemetry Monitor - LoRa")
    
    # Create and show main window directly (no serial config needed)
    try:
        main_window = SatelliteGUI()
        main_window.show()
        
        # Start the event loop
        sys.exit(app.exec_())
    except Exception as e:
        QMessageBox.critical(None, "Startup Error", 
                           f"Failed to start application:\n{str(e)}\n\n"
                           "Make sure you're running on Raspberry Pi with LoRa module connected.")
        sys.exit(1)

if __name__ == "__main__":
    main()