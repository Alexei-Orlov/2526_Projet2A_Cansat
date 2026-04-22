import sys 
from time import sleep
import RPi.GPIO as GPIO

# Import pySX127x components
from SX127x.LoRa import *
from SX127x.board_config import BOARD

# Configure RPi GPIO pins (LED, RST, DIOx) via the board_config file
# Configure RPi GPIO pins
BOARD.setup() 

# 1. Initialize the SPI device first
BOARD.SpiDev()

# 2. Now you can safely set the speed
#BOARD.spi.max_speed_hz = 500000  # <--- This is mandatory for stability



class LoRaRcvCont(LoRa):
    def __init__(self, verbose=False):
        # Calls the parent constructor to initialize SPI and GPIO events
        super(LoRaRcvCont, self).__init__(verbose)
        # Put module to Sleep for safe initialization
        self.set_mode(MODE.SLEEP) 
        # Map DIO0 to RxDone interrupt
        self.set_dio_mapping([0,0,0,0,0,0])

    def configure(self):
        print("Configuring LoRa parameters...")
        self.set_mode(MODE.STDBY)

        # 1. FIFO Management: Dedicate the full 256 bytes to RX
        self.set_fifo_rx_base_addr(0x00)
        self.set_fifo_addr_ptr(0x00)
        
        # 2. PHY parameters (Precise match for STM32)
        self.set_freq(869.53)           # 869.53 MHz
        self.set_spreading_factor(10)   # SF10
        self.set_bw(BW.BW125)           # 125 kHz Bandwidth
        self.set_coding_rate(CODING_RATE.CR4_8) # Max error correction
        self.set_preamble(8)            # Preamble length
        self.set_implicit_header_mode(False) # Explicit mode

        # 3. Synchronization & Optimization
        self.set_low_data_rate_optim(True) # Match STM32's 0x00 setting
        self.set_sync_word(0x12)        # Standard Private Sync Word
        self.set_invert_iq(0)           # Standard IQ
        
        # 4. Hardware Gain & Safety
        self.set_ocp_trim(100)          # Match 100mA TX protection
        self.set_lna(lna_gain=GAIN.G1, lna_boost_hf=0b11) # Max Gain
        self.set_agc_auto_on(True)      # Let the chip handle saturation
        print("Configuration complete.")

    def start(self):
        """ Start the continuous receive loop """
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        print("Waiting for incoming LoRa packets...")
        try:
            while True:
                sleep(.5)
        except KeyboardInterrupt:
            pass
     
    def on_rx_done(self):
        # Capture metrics
        irq_flags = self.get_irq_flags()
        hop_channel = self.get_hop_channel()
        pkt_rssi = self.get_pkt_rssi_value()
        pkt_snr = self.get_pkt_snr_value()
        if pkt_snr > 15 :
            pkt_snr = pkt_snr - 64
        # Read the frequency error in Hz
        freq_error = lora.get_fei()
        print(f"Frequency Error: {freq_error} Hz")
        
        # CRC Check logic
        crc_on_payload = hop_channel['crc_on_payload']
        if crc_on_payload == 1 and irq_flags['crc_error'] == 1:
            print(f"CRC Error! RSSI: {pkt_rssi} dBm | SNR: {pkt_snr} dB")
            self.clear_irq_flags(RxDone=1, PayloadCrcError=1)
            return
        
        self.clear_irq_flags(RxDone=1, PayloadCrcError=1)
        payload = self.read_payload(nocheck=True)
        
        if payload:
            # Filter: only include printable ASCII characters (hex 0x20 to 0x7E)
            data = ''.join([chr(c) for c in payload if 31 < c < 127])
            print(f"RECEIVED: {data} | RSSI: {pkt_rssi} dBm | SNR: {pkt_snr} dB")
        
        # Clean reset for next packet
        self.set_mode(MODE.SLEEP)
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)



# --- Execution Flow ---
lora = LoRaRcvCont(verbose=False) # 1. Init
# 3. Check the hardware version to confirm communication
# (This is the best way to see if your wiring is correct)



version = lora.get_version()
print(f"SX1276 Hardware Version: {hex(version)}")

if version != 0x12:
    print("Error: Cannot communicate with SX1276. Check SPI wiring!")
    sys.exit(1)
lora.configure()                  # 2. Setup
try: 
    lora.start()                  # 3. Action
finally:
    lora.set_mode(MODE.SLEEP)
    BOARD.teardown()