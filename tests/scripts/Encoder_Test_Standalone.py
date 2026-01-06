import os
import signal
import pigpio
import logging
from collections import deque
from statistics import median
from main import setup_logging, MicrosecondFormatter

sig_received = False  # Tracks if a signal has been received

class Encoder:
    def __init__(self):
        """
        Class initialization.
        """
        # === Constants === #
        self._encoder_transition_table = {  # Transition table for determining encoder changes
            (1, 0): +1, (3, 1): +1, (2, 3): +1, (0, 2): +1,
            (0, 1): -1, (1, 3): -1, (3, 2): -1, (2, 0): -1
        }

        # === Pi GPIO Pins (Broadcom numbers) === #
        self._pin_encoder_a = 17  # Encoder channel A signal (edge detection)
        self._pin_encoder_b = 27  # Encoder channel B signal (phase shift)
        self._pin_encoder_i = 22  # Encoder index signal

        # === Deques === #
        self.travel_rate_dq = deque([0 for _ in range(7)], maxlen = 7)  # Deque for encoder travel rate

        # === Runtime Variables === #
        self._last_state = None  # Hold A and B states in a 2-bit value
        self.position = 0
        self.travel_dir = None  # Travel direction based on encoder values. 0 for down | 1 for up
        self._last_tick = None  # Tracks the last encoder tick
        self.travel_rate = 0.0  # Encoder counts per second. <0 for moving down | >0 for moving up
        self._pulse_active = False  # Tracks if the index pulse has been encountered

        # === Logger config === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("Encoder class initialized")

        # === Connect Devices === #
        self.connect_devices()

    def connect_devices(self):
        """
        Master function for connecting to GPIO devices.
        """
        # Connect to pigio daemon
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.logger.critical(f"Could not connect to pigpiod. Closing...")
            exit()

        # Run connector function
        self._connect_encoder()

        self.logger.info(f"All devices connected")

    def disconnect_devices(self):
        """
        Close device connections.
        """
        self._cb_encoder_a.cancel()  # Cancel encoder A callback
        self._cb_encoder_b.cancel()  # Cancel encoder B callback
        self._cb_encoder_i.cancel()  # Cancel encoder I callback
        self.pi.stop()  # Close pigpio connection

        self.logger.info(f"All devices disconnected")

    def _connect_encoder(self):
        """
        Connects to the optical encoder.
        """
        # Setup encoder A/B/I pin modes and pull-ups
        for pin in [self._pin_encoder_a, self._pin_encoder_b, self._pin_encoder_i]:
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pigpio.PUD_UP)
            self.pi.set_glitch_filter(pin, 1)  # Ignores edges shorter than 1 µs

        # Register callbacks
        self._cb_encoder_a = self.pi.callback(self._pin_encoder_a, pigpio.EITHER_EDGE, self._handle_encoder_change)
        self._cb_encoder_b = self.pi.callback(self._pin_encoder_b, pigpio.EITHER_EDGE, self._handle_encoder_change)
        self._cb_encoder_i = self.pi.callback(self._pin_encoder_i, pigpio.RISING_EDGE, self._handle_index_pulse)

        # Channel A and B initial state
        a = self.pi.read(self._pin_encoder_a)
        b = self.pi.read(self._pin_encoder_b)
        self._last_state = (a << 1) | b

        self.logger.info(f"Encoder pins and callbacks configured")

    def _handle_encoder_change(self, gpio, level, tick):
        """
        Reads the current encoder value. Triggered when edge is detected on either pin.
        """
        # Read pin states
        a = self.pi.read(self._pin_encoder_a)
        b = self.pi.read(self._pin_encoder_b)
        current_state = (a << 1) | b

        # Determine if state has changed
        delta = self._encoder_transition_table.get((self._last_state, current_state), 0)
        self._last_state = current_state

        if delta:
            # Update position and direction
            self.position += delta
            self.travel_dir = 1 if delta > 0 else 0

            # Compute travel rate
            if self._last_tick is not None:
                dt = pigpio.tickDiff(self._last_tick, tick)
                if dt > 0:
                    self.travel_rate_dq.append(1_000_000 * delta / dt)  # Counts per second
            self._last_tick = tick

    def _handle_index_pulse(self, gpio, level, tick):
        self._pulse_active = True

    # === User Helper Functions === #
    def get_position(self):
        """
        Returns tower position.
        """
        return self.position
    
    def get_travel_dir(self):
        """
        Returns encoder travel direction.
        """
        return self.travel_dir
    
    def get_travel_rate(self):
        """
        Returns the median of the travel rate deque.
        """
        return median(self.travel_rate_dq)
    
    def get_index_pulse(self):
        """
        Handle index pulse logging.
        """
        if self._pulse_active:  # Pulse tripped
            self._pulse_active = False
            return 1
        else:
            return 0
    
    def kill_program(self):
        """
        Gracefully exits the program when an interrupt is received.
        """
        self.disconnect_devices()

    def log_events(self):
        """
        Logs all important values.
        """
        # Log each loop
        self.logger.debug(f"Encoder position: {self.position}")
        self.logger.debug(f"Encoder rate: {self.get_travel_rate()}")
        self.logger.debug(f"Encoder last state: {self._last_state}")
        self.logger.debug(f"Index pulse tripped: {self.get_index_pulse()}")

def signal_handler(signum, frame):
    global sig_received
    sig_received = True
    logging.info(f"Signal {signum} received")

def main():
    # Start logger
    os.makedirs("logs", exist_ok=True)
    setup_logging()

    # Create encoder instance
    encoder = Encoder()

    # Signal interrupt
    global sig_received
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    pos = encoder.get_position()

    try:
        logging.info("System startup")
        while not sig_received:
            pos = encoder.get_position()
            encoder.log_events()
    finally:
        if sig_received:
            logging.warning("Interrupt caught. Closing program.")
        encoder.disconnect_devices()
        logging.info("Clean shutdown complete")

if __name__ == "__main__":
    main()