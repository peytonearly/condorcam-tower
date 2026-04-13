# Python Libraries
import logging
import serial
import time
from collections import deque

class E5_with_Pico_USB:
    def __init__(self) -> None:
        """
        Class initialization.
        """
        # === Constants === #
        # Encoder
        self._encoder_cpr     = 32    # Encoder counts per revolution
        self._encoder_max_rev = 66.5  # Number of revolutions made from lowest to highest point
        # self._encoder_max     = self._encoder_cpr * self._encoder_max_rev  # Encoder max position position (theoretical)
        self._encoder_max     = 8000  # Encoder max position (experimental)
        # === #
        
        # === Serial Connection === #
        self.ser                = None            # Serial connection
        self.encoder_connected  = False           # Indicates connection to the Pico
        self.ser_port           = "/dev/ttyAMC0"  # Serial port
        self.timeout            = 0.25            # Serial read timeout (s)
        self.connect_timeout    = 3.0             # Connection timeout limit (s)
        # === #
        
        # === Deques === #
        size_limit = 7
        self.enc_pos_dq  = deque([0 for _ in range(size_limit)],                      maxlen=size_limit)  # Deque for encoder position
        self.pos_ts_dq   = deque([time.perf_counter_ns() for _ in range(size_limit)], maxlen=size_limit)  # Deque for position timestamps
        self.enc_vel_dq  = deque([0 for _ in range(size_limit)],                      maxlen=size_limit)  # Deque for encoder travel rate
        # === #
        
        # === Logger Config === #
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("E5 encoder via Pico USB class initialized")
        # === #
        
        # === Connect to Encoder === #
        self.connect()
        # === #
        
    # === Connection Management === #
    def connect(self) -> None:
        """
        Attempt to (re)connect to the Pico via USB serial.
        Non-blocking: Will tiemout after connect_timeout seconds.
        """
        start_time = time.time()
        while time.time() - start_time < self.connect_timeout:
            try:
                self.ser = serial.Serial(
                    self.ser_port,
                    baudrate=115200,
                    timeout=self.timeout,
                    write_timeout=self.timeout,
                )
                
                time.sleep(0.5)  # Allow link to settle
                
                self.encoder_connected = True
                self.logger.info(f"USB encoder connected on {self.ser_port} at {time.time():.3f}")
                return
            
            except serial.SerialException:
                time.sleep(0.5)
                
        self.encoder_connected = False
        self.logger.warning("Failed to connect to USB encoder (timeout)")
        
    def disconnect(self) -> None:
        """
        Safely close the serial USB connection to the Pico.
        """
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
            
        self.ser = None
        self.encoder_connected = False
        self.logger.info(f"USB encoder disconnected at {time.time():.3f}")
        
    def reconnect(self) -> bool:
        """
        Attempt to reconnect to the Pico if the port is closed.
        """
        if not self.encoder_connected:
            self.logger.info(f"Attempting USB reconnection at {time.time():.3f}")
            self.connect()
            
        return self.encoder_connected
    # === #
    
    # === Command / Response Handling === #
    def _send_command(self, cmd: int) -> None:
        """
        Send a numeric command to the Pico and capture the response.
        """
        if not self.encoder_connected:
            self.response = None
            return
        
        try:
            _ = self.ser.read_all()
            
            # Write the command
            time.sleep(0.05)                              # Allow previous cycle to finish
            self.ser.write(f"{cmd}\r\n".encode("utf-8"))
            self.ser.flush()                              # Force TX buffer to send immediately
            time.sleep(0.02)                              # Small pacing delay
            
            # Capture response
            self.response = self.ser.readline().decode("utf-8", errors="ignore").strip()
            
        except serial.SerialException:
            self.logger.warning("USB serial exception during write. Disconnecting...")
            self.response = None
            self.disconnect()
    # === #
    
    # === Encoder Commands === #
    def _reset_pico_position(self) -> None:
        """
        Send command 2: Reset encoder position to zero.
        """
        if not self.encoder_connected:
            return None
        
        self._send_command(2)
    
        for _ in range(len(self.enc_pos_dq)): self.enc_pos_dq.append(0)
    
    def _get_position(self) -> None:
        """
        Send command 1: Read current position.
        """
        if not self.encoder_connected:
            return
        
        self._send_command(1)
        
        if not self.response:
            # If no response, just skip this cycle - don't drop connection
            self.logger.debug("No response from Pico this cycle. Skipping...")
            return
        
        try:
            pos = int(self.response)
            ts  = time.perf_counter_ns()
            self.enc_pos_dq.append(pos)
            self.pos_ts_dq.append(ts)
        except ValueError:
            self.logger.warning(f"Invalid response: {self.response!r}")
    # === #
    
    # === Derived Calculations === #
    def _calculate_velocity(self) -> None:
        """
        Compute velocity (cnt/s)
        """
        if len(self.enc_pos_dq) < 2:
            return
        
        v = (self.enc_pos_dq[-1] - self.enc_pos_dq[-2]) / ((self.pos_ts_dq[-1] - self.pos_ts_dq[-2]) / 1_000_000_000)
        
        self.enc_vel_dq.append(v)
    # === #
    
    # === Public Interface === #
    def handle_zero_button_tripped(self, value: bool | int) -> None:
        """
        Zero position when triggered by Tower class.
        """
        if value:  # Ensure value is True
            self.logger.info("Zero button notification received. Position reset requested")
            self._reset_pico_position()
            
            if self.response:
                self.logger.info("Encoder position reset")
            else:
                self.logger.warning("Attempted position reset but Pico did not respond")
                
    def get_position(self) -> int:
        """
        Returns current tower position.
        """
        self._get_position()
        return self.enc_pos_dq[-1]
    
    def get_velocity(self) -> float:
        """
        Returns the current encoder velocity.
        """
        self._get_position()
        self._calculate_velocity()
        return self.enc_vel_dq[-1]
    
    def get_average_velocity(self) -> float:
        """
        Returns the average encoder velocity.
        """
        self._get_position()
        self._calculate_velocity()
        return sum(self.enc_vel_dq) / len(self.enc_vel_dq)
    
    def get_encoder_max(self) -> int | float:
        """
        Returns the max encoder value.
        """
        return self._encoder_max
    
    def get_encoder_connection(self) -> bool:
        """
        Returns encoder connection status.
        """
        return self.encoder_connected
    
    def set_encoder_max(self, enc_max: int) -> None:
        """
        Allows external programs to set the max encoder position.
        """
        self._encoder_max = enc_max
        
    def set_zero_position(self) -> None:
        """
        Allows external programs to set the encoder position to zero.
        
        Only use when zero button is deemed inoperable.
        """
        self.logger.warning("Encoder position manually reset")
        self._reset_pico_position()
    # === #
    
    # === Logging === #
    def log_debug_values(self) -> None:
        """
        Logs all debug values.
        """
        self.logger.debug(f"Encoder position: {self.enc_pos_dq[-1]}")
        self.logger.debug(f"Encoder velocity: {self.enc_vel_dq[-1]}")
        self.logger.debug(f"Encoder velocity (average): {sum(self.enc_vel_dq) / len(self.enc_vel_dq)}")
    # === #