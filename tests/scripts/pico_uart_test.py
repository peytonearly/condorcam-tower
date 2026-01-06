import serial
import time

# === Configuration === #
PORT = "/dev/serial0"  # Use the Pi's UART port
BAUD = 115200
TIMEOUT = 0.2  # Seconds

# === Connection === #
try:
    ser = serial.Serial(PORT, baudrate=BAUD, timeout=TIMEOUT)
    print(f"Connected to {PORT} at {BAUD} baud")
except serial.SerialException as e:
    print(f"Failed to connect: {e}")
    exit(1)
    
time.sleep(0.5)  # Brief pause for stability

# === Test Commands === #
def send_command(cmd):
    ser.reset_input_buffer()
    ser.write(f"{cmd}\n".encode("utf-8"))
    time.sleep(0.05)
    resp = ser.readline().decode("utf-8").strip()
    return resp

print("Testing Pico communication...")
print("Sending '1' to read position:")
resp = send_command(1)
print(f"Response: {resp!r}")

time.sleep(0.5)

print("Sending '2' to reset position:")
resp = send_command(2)
print(f"Response: {resp!r}")

time.sleep(0.5)

print("Sending invalid command '99':")
resp = send_command(99)
print(f"Response: {resp!r}")

ser.close()
print("UART test complete.")