"""
From ChatGPT -- Byte logic
"""

bit7 = 1
bit6 = 0
bits5to0 = 54

# Ensure valid ranges
bit7 &= 0b1
bit6 &= 0b1
bits5to0 &= 0b111111

# Construct the byte
byte_value = (bit7 << 7) | (bit6 << 6) | bits5to0

# Convert to a single byte (Big Endian style doesn't affect a single byte, but use for compatibility)
byte_string = byte_value.to_bytes(1, byteorder='big')

print(f"Byte value: {byte_value:08b}")
print(f"Byte string: {type(byte_string)}")