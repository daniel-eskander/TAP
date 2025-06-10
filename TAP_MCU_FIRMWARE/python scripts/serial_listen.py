import serial
import struct
import matplotlib.pyplot as plt

START_BYTE = 0xAA
STOP_BYTE = 0xCC
DATA_LENGTH = 1000  # Bytes in the data section
received_values = []

def listen(com_port, baud_rate, num_frames):
    """
    Listens to a serial port and reads data frames of specified size.

    Parameters:
        com_port (str): The serial port to listen on (e.g., 'COM3').
        baud_rate (int): The baud rate for the serial communication.
        num_frames (int): Number of frames to receive.

    Returns:
        list: A list of 16-bit integer values received from the serial data frames.
    """
    try:
        with serial.Serial(port=com_port, baudrate=baud_rate, bytesize=serial.EIGHTBITS,
                           stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE, timeout=1) as ser:
            print(f"Listening on {com_port} at {baud_rate} baud rate.")
            
            for frame in range(num_frames):
                # Synchronize with START_BYTE
                while True:
                    byte = ser.read(1)
                    if len(byte) == 1 and byte[0] == START_BYTE:
                        print(f"Frame {frame + 1}: START_BYTE detected.")
                        break
                
                # Read the 1000-byte data section
                data = ser.read(DATA_LENGTH)
                if len(data) != DATA_LENGTH:
                    print("Error: Incomplete data section.")
                    continue
                
                # Read and validate STOP_BYTE
                stop_byte = ser.read(1)
                if len(stop_byte) != 1:
                    print("Error: Missing STOP_BYTE.")
                    continue
                
                # Print the actual received frame for inspection
                frame_bytes = [START_BYTE] + list(data) + [stop_byte[0]]
                frame_bytes_hex = [f"0x{byte:02X}" for byte in frame_bytes]  # Format as 2-digit hex
                print(f"Frame {frame + 1} (101 bytes in hex): {frame_bytes_hex}")
                                
                # Check if the received STOP_BYTE matches the expected STOP_BYTE
                if stop_byte[0] != STOP_BYTE:
                    print(f"Warning: STOP_BYTE mismatch in Frame {frame + 1}. Received: {stop_byte[0]}")
                
                # Process every other 16-bit value from the 1000-byte data section
                for i in range(0, DATA_LENGTH, 4):  # Step by 4 bytes (process 2, skip 2)
                    if i + 2 <= DATA_LENGTH:
                        value = struct.unpack('<H', data[i:i+2])[0]  # Little-endian 16-bit value
                        received_values.append(value)
            
            print(f"Received {len(received_values)} 16-bit values from {num_frames} frames.")
    
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

    return received_values

def plot(data):
    """
    Plots the received data in three formats:
    1. Normal plot of the data.
    2. Plot scaled to the full 16-bit range (0 to 65535).
    3. Histogram showing the data distribution.
    
    Parameters:
        data (list): A list of 16-bit integer values to plot.
    """
    # Normal plot
    plt.figure(figsize=(10, 6))
    plt.plot(data, marker='o', markersize=2, linestyle='-', linewidth=1)
    plt.title("entropy signal zoomed plot")
    plt.xlabel("Index")
    plt.ylabel("16-bit Value")
    plt.grid()
    plt.show()
    
    # Full scale plot
    plt.figure(figsize=(10, 6))
    plt.plot(data, marker='o', markersize=2, linestyle='-', linewidth=1)
    plt.title("entropy signal : Full Scale (0 to 65535)")
    plt.xlabel("Index")
    plt.ylabel("16-bit Value")
    plt.ylim(0, 2**16 - 1)
    plt.grid()
    plt.show()
    
    # Histogram
    plt.figure(figsize=(10, 6))
    plt.hist(data, bins=50, edgecolor='black')
    plt.title("entropy: Histogram")
    plt.xlabel("16-bit Value")
    plt.ylabel("Frequency")
    plt.grid()
    plt.show()
