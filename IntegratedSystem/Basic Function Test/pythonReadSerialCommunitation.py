import serial

port = '/dev/ttyUSB0'  # Replace with your Arduino's port name
baud_rate = 9600  # Set the baud rate to match your Arduino

ser = serial.Serial(port, baud_rate)

while True:
    try:
        line = ser.readline().decode('latin-1').rstrip()
        print(line)
    except KeyboardInterrupt:
        break

ser.close()