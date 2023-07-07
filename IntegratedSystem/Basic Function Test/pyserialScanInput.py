import serial

port = '/dev/ttyUSB0'  # Linux
#port = 'COM1'  # Windows
baud_rate = 9600

ser = serial.Serial(port, baud_rate)
command = '1'  # Example command
ser.write(command.encode())
ser.close()