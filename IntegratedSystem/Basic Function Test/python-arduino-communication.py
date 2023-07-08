import time

import serial

# Configure the serial port
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Replace 'COM3' with your port name

ser.write('0'.encode())
time.sleep(1)
# Read data from Arduino
while True:
    # message = '1'
    message = input("Enter a message to send to Arduino: ")
    # ser.write('1'.encode())
    ser.write(message.encode())
    # print("nilai message= ", message)
    # line = ser.readline().decode('latin-1').rstrip()
    # print(line)
    time.sleep(1)
# Close the serial port connection
ser.close()