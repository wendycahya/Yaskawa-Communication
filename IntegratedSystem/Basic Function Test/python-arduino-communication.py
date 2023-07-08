import serial

# Configure the serial port
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Replace 'COM3' with your port name

# Read data from Arduino
while True:
    #message = "1"
    message = input("Enter a message to send to Arduino: ")
    ser.write(message.encode())
    print("nilai message= ", message)
    line = ser.readline().decode('latin-1').rstrip()
    print(line)
# Close the serial port connection
ser.close()