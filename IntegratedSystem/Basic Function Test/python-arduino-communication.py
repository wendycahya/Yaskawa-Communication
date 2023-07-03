import serial

# Configure the serial port
ser = serial.Serial('COM3', 9600)  # Replace 'COM3' with your port name

# Read data from Arduino
while True:
    if ser.in_waiting > 0:
        data = ser.readline().decode('utf-8').rstrip()
        print("Received:", data)

    # Example of sending data from Python to Arduino
    message = input("Enter a message to send to Arduino: ")
    ser.write(message.encode('utf-8'))

# Close the serial port connection
ser.close()