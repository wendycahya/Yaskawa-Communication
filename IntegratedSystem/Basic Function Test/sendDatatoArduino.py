import serial

port = '/dev/ttyUSB0'  # Replace with your Arduino's port name
baud_rate = 9600  # Set the baud rate to match your Arduino

ser = serial.Serial(port, baud_rate)

# Read data from Arduino
def read_data():
    data = ser.readline().decode().rstrip()
    return data

# Send data to Arduino
def send_data(data):
    ser.write(data.encode())

# Main program
if __name__ == '__main__':
    # Example usage: Read data and send a response
    received_data = read_data()
    print('Received data:', received_data)
    response = 'Hello Arduino!'
    send_data(response)

ser.close()