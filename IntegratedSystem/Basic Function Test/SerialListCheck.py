import serial.tools.list_ports

# List available serial ports
ports = serial.tools.list_ports.comports()
for port in ports:
    print(port.device)