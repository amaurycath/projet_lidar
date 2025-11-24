import serial
arduino = serial.Serial('COM9', 9600, timeout=1)


def send_command(command):
    arduino.write(f"{cmd}\n".encode())