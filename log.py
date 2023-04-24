import serial

ser = serial.Serial('/dev/ttyACM0', 115200)
file = open("log.csv", "a")

ser.read_until(b'BEGIN')

while True:
    line = ser.readline()
    print(line)
    file.write(line.decode('utf-8'))
