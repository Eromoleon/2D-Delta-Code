# Port: /dev/ttyACM0

from time import sleep
import serial
BAUDRATE = 115200


ser = serial.Serial('/dev/ttyACM0', BAUDRATE) # Establish the connection on a specific port
counter = 32 # Below 32 everything in ASCII is gibberish

ser.timeout = 10
while True:
    counter +=1
    number = str(chr(counter))
    #print(number)
    #print(number.encode())
    encoded = number.encode('ascii')
    ser.write(encoded) # Convert the decimal number to ASCII then send it to the Arduino
    print (ser.readline().decode()) # Read the newest output from the Arduino
    sleep(.1) # Delay for one tenth of a second
    if counter == 255:
        counter = 32
