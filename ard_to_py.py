import serial

PORT = '아두이노 포트'
BaudRate = 9600

ARD=serial.Serial(PORT, BaudRate)

while(True):
    a=ARD.readline()
    str = a[:-2].decode()
    angle = int(str)
