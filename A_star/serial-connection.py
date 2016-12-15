import serial
import numpy as np
import matplotlib.pyplot as p
import time
import pylab
import re
from time import sleep

arduino=serial.Serial('/dev/ttyACM0', 115200,timeout=1)
#arduino=serial.Serial('/dev/ttyACM0',115200)
arduino.bytesize = serial.EIGHTBITS #number of bits per bytes
arduino.parity = serial.PARITY_NONE #set parity check: no parity
arduino.stopbits = serial.STOPBITS_ONE

f= open('Arduinodata1.txt','r')

s=f.read()
newstr=re.split(", |,\n",s)
sleep(3)

for i in range(len(newstr)-1):	
	word=newstr[i]
	arduino.write(bytearray(str(word).encode()))
	y= bytearray(str(word).encode())
	print y
	sleep(1)
	q=0
	if int(word)<20000:
		q=int(word)
		q=q-10000
		q=(q/180)
		q=q+1
		print'q1', q
	if int(word)>20000 and int(word)<30000:
		q=int(word)
		q=q-20000
		q=(q/180)
		q=q+1
		print'q2', q
	if int(word)>30000:
		q=int(word)
		q=q-30000
		q=(q/180)
		q=q
		print'q3', q
	sleep(q)
	arduino.readline()
	
	print'arduino', arduino.readline()
	sleep(1)
