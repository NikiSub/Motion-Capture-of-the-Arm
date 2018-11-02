import numpy as np
import math as math
import numpy as np
import smbus
import math
import time
#import matplotlib.pyplot as plt

bus = smbus.SMBus(1)
bus.write_byte_data(0x68, 0x6b, 0)
bus.write_byte_data(0x68, 0x1b, 8)
bus.write_byte_data(0x68, 0x1c, 0)
address = 0x68
aScale = 1668.0
gScale = 65.50

def read_byte(adr):
	return bus.read_byte_data(address, adr)
def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val
    
def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
        
 
def xAccel():
    high = bus.read_byte_data(0x68, 0x3b)
    low = bus.read_byte_data(0x68, 0x3c)
    val = (high << 8) + low
    if (val >= 0x8000):
        val = -((65535 - val) + 1)
    val = val / aScale
    return val


def zAccel():
    high = bus.read_byte_data(0x68, 0x3f)
    low = bus.read_byte_data(0x68, 0x40)
    val = (high << 8) + low
    if (val >= 0x8000):
        val = -((65535 - val) + 1)
    val = val / aScale
    return val

def yAccel():
    high = bus.read_byte_data(0x68, 0x3d)
    low = bus.read_byte_data(0x68, 0x3e)
    val = (high << 8) + low
    if (val >= 0x8000):
        val = -((65535 - val) + 1)
    val = val / aScale
    return val
    
def xGyro():
	gyro_xout = read_word_2c(0x43)
	return gyro_xout/gScale
def yGyro():
	gyro_yout = read_word_2c(0x45)
	return gyro_yout / gScale
def zGyro():
	gyro_zout = read_word_2c(0x47)
	return gyro_zout / gScale
	
file = open('const45.txt','w') 
 

for i in range(0,500):
	#print("AX: " + str(xAccel())+ " AY: " + str(yAccel()) + " AZ: " + str(zAccel()))
	#print("GX: " + str(xGyro()) + " GY: " + str(yGyro()) + " GZ: " + str(zGyro()))
	file.write(str(xAccel())+ " " + str(yAccel()) + " " + str(zAccel()) + " " + str(xGyro()) + " " + str(yGyro()) + " " + str(zGyro())) 
	file.write('\n')
	print(i)
	time.sleep(0.03)

#file.write(str(xAccel())+ " " + str(yAccel()) + " " + str(zAccel()) + " " + str(xGyro()) + " " + str(yGyro()) + " " + str(zGyro())) 

 
file.close() 
