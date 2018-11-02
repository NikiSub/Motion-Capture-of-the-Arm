
import smbus
import math
import time
import numpy as np
# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
timeStep = 0.03
 
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

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)
    
def array(x,y,z):
    return np.transpose(np.array([[x,y,z]]))
def createRotationMatrix(b):
	angles = np.radians(b)
	a = np.zeros((3,3))
	roll = angles[0][0]
	pitch = angles[1][0]
	yaw = angles[2][0]
	a[0][0] = 1
	a[0][1] = np.sin(roll)*np.tan(pitch)
	a[0][2] = np.cos(roll)*np.tan(pitch)
	a[1][1] = np.cos(roll)
	a[1][2] = np.sin(roll)*-1
	a[2][1] = np.sin(roll)/np.cos(pitch)
	a[2][2] = np.cos(roll)/np.cos(pitch)
	
	
	return a
bus = smbus.SMBus(1) #for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command
 
# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)
bus.write_byte_data(0x68, 0x1b, 8)
bus.write_byte_data(0x68, 0x1c, 8)

aScale = 8192.0
gScale = 65.50

angles = array(0,0,0)

print "gyro data"
print "---------"

gyro_xout = read_word_2c(0x43)
gyro_yout = read_word_2c(0x45)
gyro_zout = read_word_2c(0x47)

print "gyro_xout: ", gyro_xout, " scaled: ", (gyro_xout / gScale)
print "gyro_yout: ", gyro_yout, " scaled: ", (gyro_yout / gScale)
print "gyro_zout: ", gyro_zout, " scaled: ", (gyro_zout / gScale)
"""
print
print "accelerometer data"
print "------------------"
accel_xout = read_word_2c(0x3b)
accel_yout = read_word_2c(0x3d)
accel_zout = read_word_2c(0x3f)
accel_xout_scaled = accel_xout / aScale
accel_yout_scaled = accel_yout / aScale
accel_zout_scaled = accel_zout / aScale

print "accel_xout: ", accel_xout, " scaled: ", accel_xout_scaled
print "accel_yout: ", accel_yout, " scaled: ", accel_yout_scaled
print "accel_zout: ", accel_zout, " scaled: ", accel_zout_scaled
print "x rotation: " , get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
print "y rotation: " , get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
gXinit = 0
gYinit = 0
gZinit = 0
i = 0
"""
while(True): 
	print "gyro data"
	print "---------"

	gyro_xout = read_word_2c(0x43)
	gyro_yout = read_word_2c(0x45)
	gyro_zout = read_word_2c(0x47)
	print "gyro_xout: ", gyro_xout, " scaled: ", (gyro_xout / gScale)
	print "gyro_yout: ", gyro_yout, " scaled: ", (gyro_yout / gScale)
	print "gyro_zout: ", gyro_zout, " scaled: ", (gyro_zout / gScale)
	r = createRotationMatrix(angles)
	g = array(gyro_xout/gScale,gyro_yout/gScale,gyro_zout/gScale)
	angles = angles + timeStep*(np.dot(r,g))

	print ("X: " + str(angles[0][0]) + "Y: " + str(angles[1][0]) + "Z: " + str(angles[2][0]))
	
	print
	
	time.sleep(0.03)
	
