import smbus
import math
import time
# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
 
def read_byte(adr):
	return bus.read_byte_data(address, adr)
def read_word(adr):
    low = bus.read_byte_data(address, adr)
    high = bus.read_byte_data(address, 0x2b)
    print("l", low)
    print("h", high)
    val = (high << 8) + low
    return val
    
def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
bus = smbus.SMBus(1) #for Revision 2 boards
address = 0x6b       # This is the address value read via the i2cdetect command

bus.write_byte_data(address, 0x20, 15)
bus.write_byte_data(address, 0x23, 0)

while True:
	gyro_yout = read_word_2c(0x2a)

	print "gyro_yout: ", (gyro_yout/8.75)-10.0
	time.sleep(0.03)
