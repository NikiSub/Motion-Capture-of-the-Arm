import numpy as np
import smbus
import math
import time

bus = smbus.SMBus(1)
bus.write_byte_data(0x68, 0x6b, 0)

def xAccel():
	high = bus.read_byte_data(0x68, 0x3b)
	low = bus.read_byte_data(0x68, 0x3c)
	val = (high << 8) + low
	if (val >= 0x8000):
		val = -((65535 - val) + 1)
	val = val/1668.0
	return val
	
def zAccel():
	high = bus.read_byte_data(0x68, 0x3f)
	low = bus.read_byte_data(0x68, 0x40)
	val = (high << 8) + low
	if (val >= 0x8000):
		val = -((65535 - val) + 1)
	val = val/1668.0
	return val


n_iter = 5000000
sz = (n_iter,)
accX = []
accZ = []

Q = 9.2e-6 # process variance NEEDS TO BE FINEUTNED


xhat=np.zeros(sz)      # a posteri estimate of x  PREDICTION
Px=np.zeros(sz)         # a posteri error estimate PREDICTION
xhatminus=np.zeros(sz) # a priori estimate of x
Pxminus=np.zeros(sz)    # a priori error estimate
Kx=np.zeros(sz)         # gain


zhat=np.zeros(sz)      # a posteri estimate of z  PREDICTION
Pz=np.zeros(sz)         # a posteri error estimate PREDICTION
zhatminus=np.zeros(sz) # a priori estimate of z
Pzminus=np.zeros(sz)    # a priori error estimate
Kz=np.zeros(sz)         # gain


R = 0.01*2 # measurement variance NEEDS TO BE FINEUTNED


xhat[0] = 0.0
Px[0] = 1.0

zhat[0] = -1.0
Pz[0] = 1.0
g=9.81
for k in range(1,n_iter):
    # time update
    xhatminus[k] = xhat[k-1]
    Pxminus[k] = Px[k-1]+Q

    zhatminus[k] = zhat[k - 1]
    Pzminus[k] = Pz[k - 1] + Q

    # measurement update
    Kx[k] = Pxminus[k]/( Pxminus[k]+R )
    xhat[k] = xhatminus[k]+Kx[k]*(xAccel()-xhatminus[k])
    Px[k] = (1-Kx[k])*Pxminus[k]

    Kz[k] = Pzminus[k] / (Pzminus[k] + R)
    zhat[k] = zhatminus[k] + Kz[k] * (zAccel() - zhatminus[k])
    Pz[k] = (1 - Kz[k]) * Pzminus[k]

    gZ = zhat[k]/(-1*g)
    gX = xhat[k]/(g)
    if(gX>1) : gX = 1
    if(gX<-1) : gX = -1
    
    if(gZ>0): gZ = 0
    if(gZ<-1): gZ = -1
    pitchAngle = (180-np.degrees(np.arccos(gZ))+np.degrees(np.arcsin(gX)))/2 #Pitch Angle
    #print(pitchAngle)
    
    print('Angle:' + str(np.degrees(np.arcsin(gX))))
    #print(pitchAngle)
    time.sleep(.03)




