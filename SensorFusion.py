import numpy as np
import smbus
import math
import time
import pygame

pygame.init()
screen = pygame.display.set_mode((600, 600))
done = False

        

#import matplotlib.pyplot as plt
bus = smbus.SMBus(1)
address = 0x68
bus.write_byte_data(address, 0x6b, 0)
bus.write_byte_data(address, 0x1b, 8)
bus.write_byte_data(address, 0x1c, 0)

aScale = 1668.0
gScale = 65.50

def read_byte(a,adr):
	return bus.read_byte_data(a, adr)
def read_word(a,adr):
    high = bus.read_byte_data(a, adr)
    low = bus.read_byte_data(a, adr+1)
    val = (high << 8) + low
    return val
    
def read_word_2c(a,adr):
    val = read_word(a,adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
        
 
def xAccel(adr):
    high = bus.read_byte_data(adr, 0x3b)
    low = bus.read_byte_data(adr, 0x3c)
    val = (high << 8) + low
    if (val >= 0x8000):
        val = -((65535 - val) + 1)
    val = val / aScale
    return val


def zAccel(adr):
    high = bus.read_byte_data(adr, 0x3f)
    low = bus.read_byte_data(adr, 0x40)
    val = (high << 8) + low
    if (val >= 0x8000):
        val = -((65535 - val) + 1)
    val = val / aScale
    return val

def yAccel(adr):
    high = bus.read_byte_data(adr, 0x3d)
    low = bus.read_byte_data(adr, 0x3e)
    val = (high << 8) + low
    if (val >= 0x8000):
        val = -((65535 - val) + 1)
    val = val / aScale
    return val
    
def xGyro(adr):
	gyro_xout = read_word_2c(adr,0x43)
	return gyro_xout/gScale
def yGyro(adr):
	gyro_yout = read_word_2c(adr,0x45)
	return gyro_yout / gScale
def zGyro(adr):
	gyro_zout = read_word_2c(adr,0x47)
	return gyro_zout / gScale


def cross(r):
    return np.array([[0,-r[2][0],r[1][0]],[r[2][0],0,r[0][0]],[-r[1][0],r[0][0],0]])

def array(x,y,z):
    return np.transpose(np.array([[x,y,z]]))

def normalize(a,c):
    mag = math.sqrt(a[0][0]**2+a[1][0]**2+a[2][0]**2)
    b = []
    if(mag==0):
        return a
    for i in range(0,len(a)):
        b.append((a[i][0]*c)/mag)
    return array(b[0],b[1],b[2])

def getMagTheta(a):
    mag = 0
    t=90
    if(not a[2][ 0]==0):
        t = np.arctan(a[0][0]/a[2][0])
    for i in range(0,len(a)):
        mag = mag+(a[i][0]*a[i][0])
    mag = np.sqrt(mag)
    return mag,t

def returnMagTheta(mag, t, y):
    #a = np.sqrt((mag*mag)-(y*y))
    #return array(a*1*np.sin(t),y,a*1*np.cos(t))
    return array(mag * 1 * np.sin(t), 0, mag * 1 * np.cos(t))


def rotate(w,r,timeStep): ## only uses the y component to Rotate
    #return (np.subtract(r,np.transpose(np.cross(np.transpose(timeStep*w),np.transpose(r)))))
    return r-np.dot(cross((timeStep*w)),r)

    theta = w[1,0]*timeStep
    mag,t = getMagTheta(r)
    theta = t+theta

    return returnMagTheta(mag,theta, r[1][0])


def rotate2(w, r, timeStep):  ## ROTATION ORDER: Y, Z, X (For now only y and z)
    # return (np.subtract(r,np.transpose(np.cross(np.transpose(timeStep*w),np.transpose(r)))))
    angles = timeStep*w
    Rx = np.array([[1,0,0],[0,np.cos(angles[0][0]),np.sin(angles[0][0])],[0,-np.sin(angles[0][0]),np.cos(angles[0][0])]])
    Ry = np.array([[np.cos(angles[1][0]),0,-np.sin(angles[1][0])],[0,1,0],[np.sin(angles[1][0]),0,np.cos(angles[1][0])]])
    Rz = np.array([[np.cos(angles[2][0]),np.sin(angles[2][0]),0],[-np.sin(angles[2][0]),np.cos(angles[2][0]),0],[0,0,1]])

    return np.dot(Rx,np.dot(Rz,np.dot(Ry,r)))
    


gConstant=9.81
yX = 0
yY = 0
yZ = -gConstant
timeStep = 0.03
n_iter = 2000
w=5
sigmaAngles = 0.0001
sigmaGyro = 1
xVal = []
yVal = []
rVal = []
xRVal = []

gY = yGyro(address)#float(a[4])
gX = xGyro(address)#float(a[3])
gZ = zGyro(address)#float(a[3])
aZ = zAccel(address)#float(a[2])
aY = yAccel(address)
aX = xAccel(address)

################################GYRO#############################################

A = np.array([[1,-timeStep],[0,1]])
B = np.array([[timeStep],[0]])

H = np.array([[1,0]])


Rg = np.array([[(sigmaGyro*sigmaGyro)]])
Q = np.array([[(timeStep*timeStep*sigmaAngles*sigmaAngles),0],[0,(sigmaAngles*sigmaAngles)]])

P = np.zeros((2,2))
PR = np.zeros((2,2))


if(aZ>1):
    bZ=1
elif(aZ<-1):
    bZ=-1


angle = np.degrees(np.arccos(bZ))
y = np.array([[angle]])
x = np.array([[0,gY]]).transpose()
xR = np.array([[0,gX]]).transpose()
u = np.array([[gY]])


################################Accel#############################################
y0 = array(yX,yY,yZ)#array(-0.4748,-0.4628,-9.7290)#array(yX,yY,yZ)
y1 = array(aX,aY,aZ)

a0 = array(0,0,0)


g0 = array(0,0,gConstant)


b0 = array(0,0,0)


Qa = np.zeros((3,3))
Qg = np.zeros((3,3))
Qb = np.zeros((3,3))

abc = 0.1**2
abcd = 0.1**2
basdl = 0.1**2000
R =  np.full((3,3),abcd)# measurement variance NEEDS TO BE FINEUTNED
Qomega = np.full((3,3),abc) # real angular velocity variance NEEDS TO BE FINEUTNED
QdeltaAcceleration = np.full((3,3),abc) # change in acceleration variance NEEDS TO BE FINEUTNED
Qwb = np.full((3,3),basdl) # white noise variance NEEDS TO BE FINEUTNED
#P = np.zeros((6,6))


C = np.concatenate((-1 * np.identity(3),np.identity(3)),axis=1)
xgr = np.array([[0,gY]]).transpose()
xR = np.array([[0,gX]]).transpose()

for k in range(1,10000):
	gY = yGyro(address)#float(a[4])
	gX = xGyro(address)#float(a[3])
	gZ = zGyro(address)#float(a[3])
	aZ = zAccel(address)#float(a[2])
	aY = yAccel(address)
	aX = xAccel(address)
	
	################################Accel#############################################
	y0 = y1
	y1 = array(aX,aY,aZ)
	n = normalize(y1,1)
	n0 = normalize(y0,1/timeStep)
	w = array(0,0,0)
	w = normalize(np.transpose(np.cross(np.transpose(y0),np.transpose(y1))),(1/timeStep))
	w2 = np.transpose(np.cross(np.transpose(n0),np.transpose(n)))
	aPred = rotate2(w2,a0,timeStep)
	gPred = rotate2(w2,g0,timeStep)
	gPred = normalize(gPred,gConstant)
	bPred = rotate2(w2,b0,timeStep)
	
	yPred = aPred + bPred - gPred#
	
	yError = y1 + yPred

	Qw = Qomega*np.dot(n,np.transpose(n)) + np.dot(1/(gConstant*gConstant)*cross(gPred)*Qomega,np.transpose(cross(gPred))) # angular velociy error covariance matrix
	Qa = np.multiply(np.multiply(np.identity(3)-timeStep*(cross(w)),Qa),np.transpose(np.identity(3)-timeStep*(cross(w)))) + (timeStep*timeStep)*np.multiply(np.multiply(cross(aPred),Qw),np.transpose(cross(aPred)))
	Qg = (timeStep*timeStep)*np.dot(np.dot(cross(gPred),Qw),np.transpose(cross(gPred))) # gravity error covariance matrix np.dot(np.dot(np.identity(3)-timeStep*(cross(w)),Qg),np.transpose(np.identity(3)-timeStep*(cross(w)))) + 
	Qb = Qb + Qwb # offset error covariance matrix
	Qt1 = np.concatenate((Qg,np.zeros((3,3))),axis=0)
	Qt2 = np.concatenate((np.zeros((3,3)),Qb),axis=0)
	Qt = np.concatenate((Qt1,Qt2),axis=1)	
		
	R = Qa
	detTest = np.dot(C,np.dot(Qt,C.transpose()))+R
	K = np.dot(np.dot(Qt,C.transpose()),np.linalg.pinv(detTest))
	x = np.dot(K,yError)
	  
	xg = array(x[0][0],x[1][0],x[2][0])
	g0=gPred+xg
	g0 = normalize(g0,gConstant)

	b0=bPred+array(x[3][0],x[4][0],x[5][0])
	#a0=a1
	#a1=aPred
	gX = g0[0][0] / (gConstant)
	if (gX > 1): gX = 1
	if (gX < -1): gX = -1
	aX = y1[0][0] / (gConstant)
	if (aX > 1): aX = 1
	if (aX < -1): aX = -1
	gZ = -g0[2][0]
	gY = -g0[1][0]

	pitchAngle = (np.degrees(np.arcsin(gX)))   # Pitch Angle
	rollNeg = np.arcsin(gY / (gZ ** 2 + gY ** 2) ** 0.5) #/ abs(np.arcsin(aY / (aZ ** 2 + aY ** 2) ** 0.5))
	if(rollNeg==0):
		rollNeg = 1
	else:
		rollNeg = rollNeg/abs(rollNeg)
	rollAngle = abs(np.degrees(np.arccos(gZ / (gZ ** 2 + gY ** 2) ** 0.5))) * rollNeg
		
	print("Pitch: " + str(pitchAngle) + " Roll: " + str(rollAngle))
	
	
	################################Gyro#############################################
	
	u = np.array([[gY]])
	y = np.array([[pitchAngle]])
	uR = np.array([[gX]])
	yR = np.array([[rollAngle]])
	#time update
	xgr = np.dot(A,xgr)
	xgr = xgr +np.dot(B,u)
	P = np.dot(np.dot(A,P),A.transpose())+Q

	xR = np.dot(A, xR) + np.dot(B, uR)
	PR = np.dot(np.dot(A, PR), A.transpose()) + Q


	#measurement update

	Kinv = np.linalg.inv(np.dot(np.dot(H,P),H.transpose())+Rg)
	K = np.dot(np.dot(P,H.transpose()),Kinv)
	xgr = xgr + np.dot(K,(y-np.dot(H,xgr)))

	I = np.identity(2)

	P = np.dot((I - np.dot(K,H)),P)

	KinvR = np.linalg.inv(np.dot(np.dot(H, PR), H.transpose()) + Rg)
	KR = np.dot(np.dot(PR, H.transpose()), KinvR)
	xR = xR + np.dot(K, (yR - np.dot(H, xR)))

	PR = np.dot((I - np.dot(KR, H)), PR)
	print("Pitch: " + str(x[0][0]) + " Roll: " + str(xR[0][0]))
	time.sleep(timeStep)
	



