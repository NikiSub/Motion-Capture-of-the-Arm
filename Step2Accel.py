import numpy as np
import math as math
import numpy as np
#import smbus
import math
import time
import matplotlib.pyplot as plt

#bus = smbus.SMBus(1)
#bus.write_byte_data(0x68, 0x6b, 0)

"""
def xAccel():
    high = bus.read_byte_data(0x68, 0x3b)
    low = bus.read_byte_data(0x68, 0x3c)
    val = (high << 8) + low
    if (val >= 0x8000):
        val = -((65535 - val) + 1)
    val = val / 1668.0
    return val


def zAccel():
    high = bus.read_byte_data(0x68, 0x3f)
    low = bus.read_byte_data(0x68, 0x40)
    val = (high << 8) + low
    if (val >= 0x8000):
        val = -((65535 - val) + 1)
    val = val / 1668.0
    return val

def yAccel():
    high = bus.read_byte_data(0x68, 0x3d)
    low = bus.read_byte_data(0x68, 0x3e)
    val = (high << 8) + low
    if (val >= 0x8000):
        val = -((65535 - val) + 1)
    val = val / 1668.0
    return val
"""
gConstant=9.81
yX = 0
yY = 0
yZ = -gConstant
timeStep = 0.0003
n_iter = 2000

def cross(r):
    return np.array([[0,-r[2],r[1]],[r[2],0,-r[0]],[-r[1],r[0],0]])

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

    return np.dot(Rz,np.dot(Ry,np.dot(Rx,r)))




#y=np.array([[yX,yY,yZ],[xAccel(),yAccel(),zAccel()]]) # yt-1, yt
testR = array(0,0,-9.81)
testW = array(0,1.57079,0)
testRotate = rotate(testW,testR,1)
readings = []
#readings.append(array(0,0,-1*gConstant))
readings.append(array(-0.4748,-0.4628,-9.7290))
#readings.append(array(-0.4868,-0.4484,-9.808))
#readings.append(array(-1.6475,-0.468,-9.7218))
file = open('pitch30Osc.txt', 'r')
lines = file.readlines()
count = 0

def getReading(count):
    count+=1
    str1 = lines[count]
    a = str1.split()
    ar = array(float(a[0]),float(a[1]),float(a[2]))
    return ar,count


y0 = array(-0.4748,-0.4628,-9.7290)#array(yX,yY,yZ)
y1,count = getReading(count)#array(1,2,3)#array(xAccel(),yAccel(),zAccel())

a0 = array(0,0,0)
a1 = array(0,0,0)

g0 = array(0,0,-gConstant)
#g0 = array(0.4748,0.4628,9.7290)
#g0 = array(0.4748,0.4628,9.7290)
#g1 = array(0,0,gConstant)

b0 = array(0,0,0)
b1 = array(0,0,0)



#a = np.array([[0,0,0],[0,0,0]])

#g = np.array([[0,0,gConstant],[0,0,gConstant]])

#b = np.array([[0,0,0],[0,0,0]])
Qa = np.zeros((3,3))
Qg = np.zeros((3,3))
Qb = np.zeros((3,3))

abc = 0.1**2
abcd = 0.1**2
basdl = 0.1**2000
R =  np.identity(3)*abcd#np.full((3,3),abcd)# measurement variance NEEDS TO BE FINEUTNED
Qomega = np.identity(3)*0.0001#np.full((3,3),abc) # real angular velocity variance NEEDS TO BE FINEUTNED
QdeltaAcceleration = np.identity(3)*0.0001#np.full((3,3),abc) # change in acceleration variance NEEDS TO BE FINEUTNED
Qwb = np.identity(3)*basdl#np.full((3,3),basdl) # white noise variance NEEDS TO BE FINEUTNED
P = np.zeros((6,6))

print(np.linalg.det(R))

C = np.concatenate((-1 * np.identity(3),np.identity(3)),axis=1)
x = np.zeros((6,1))

actX = []
actY = []
actZ = []
pX = []
pY = []
pZ = []

w = array(45,0,0)
t = 2
r = array(0,1,1)
asdnlsa = rotate(w,r,t)
singcount = []
sing=[]

q = []
for k in range(1,498):
    if(k==1015):
        print("hi")
    y0=y1
    y1,count= getReading(count)#array(1,2,3)#array(xAccel(),yAccel(),zAccel())


    n = normalize(y1,1)
    n0 = normalize(y0,1/timeStep)
    w = array(0,0,0)
    w2 = np.transpose(np.cross(np.transpose(n0), np.transpose(n)))
    aPred = rotate2(w2, a0, timeStep)
    gPred = rotate2(w2, g0, timeStep)
    gPred = normalize(gPred, gConstant)
    bPred = rotate2(w2, b0, timeStep)

    yPred = aPred + bPred - gPred


    yError = y1 + yPred

    #gPred = g0
    #aPred = a0
    #bPred = b0
    """
    Qw = Qomega*np.dot(n,np.transpose(n)) + np.dot(1/(gConstant*gConstant)*cross(gPred)*Qomega,np.transpose(cross(gPred))) # angular velociy error covariance matrix
    Qa = np.dot(np.dot(np.identity(3)-timeStep*(cross(w)),Qa),np.transpose(np.identity(3)-timeStep*(cross(w)))) + (timeStep*timeStep)*np.dot(np.dot(cross(aPred),Qw),np.transpose(cross(aPred)))
    Qg = np.dot(np.dot(np.identity(3)-timeStep*(cross(w)),Qg),np.transpose(np.identity(3)-timeStep*(cross(w)))) + (timeStep*timeStep)*np.dot(np.dot(cross(gPred),Qw),np.transpose(cross(gPred))) # gravity error covariance matrix
    Qb = Qb + Qwb # offset error covariance matrix
    Qt1 = np.concatenate((Qg,np.zeros((3,3))),axis=0)
    Qt2 = np.concatenate((np.zeros((3,3)),Qb),axis=0)
    Qt = np.concatenate((Qt1,Qt2),axis=1)
    q.append(Qt[0][0])
    """
    Qw = Qomega * np.dot(n, np.transpose(n)) + np.dot(1 / (gConstant * gConstant) * cross(gPred) * QdeltaAcceleration,np.transpose(cross(gPred)))  # angular velociy error covariance matrix
    Qa = np.multiply(np.multiply(np.identity(3) - timeStep * (cross(w)), Qa),np.transpose(np.identity(3) - timeStep * (cross(w)))) + (timeStep * timeStep) * np.multiply(np.multiply(cross(aPred), Qw), np.transpose(cross(aPred)))
    Qg = (timeStep * timeStep) * np.dot(np.dot(cross(gPred), Qw), np.transpose(cross(gPred)))  # gravity error covariance matrix np.dot(np.dot(np.identity(3)-timeStep*(cross(w)),Qg),np.transpose(np.identity(3)-timeStep*(cross(w)))) +
    Qb = Qb + Qwb  # offset error covariance matrix
    Qt1 = np.concatenate((Qg, np.zeros((3, 3))), axis=0)
    Qt2 = np.concatenate((np.zeros((3, 3)), Qb), axis=0)
    Qt = np.concatenate((Qt1, Qt2), axis=1)
    q.append(Qt[0][0])
    #time update
    #x = np.concatenate((g0,b0))
    R = Qa
    detTest = np.dot(C, np.dot(Qt, C.transpose())) + R
    K = np.dot(np.dot(Qt, C.transpose()), np.linalg.pinv(detTest))
    #K = K *0.75
    x = np.dot(K, yError)

    #g0=g1
    xg = array(x[0][0],x[1][0],x[2][0])
    g0=gPred+xg
    g0 = normalize(g0,gConstant)
    #b0=b1
    b0=bPred+array(x[3][0],x[4][0],x[5][0])
    a0=a1
    a1=aPred



    gX = g0[0][0] / (gConstant)
    if (gX > 1): gX = 1
    if (gX < -1): gX = -1

    gZ = g0[2][0] / (-1 * gConstant)
    if (gZ > 1): gZ = 1
    if (gZ < -1): gZ = -1

    pitchAngle = (np.degrees(np.arcsin(gX)))   # Pitch Angle
    #print('Angle:' + str(pitchAngle))

    actX.append(y1[0][0])
    actY.append(y1[1][0])
    actZ.append(y1[2][0])

    pX.append(-g0[0][0])
    pY.append(-g0[1][0])
    pZ.append(-g0[2][0])

    print(k)
    print(-pitchAngle)
    print("X: " + str(y1[0][0])+ " Y: " + str(y1[1][0]) + " Z: " + str(y1[2][0]))
    print("GX: " + str(-g0[0][0]) + " Y: " + str(-g0[1][0]) + " Z: " + str(-g0[2][0]))
    time.sleep(timeStep/1000)

print(singcount)
#print(sing)


plt.figure()
plt.plot(q,'k-',label='qt')
plt.legend()
plt.title('Estimate Qt vs. iteration step', fontweight='bold')
plt.xlabel('Iteration')
plt.ylabel('Value')

plt.figure()
plt.plot(actX,'k+',label='Actual')
plt.plot(pX,'b-',label='Estimate')
plt.legend()
plt.title('Estimate X vs. iteration step', fontweight='bold')
plt.xlabel('Iteration')
plt.ylabel('Value')

plt.figure()
plt.plot(actY,'k+',label='Actual')
plt.plot(pY,'b-',label='Estimate')
plt.legend()
plt.title('Estimate Y vs. iteration step', fontweight='bold')
plt.xlabel('Iteration')
plt.ylabel('Value')

plt.figure()
plt.plot(actZ,'k+',label='Actual')
plt.plot(pZ,'b-',label='Estimate')
plt.legend()
plt.title('Estimate Z vs. iteration step', fontweight='bold')
plt.xlabel('Iteration')
plt.ylabel('Value')
plt.show()

