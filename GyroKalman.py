import numpy as np
#import smbus
import math
import time
import matplotlib.pyplot as plt

gConstant=9.81
yX = 0
yY = 0
yZ = -gConstant
timeStep = 0.03
n_iter = 2000
w=5
sigmaAngles = 0.001
sigmaGyro = 0.001
xVal = []
yVal = []
rVal = []
xRVal = []

A = np.array([[1,-timeStep],[0,1]])
B = np.array([[timeStep],[0]])

H = np.array([[1,0]])


R = np.array([[(sigmaGyro*sigmaGyro)]])
Q = np.array([[(timeStep*timeStep*sigmaAngles*sigmaAngles),0],[0,(sigmaAngles*sigmaAngles)]])

P = np.zeros((2,2))
PR = np.zeros((2,2))



file = open('pitchAndRoll2.txt', 'r')
lines = file.readlines()
count = 0

str1 = lines[0]
a = str1.split()
gY = float(a[4])
gX = float(a[3])
aZ = float(a[2])
aZ = (aZ/gConstant)
if(aZ>1):
    aZ=1
elif(aZ<-1):
    aZ=-1


angle = np.degrees(np.arccos(aZ))
y = np.array([[angle]])
x = np.array([[0,gY]]).transpose()
xR = np.array([[0,gX]]).transpose()
u = np.array([[gY]])
rollNeg = 1.0
for k in range(1,len(lines)):
    str1 = lines[k]
    a = str1.split()
    gY = float(a[4])
    gX = float(a[3])
    aZ = float(a[2])
    aX = float(a[0])
    aY = float(a[1])
    mag = (aZ*aZ+aX*aX+aY*aY)**0.5
    acX = (aX / gConstant)
    if (acX > 1):
        acX = 1
    elif (acX < -1):
        acX = -1

    angle = np.degrees(np.arcsin(acX))
    n = np.arcsin(aY/(aZ**2+aY**2)**0.5)
    if (n != 0):
        rollNeg = n / (abs(n))
    else:
        rollNeg = n


    rollAngle = abs(np.degrees(np.arccos(aZ/(aZ**2+aY**2)**0.5))) * rollNeg
    rVal.append(rollAngle)
    u = np.array([[gY]])
    y = np.array([[angle]])
    uR = np.array([[gX]])
    yR = np.array([[rollAngle]])

    #time update
    x = np.dot(A,x)+np.dot(B,u)
    P = np.dot(np.dot(A,P),A.transpose())+Q

    xR = np.dot(A, xR) + np.dot(B, uR)
    PR = np.dot(np.dot(A, PR), A.transpose()) + Q


    #measurement update

    Kinv = np.linalg.inv(np.dot(np.dot(H,P),H.transpose())+R)
    K = np.dot(np.dot(P,H.transpose()),Kinv)
    x = x + np.dot(K,(y-np.dot(H,x)))

    I = np.identity(2)

    P = np.dot((I - np.dot(K,H)),P)

    KinvR = np.linalg.inv(np.dot(np.dot(H, PR), H.transpose()) + R)
    KR = np.dot(np.dot(PR, H.transpose()), KinvR)
    xR = xR + np.dot(K, (yR - np.dot(H, xR)))

    PR = np.dot((I - np.dot(KR, H)), PR)


    print("Pitch:  " + str(x[0][0]) + " Roll: " + str(xR[0][0]))
    xVal.append(x[0][0])
    yVal.append(y[0][0])
    xRVal.append(xR[0][0])
    #print(y[0][0])


plt.figure()
plt.plot(xVal, 'k+', label='Gyro')
plt.plot(yVal, 'b+', label='Accel')
#plt.plot(rVal, 'g+', label='RollA' )
plt.plot(xRVal, 'r+', label='RollG' )
plt.legend()
plt.title('A and G', fontweight='bold')
plt.xlabel('Iteration')
plt.ylabel('Value')
plt.show()
#print (lines)