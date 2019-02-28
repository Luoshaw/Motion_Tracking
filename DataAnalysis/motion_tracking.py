#encoding:utf-8
# import csv
import csv 
import math
import warnings

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

from scipy import signal

#===================================function and class define============================
#================average function==================
def average(seq, num):
	count = 0
	total = 0.0
	for item in range(len(num)):
		total = total + seq[num[item]]
		count += 1
	return total / count
#============quaternConj function==================
def quaternConj(quaternion):
	qConj = []
	for i in range(len(quaternion)):
		qConj.append([quaternion[i][0], -quaternion[i][1], -quaternion[i][2], -quaternion[i][3]])
	return qConj
#==================Quaternion==================
class Quaternion:
    def __init__(self,array):
        self.w=0
        self.x=array[0]
        self.y=array[1]
        self.z=array[2]
        self.array=array
    def toArray(self):
        return [self.w, self.x, self.y, self.z]
    def __add__(self,quaternion):
        result=Quaternion(self.array)
        result.w+=quaternion.w
        result.x+=quaternion.x
        result.y+=quaternion.y
        result.z+=quaternion.z
        return result
    def __sub__(self,quaternion):
        result=Quaternion(self.array)
        result.w-=quaternion.w
        result.x-=quaternion.x
        result.y-=quaternion.y
        result.z-=quaternion.z
        return result
    def multiplication(self,quaternion):
        result=Quaternion(self.array)
        result.w=self.w*quaternion.w-self.x*quaternion.x-self.y*quaternion.y-self.z*quaternion.z
        result.x=self.w*quaternion.x+self.x*quaternion.w+self.y*quaternion.z-self.z*quaternion.y
        result.y=self.w*quaternion.y-self.x*quaternion.z+self.y*quaternion.w+self.z*quaternion.x
        result.z=self.w*quaternion.z+self.x*quaternion.y-self.y*quaternion.x+self.z*quaternion.w
        return result
    def divides(quaternion):
        result=Quaternion(self.array)
        return result.multiplication(quaternion.inverse());
    def mod(self):
        return pow((pow(self.x,2)+pow(self.y,2)+pow(self.z,2)+pow(self.w,2)),1/2)
    def star(self):
        result=Quaternion(self.array)
        result.w=self.w
        result.x=-self.x
        result.y=-self.y
        result.z=-self.z
        return result
    def inverse(self):
        result=Quaternion(self.array)
        moder=self.mod()
        result.w/=moder
        result.x/=moder
        result.y/=moder
        result.z/=moder
        return result
    def __str__(self):
        return str(self.x)+"i "+str(self.y)+"j "+str(self.z)+"k "+str(self.w)
#=============Quaternion Product===============
def quaternProd(a, b):
	result = [0, 0, 0, 0]
	result[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3]
	result[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2]
	result[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1]
	result[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
	return result
#=============Quaternion Rotate================
def quaternRotate(v, q, qconj):
	for i in range(len(v)):
		v0XYZ = quaternProd(quaternProd(q[i], [0, v[i][0], v[i][1], v[i][2]]), qconj[i])
		v[i] = [v0XYZ[1], v0XYZ[2], v0XYZ[3]]
	return v
#=============UpdateIMU Function===============
class AHRS(object):
	SamplePeriod = 1 / 256
	Kp = 2
	Ki = 0
	KpInit = 200
	InitPeriod = 5
	Quaternion = Quaternion([0, 0, 0])
	q = [1, 0, 0, 0]
	IntError = [0, 0, 0]
	"""docstring for AHRS"""
	def __init__(self, SamplePeriod, Kp, KpInit):
		self.SamplePeriod = SamplePeriod
		self.Kp = Kp
		self.KpInit = KpInit

	def UndateIMU(self, Gyroscope, Accelerometer):
		# Normalise accelerometer measurement
		normAccel = np.linalg.norm(Accelerometer, ord = 2)
		if(normAccel == 0):
			warnings.warn('Accelerometer magnitude is zero. Algorithm updata aborted')
			return
		else:
			Accelerometer = Accelerometer / normAccel
		# print 'Accelerometer: ', Accelerometer

		#Compute error between estimated and measured direction of gravity
		v = [2*(self.q[1]*self.q[3] - self.q[0]*self.q[2]), 
			2*(self.q[0]*self.q[1] + self.q[2]*self.q[3]), 
			self.q[0]*self.q[0] - self.q[1]*self.q[1] - self.q[2]*self.q[2] + self.q[3]*self.q[3]]
		# print 'v: ', v
		error = np.cross(v, Accelerometer)	
		# print 'error: ', error
		
		# Compute ramped Kp value used during init period
		self.IntError = self.IntError + error
		# print 'IntError: ', self.IntError
		
		#Apply feedback terms
		Ref = Gyroscope - (self.Kp*error + self.Ki*self.IntError)
		# print 'Ref: ', Ref
		
		# Compute rate of change of quaternion

		quatRef = Quaternion(Ref)
		# print 'quatRefbe: ', quatRef.w, quatRef.x, quatRef.y, quatRef.z
		# List value send to quaternion 
		self.Quaternion.w = self.q[0]
		self.Quaternion.x = self.q[1]
		self.Quaternion.y = self.q[2]
		self.Quaternion.z = self.q[3]
		# print 'Quaternion: ', self.Quaternion.w, self.Quaternion.x, self.Quaternion.y, self.Quaternion.z
		# quaternion multiplication
		quatRef = self.Quaternion.multiplication(quatRef)
		# print 'quatRefaf: ', quatRef.w, quatRef.x, quatRef.y, quatRef.z
		quatRefArray = quatRef.toArray()
		# print 'quatRefArray: ', quatRefArray		
		pDot = [x*0.5 for x in quatRefArray]
		# print 'pDot: ', pDot
		self.q = np.sum([self.q, [x*samplePeriod for x in pDot]], axis = 0)
		# print 'q: ', self.q
		self.q = self.q / np.linalg.norm(self.q, ord = 2)		
		# print 'q: ', self.q

		# Store conjugate
		self.Quaternion.w =  self.q[0]
		self.Quaternion.x = -self.q[1]
		self.Quaternion.y = -self.q[2]
		self.Quaternion.z = -self.q[3]
#==============================================

#======================================main thread======================================
#=============read row gyro and acc data from csv=================
gyroXraw = []
gyroYraw = []
gyroZraw = []
accXraw = []
accYraw = []
accZraw = []
 
with open('test4.csv','rb') as csvfile:
	datareader = csv.reader(csvfile, delimiter=',',quoting=csv.QUOTE_NONE)
	numofline = 0
	for row in datareader:
	    if numofline > 0:
	    	gyroXraw.append(float(row[1]))
	    	gyroYraw.append(float(row[2]))
	    	gyroZraw.append(float(row[3]))
	    	accXraw.append(float(row[4]))
	    	accYraw.append(float(row[5]))
	    	accZraw.append(float(row[6]))
	    numofline = numofline + 1   
# print 'gyroXraw[0:9]: ', gyroXraw[0:9]
# print 'accXraw[0:9]: ', accXraw[0:9]

#===============def samplePeriod and timeslice=======================
samplePeriod = float(1)/200
# print samplePeriod
startTime = 8
stopTime = 53
timeraw = []

for i in range(len(gyroXraw)):
	timeraw.append(float(samplePeriod * i))
# print 'timeraw[0:10]: ', timeraw[0:10]

#=======================Manully frame data===========================
indexSel = []
time = []
gyroX = []
gyroY = []
gyroZ = []
accX = []
accY = []
accZ = []

for i in range(len(timeraw)):
	if timeraw[i] > startTime:
		indexSel.append(i)
	if timeraw[i] > stopTime:
		break
# print 'The selected indexSel: ', indexSel[0],indexSel[-1]
# print 'The length of indecSel: ', len(indexSel)

for selnum in range(len(indexSel)):
	time.append(timeraw[indexSel[selnum] - 1])
	gyroX.append(gyroXraw[indexSel[selnum] - 1])
	gyroY.append(gyroYraw[indexSel[selnum] - 1])
	gyroZ.append(gyroZraw[indexSel[selnum] - 1])
	accX.append(accXraw[indexSel[selnum] - 1])
	accY.append(accYraw[indexSel[selnum] - 1])
	accZ.append(accZraw[indexSel[selnum] - 1])
# print 'gyroX[0:9]: ', gyroX[0:9]
# print 'accX[0:9]: ', accX[0:9]

#======================detect stationary period======================
acc_mag = []
gyro_mag = []
stationary = []
i = 0
while (i < len(accX)):
	calsq = math.sqrt(accX[i] * accX[i] + accY[i] * accY[i] + accZ[i] * accZ[i])
	gyromagsq = math.sqrt(gyroX[i] * gyroX[i] + gyroY[i] * gyroY[i] + gyroZ[i] * gyroZ[i])
	acc_mag.append(calsq)
	gyro_mag.append(gyromagsq)
	i = i + 1
# print 'acc_mag[0:99]: ', acc_mag[0:100]

filtCutOff = 0.001
b,a = signal.butter(1, (2*filtCutOff)/(1/samplePeriod), 'high')
acc_magFilt = signal.filtfilt(b, a, acc_mag)  

acc_magFilt = abs(acc_magFilt)

filtCutOff = 5
b,a = signal.butter(1, (2*filtCutOff)/(1/samplePeriod), 'low')
acc_magFilt = signal.filtfilt(b, a, acc_magFilt)  

# print 'acc_magFilt[0:99]: ', acc_magFilt[0:100]

for num in range(len(acc_magFilt)):
	if acc_magFilt[num] < 0.05:
		stationary.append(1)
	else:
		stationary.append(0)
# print 'stationary[600:860]: ', stationary[600:860]

#==========plot data raw sensor data and stationary periods========
plt.figure(num = 'Sensor Data', figsize=(9, 25))
plt.subplot(211)
plt.plot(time, gyroX, 'r')
plt.plot(time, gyroY, 'y')
plt.plot(time, gyroZ, 'b')
plt.xlim((startTime,stopTime))
plt.xlabel('Time(s)')
plt.ylabel('Angular velocity(deg/s)')
plt.title('Gyroscope')
plt.legend(('X', 'Y', 'Z'))
# plt.plot(time, gyro_mag, 'r')
# plt.xlim((startTime,stopTime))
# plt.xlabel('Time(s)')
# plt.ylabel('Angular velocity(deg/s)')
# plt.title('Gyroscope')
# plt.legend(('X'))
plt.subplot(212)
plt.plot(time, accX, 'r')
plt.plot(time, accY, 'y')
plt.plot(time, accZ, 'b')
plt.plot(time, acc_magFilt, ':k')
plt.plot(time, stationary, 'k', 'LineWidth', 2)
plt.xlim((startTime,stopTime))
plt.title('Accelerometer')
plt.xlabel('Time(s)')
plt.ylabel('Accelerometer(g)')
plt.legend(('X', 'Y', 'Z', 'Filtered', 'stationary'))
plt.show()

#=======================compute orientation=========================
#=======Initial convergence======
quat = []
initPeriod = 2
for n in range(len(indexSel)): 
	indexSel.pop()
InitialTime = time[0] + initPeriod
for i in range(len(time)):
	if time[i] <= InitialTime:
		indexSel.append(i)
	else:
		break
accX_mean = average(accX, indexSel)
accY_mean = average(accY, indexSel)
accZ_mean = average(accZ, indexSel)
# print accX_mean, accY_mean, accZ_mean
AHRSalgorithm = AHRS(samplePeriod, 1, 1)
for x in range(2000):
	AHRSalgorithm.UndateIMU([0, 0, 0], [accX_mean, accY_mean, accZ_mean])
# print 'Quaternion: ', AHRSalgorithm.Quaternion
#====== For all data=============
for i in range(len(time)):
	if stationary[i]:
		AHRSalgorithm.Kp = 0.5
	else:
		AHRSalgorithm.Kp = 0
	gyroXrad = np.deg2rad(gyroX[i])
	gyroYrad = np.deg2rad(gyroY[i])
	gyroZrad = np.deg2rad(gyroZ[i])
	AHRSalgorithm.UndateIMU([gyroXrad, gyroYrad, gyroZrad], [accX[i], accY[i], accZ[i]])
	quat.append(AHRSalgorithm.Quaternion.toArray())

#====================Compute translational accelerations==============
#==== Rotate body acc to Earth frame=====
quatConj = quaternConj(quat)
acc = []
for i in range(len(accX)):
	acc.append([accX[i], accY[i], accZ[i]])
acc = quaternRotate(acc, quatConj, quat)
# print acc[0:100]
#===== Convert acceleration measurements to m/s/s====
for i in range(len(acc)):
	acc[i] = [x*9.81 for x in acc[i]]
# print acc[0:9]
#===Plot translational accelerations=====
acc_x_afrotate = []
acc_y_afrotate = []
acc_z_afrotate = []
for i in range(len(acc)):
	acc_x_afrotate.append(acc[i][0])
	acc_y_afrotate.append(acc[i][1])
	acc_z_afrotate.append(acc[i][2])

plt.figure(num = 'Accelerations')
plt.plot(time, acc_x_afrotate, 'r')
plt.plot(time, acc_y_afrotate, 'y')
plt.plot(time, acc_z_afrotate, 'b')
plt.xlim((startTime,stopTime))
plt.xlabel('Time(s)')
plt.ylabel('Acceleration (m/s/s)')
plt.title('Acceleration')
plt.legend(('X', 'Y', 'Z'))
plt.show()
#====Compute translational velocities====
for i in range(len(acc)):
	acc[i][2] = acc[i][2] - 9.81
#==Integrate acceleration to yield velocity==
vel = []
for i in range(len(acc)):
	vel.append([])
for j in range(3):
	vel[i].append(0)
vel[0] = [0, 0, 0]
for t in range(1, len(acc)):
	vel[t] = list(np.sum([vel[t-1], [x*samplePeriod for x in acc[t]]], axis = 0))
	if stationary[t] == 1:
		vel[t] = [0, 0, 0]
#==Compute integral drift during non-stationary periods==
velDrift = list(np.zeros(np.size(vel,0)))
stationaryStart = []
stationaryEnd = []
for i in range(1, len(stationary)):
	if stationary[i] - stationary[i-1] == -1:
		stationaryStart.append(i)
	if stationary[i] - stationary[i-1] == 1:
		stationaryEnd.append(i)
# print stationaryStart, '\n', stationaryEnd
drift = []
for i in range(len(stationaryEnd)):
	driftRate = np.array(vel[stationaryEnd[i] - 1]) / (stationaryEnd[i] - stationaryStart[i])
	enum = list(range(1, stationaryEnd[i] - stationaryStart[i]))
	for n in range(len(enum)):
		drift.append([enum[n]*driftRate[0], enum[n]*driftRate[1], enum[n]*driftRate[2]])
	for m in range(stationaryStart[i], stationaryEnd[i] - 1):
		velDrift[m] = drift[m - stationaryStart[i]]
#===remove integral drift===
for i in range(len(vel)):
	vel[i] = list(np.array(vel[i]) - np.array(velDrift[i]))
#==plot translational velocity===
vel_x = []
vel_y = []
vel_z = []
for i in range(len(vel)):
	vel_x.append(vel[i][0])
	vel_y.append(vel[i][1])
	vel_z.append(vel[i][2])
plt.figure(num = 'Velocity')
plt.plot(time, vel_x, 'r')
plt.plot(time, vel_y, 'y')
plt.plot(time, vel_z, 'b')
plt.xlim((startTime, stopTime))
plt.xlabel('Time(s)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity')
plt.legend(('X', 'Y', 'Z'))
plt.show()

#====Compute translational position====
pos = []
for i in range(len(vel)):
	pos.append([])
for j in range(3):
	pos[i].append(0)
pos[0] = [0, 0, 0]
for t in range(1, len(vel)):
	pos[t] = list(np.sum([pos[t-1], [x*samplePeriod for x in vel[t]]], axis = 0))
#==plot translational position===
pos_x = []
pos_y = []
pos_z = []
for i in range(len(pos)):
	pos_x.append(pos[i][0])
	pos_y.append(pos[i][1])
	pos_z.append(pos[i][2])
plt.figure(num = 'Position')
plt.plot(time, pos_x, 'r')
plt.plot(time, pos_y, 'y')
plt.plot(time, pos_z, 'b')
plt.xlim((startTime, stopTime))
plt.xlabel('Time(s)')
plt.ylabel('Position (m)')
plt.title('Position')
plt.legend(('X', 'Y', 'Z'))
plt.show()