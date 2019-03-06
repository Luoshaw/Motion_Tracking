#coding:utf-8
import serial
import struct
import time
import numpy as np
import warnings
import math
from ReadData import TcpData, average
#====================AHRS========================
# quaternConj function
def quaternConj(quaternion):
	qConj = []
	qConj = [quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]]
	return qConj
# quaternion product	
def quaternProd(a, b):
	result = [0, 0, 0, 0]
	result[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3]
	result[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2]
	result[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1]
	result[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
	return result
# Vector Rotate
def quaternRotate(v, q, qconj):
	v0xyz = quaternProd(quaternProd(q, [0, v[0], v[1], v[2]]), qconj)
	v = [v0xyz[1], v0xyz[2], v0xyz[3]]
	return v
# Initial AHRS parameter
def InitCovergence(reader, samplePeriod, gyro_offset):
	count = 0
	SampleNumber = 400
	accX = []
	accY = []
	accZ = []
	gyroX = []
	gyroY = []
	gyroZ = []
	AHRSalgorithm = AHRS(samplePeriod, 1, 1)
	while count < SampleNumber:
		reader.read()
		if reader.rec == True:
			Data = TcpData(reader.receidata, gyro_offset)
			if (abs(Data.dmp_accx) < 8 and abs(Data.dmp_accy) < 8 and abs(Data.dmp_accz) < 8 and abs(Data.dmp_gyrox) < 2000 and abs(Data.dmp_gyroy) < 2000 and abs(Data.dmp_gyroz) < 2000 ):
				accX.append(Data.dmp_accx)
				accY.append(Data.dmp_accy)
				accZ.append(Data.dmp_accz)
				gyroX.append(Data.dmp_gyrox)
				gyroY.append(Data.dmp_gyroy)
				gyroZ.append(Data.dmp_gyroz)
				count += 1
	accX_mean = average(accX, SampleNumber)
	accY_mean = average(accY, SampleNumber)
	accZ_mean = average(accZ, SampleNumber)
	gyro_offset = [average(gyroX, SampleNumber), average(gyroY, SampleNumber), average(gyroZ, SampleNumber)]
	for x in range(2000):
		AHRSalgorithm.UndateIMU([0, 0, 0], [accX_mean, accY_mean, accZ_mean])
	return [gyro_offset, AHRSalgorithm]
# Update Quaternion
def UpdateQuat(data):
	gyroXrad = np.deg2rad(data.dmp_gyrox)
	gyroYrad = np.deg2rad(data.dmp_gyroy)
	gyroZrad = np.deg2rad(data.dmp_gyroz)
	AHRSalgorithm.UndateIMU([gyroXrad, gyroYrad, gyroZrad], [data.dmp_accx, data.dmp_accy, data.dmp_accz])
	return AHRSalgorithm.Quaternion.toArray()
# Convert quaternion to euler
def quat2euler(quat):
	w = quat[0]
	x = quat[1]
	y = quat[2]
	z = quat[3]
	r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
	p = math.asin(2*(w*y-z*z))
	y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
	angleR = r*180/math.pi
	angleP = p*180/math.pi
	angleY = y*180/math.pi
	result = [angleR, angleP, angleY]
	return result
# quaternion class
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
# AHRS init and update algorithm
class AHRS(object):
	SamplePeriod = 1 / 200
	Kp = 1
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