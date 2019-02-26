#coding:utf-8
import serial
import struct
import time
import numpy as np
import warnings

#==============Read Serial data==================
# Init the serial port and read data by 
# checkout the first two letters
class packet_reader:
	def __init__(self, port, baudrate, timeout):
		self.ser = serial.Serial(port, baudrate, timeout = timeout)

	def read(self):
		NUM_BYTES = 88
		self.rec = False
		self.receidata = []
		head = self.ser.read(1)
		if chr(ord(head)) == 'R':
			self.receidata.append(head)
			nex = self.ser.read(1)
			if chr(ord(nex)) == 'o':
				self.receidata.append(nex)
				rm = self.ser.read(86)
				for s in rm[0:86]:
					self.receidata.append(s)
				self.rec = True
				# print receidata			

	def close(self):
		self.ser.close()
# =================== PACKETS ===================
# Calculate average value
def average(seq, num):
	count = 0
	total = 0.0
	for item in range(num):
		total = total + seq[item]
		count += 1
	return total / count 
# 2bytes to 1 short value
def bytestoshort(d1,d2):
    d = ord(d1)*256 + ord(d2)
    if d > 32767:
        d -= 65536
    return d
# 4 Bytes to 1 float value
def bytestofloat(d1, d2, d3, d4):
	x = "".join([d1, d2, d3, d4])
	x = struct.unpack('<f', x)[0]
	return x
#Tcpdata struct
class TcpData(object):
	def __init__(self, l):
		self.DataType = []
		self.DataName = []
		self.dmp_qx = 0.0
		self.dmp_qy = 0.0
		self.dmp_qz = 0.0
		self.dmp_qw = 0.0
		self.dmp_accx = 0.0
		self.dmp_accx = 0.0
		self.dmp_accx = 0.0
		self.dmp_gyrox = 0.0
		self.dmp_gyrox = 0.0
		self.dmp_gyrox = 0.0
		self.LocalTime = []
		self.nothing = 0
		self.year = 0
		self.month = 0
		self.day = 0
		self.hour = 0
		self.minute = 0
		self.second = 0
		self.ms = 0

		#DataType
		for i in l[0:10]:
			self.DataType.append(chr(ord(i)))
		self.DataType = "".join(self.DataType)
		#DataName
		for i in l[10:20]:
			self.DataName.append(chr(ord(i)))
		self.DataName = "".join(self.DataName)
		#quat  
		self.dmp_qx = bytestofloat(l[20], l[21], l[22], l[23])
		self.dmp_qy = bytestofloat(l[24], l[25], l[26], l[27])
		self.dmp_qz = bytestofloat(l[28], l[29], l[30], l[31])
		self.dmp_qw = bytestofloat(l[32], l[33], l[34], l[35])
		#accel
		self.dmp_accx = bytestofloat(l[36], l[37], l[38], l[39])
		self.dmp_accy = bytestofloat(l[40], l[41], l[42], l[43])
		self.dmp_accz = bytestofloat(l[44], l[45], l[46], l[47])
		#gyro
		self.dmp_gyrox = bytestofloat(l[48], l[49], l[50], l[51])
		self.dmp_gyroy = bytestofloat(l[52], l[53], l[54], l[55])
		self.dmp_gyroz = bytestofloat(l[56], l[57], l[58], l[59])
		#LocalTime
		for i in l[60:70]:
			self.LocalTime.append(chr(ord(i)))
		self.LocalTime = "".join(self.LocalTime)
		#Time
		self.year = bytestoshort(l[73], l[72])
		self.month = bytestoshort(l[75], l[74])
		self.day = bytestoshort(l[77], l[76])
		self.hour = bytestoshort(l[79], l[78])
		self.minute = bytestoshort(l[81], l[80])
		self.second = bytestoshort(l[83], l[82])
		self.ms = bytestoshort(l[85], l[84])

	def display(self):
		print "DataType: ", self.DataType
		print "DataName: ", self.DataName
		print "Quat: ", self.dmp_qw, self.dmp_qx, self.dmp_qy, self.dmp_qz
		print "Accel: ", self.dmp_accx, self.dmp_accy, self.dmp_accz
		print "Gyro: ", self.dmp_gyrox, self.dmp_gyroy, self.dmp_gyroz
		print "LocalTime: ", self.LocalTime
		print "Time: "+str(self.year)+"/"+str(self.month)+"/"+str(self.day)+" "+str(self.hour)+":"+str(self.minute)+":"+str(self.second)+":"+str(self.ms) 
		print '\n'
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
def InitCovergence():
	InitConvegence = 0
	accX = []
	accY = []
	accZ = []
	AHRSalgorithm = AHRS(samplePeriod, 1, 1)
	while InitConvegence < 400:
		reader.read()
		if reader.rec == True:
			Data = TcpData(reader.receidata)
			accX.append(Data.dmp_accx)
			accY.append(Data.dmp_accy)
			accZ.append(Data.dmp_accz)
			InitConvegence += 1
	accX_mean = average(accX, 400)
	accY_mean = average(accY, 400)
	accZ_mean = average(accZ, 400)
	for x in range(2000):
		AHRSalgorithm.UndateIMU([0, 0, 0], [accX_mean, accY_mean, accZ_mean])
	return AHRSalgorithm
# Update Quaternion
def UpdateQuat(data):
	gyroXrad = np.deg2rad(data.dmp_gyrox)
	gyroYrad = np.deg2rad(data.dmp_gyroy)
	gyroZrad = np.deg2rad(data.dmp_gyroz)
	AHRSalgorithm.UndateIMU([gyroXrad, gyroYrad, gyroZrad], [data.dmp_accx, data.dmp_accy, data.dmp_accz])
	return AHRSalgorithm.Quaternion.toArray()
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
# ==================== MAIN ======================
#Init velocity and position
samplePeriod = float(1)/200
velocity = [0.0, 0.0, 0.0]
position  = [0.0, 0.0, 0.0]

#Init serial port
reader = packet_reader('COM7', 500000, 1)

#Init AHRS parameters
AHRSalgorithm = InitCovergence()
# print AHRSalgorithm.Quaternion.w, AHRSalgorithm.Quaternion.x, AHRSalgorithm.Quaternion.y, AHRSalgorithm.Quaternion.z

#Forever LOOP
while True:
	#Read raw data from serial		
	reader.read()
	#while data receives
	if reader.rec == True:	
		#Raw data dispose
		Data = TcpData(reader.receidata)
		#display data
		# Data.display()
		if (abs(Data.dmp_accx) < 8 and abs(Data.dmp_accy) < 8 and abs(Data.dmp_accz) < 8 and abs(Data.dmp_gyrox) < 2000 and abs(Data.dmp_gyroy) < 2000 and abs(Data.dmp_gyroz) < 2000 ):
			#Attitude algorithm
			quat = UpdateQuat(Data)
			# print quat
			quatConj = quaternConj(quat)
			acc = quaternRotate([Data.dmp_accx, Data.dmp_accy, Data.dmp_accz], quatConj, quat)
			accReal = [x*9.81 for x in acc]
			accReal[2] = accReal[2] - 9.81
			print "acc: ", accReal
			for i in range(3):
				velocity[i] = velocity[i] + samplePeriod * accReal[i]
			print "velocity: ", velocity
			for i in range(3):
				position[i] = position[i] + samplePeriod * velocity[i]
			print "position: ", position, "\n"
			