#coding:utf-8
import serial
import struct
import time
import numpy as np
import warnings
import math
from AHRS import quaternConj, quaternProd, quaternRotate, InitCovergence
from AHRS import UpdateQuat, quat2euler, Quaternion, AHRS
from ReadData import packet_reader, average, bytestoshort, bytestofloat
from ReadData import TcpData
# ==================== MAIN ======================
#Init velocity and position
samplePeriod = float(1)/200
accReal_1 = [0.0, 0.0, 0.0]
velocity_1 = [0.0, 0.0, 0.0]
velocity = [0.0, 0.0, 0.0]
position  = [0.0, 0.0, 0.0]
gyro_offset = [0.0, 0.0, 0.0]
stationary = 0
disp_count = 0

#Init serial port
reader = packet_reader('COM4', 500000, 1)

#Init AHRS parameters
[gyro_offset, AHRSalgorithm] = InitCovergence(reader, samplePeriod, gyro_offset)	
# print AHRSalgorithm.Quaternion.w, AHRSalgorithm.Quaternion.x, AHRSalgorithm.Quaternion.y, AHRSalgorithm.Quaternion.z
#Forever LOOP
while True:
	#Read raw data from serial		
	reader.read()
	#while data receives
	if reader.rec == True:	
		#Raw data dispose
		Data = TcpData(reader.receidata, gyro_offset)
		#display data
		# Data.display()
		if (abs(Data.dmp_accx) < 8 and abs(Data.dmp_accy) < 8 and abs(Data.dmp_accz) < 8 and abs(Data.dmp_gyrox) < 2000 and abs(Data.dmp_gyroy) < 2000 and abs(Data.dmp_gyroz) < 2000 ):
			#Attitude algorithm
			quat = UpdateQuat(Data)
			euler = quat2euler(quat)
			#calculate quaternion conjugate
			quatConj = quaternConj(quat)
			#transform acc to geographic coordinate system
			acc = quaternRotate([Data.dmp_accx, Data.dmp_accy, Data.dmp_accz], quatConj, quat)
			accReal = [x*9.81 for x in acc]
			#remove gravity
			acc_mag = math.sqrt(accReal[0]*accReal[0] + accReal[1]*accReal[1] + accReal[2]*accReal[2])
			accReal[2] = accReal[2] - 9.81			
			gyro_mag = math.sqrt(Data.dmp_gyrox*Data.dmp_gyrox + Data.dmp_gyroy*Data.dmp_gyroy + Data.dmp_gyroz*Data.dmp_gyroz)	
			if (acc_mag < 10.8 and acc_mag > 9.5 and gyro_mag < 50):
				stationary = 1
			else:
				stationary = 0
			#detect stationary states
			if stationary != 1:
				#Update velocity and position
				for i in range(3):
					velocity[i] = velocity[i] + samplePeriod * (accReal[i] + accReal_1[i]) / 2
				for i in range(3):
					position[i] = position[i] + samplePeriod * (velocity[i]	+ velocity_1[i]) / 2			
			else:
				for i in range(3):
					velocity[i] = 0
				for i in range(3):
					position[i] = position[i] + samplePeriod * (velocity[i]	+ velocity_1[i]) / 2
			accReal_1 = accReal
			velocity_1 = velocity
			#display parameter
			disp_count += 1
			if disp_count == 200:
				disp_count = 0			
				print "quaternion: ", quat
				print "euler: ", euler
				print "acc: ", accReal
				print "acc_mag: ", acc_mag
				print "gyro_mag: ", gyro_mag
				print "velocity: ", velocity
				print "position: ", position, "\n"