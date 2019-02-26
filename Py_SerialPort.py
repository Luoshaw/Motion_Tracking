#coding:utf-8
import csv
import serial
import binascii
import struct
import numpy as np
import time
#============Timing Function=====================
def dur(op=None, clock=[time.time()]):
	if op != None:
		duration = time.time() - clock[0]
		print '%s finished. Duration %.6f seconds.' % (op, duration)
	clock[0] = time.time()
#==============Read Serial data==================
class packet_reader:
	def __init__(self, port, baudrate, timeout):
		self.ser = serial.Serial(port, baudrate, timeout = timeout)

	def read(self):
		NUM_BYTES = 88
		self.rec = 0
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
				self.rec = 1
				# print receidata			

	def close(self):
		self.ser.close()
# =================== PACKETS =================== 
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
# ==================== MAIN ======================
if __name__ == "__main__":

	count = 0
	Packet_number = 0
 	reader = packet_reader('COM4', 500000, 1)
	dur() #Initialise the timing clock

	with open("DataAnalysis/IMUrawdata4.csv", "wb") as csvfile:	
		writer = csv.writer(csvfile)
		writer.writerow(["Packet number", "gyrox", "gyroy", "gyroz", "accx", "accy", "accz"])	
		while 1:		
			reader.read()
			#Raw data dispose
			if reader.rec == 1:	
				Data = TcpData(reader.receidata)
				Packet_number += 1
				writer.writerow([Packet_number, Data.dmp_gyrox, Data.dmp_gyroy, Data.dmp_gyroz, Data.dmp_accx, Data.dmp_accy, Data.dmp_accz])	
				count += 1
				if count == 100:
					dur('Data dispose')
					count = 0
					Data.display()
					