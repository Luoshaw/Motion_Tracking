# Motion_Tracking
Motion capture and walking trajectory tracking were performed based on the acceleration and angular velocity data of the inertial sensor.

Py_SerialPort.py : Read raw acc and gyro data from MPU9250 according to usart, and saved in csv file

Py_Serial2Socket.py: Read quat data from MPU9250, and send to socket server for Unity 3D

DataAnalysis: Contain a python code for walking trajectory tracking and some csv data files 
