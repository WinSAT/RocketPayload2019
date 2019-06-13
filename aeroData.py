
#!/usr/bin/env python

from nxp_imu import IMU
import time
import csv
import os
from numpy import array, hstack, vstack, empty, around
from Adafruit_BME280 import *

print "Start aeroData.py"

folderName = "/home/pi/outputData"

fileNum = 0
while "{}.csv".format(fileNum) in os.listdir(folderName):
	fileNum += 1

fileName = "{}/{}.csv".format(folderName,fileNum)
print "saving to: {}".format(fileName)

def createCsv(data, fileName=fileName):
	with open(fileName, 'w') as csvFile:
		writer = csv.writer(csvFile)
		writer.writerow(data)

def write_to_csv(data, fileName=fileName):
	with open(fileName,'a') as csvFile:
		writer = csv.writer(csvFile)
		writer.writerows(data)

headers = array(['idx','accelX_g', 'accelY_g', 'accelZ_g', 'magX_uT', 'magY_uT', 'magZ_uT', 'gyroX_dps', 'gyroY_dps', 'gyroZ_dps', 'gyroX_deg', 'gyroY_deg', 'gyroZ_deg', 'temp_C', 'humidity', 'pres_Pa', 'alt_m'])

createCsv(headers)

imu = IMU(gs=8, dps=2000, verbose=True)
incrementalData = empty((0,len(headers)))
saveIdx = 50
idx = 0
while True:
	imuError = False
	bmeError = False
	try:
		a, m, g = imu.get()
		q = imu.getOrientation(a,m)
		a, m, g, q = [around(a,3),around(m,3),around(g,3),around(q,3)]
	except: 
		a,m,g,q = [["Err"]*3]*4
	try:
		bmeError = True
		bmeSensor = BME280(t_mode=BME280_OSAMPLE_8, p_mode=BME280_OSAMPLE_8, h_mode=BME280_OSAMPLE_8)
		temp = bmeSensor.read_temperature()
		humidity = bmeSensor.read_humidity()
		pressure = bmeSensor.read_pressure()
		alt = (((101325.0/pressure)**(1/5.257)-1)*(temp+273.15))/0.0065
		temp, humidity, pressure, alt = [around(temp,3),around(humidity,2),around(pressure,2),around(alt,2)]
	except: 
		temp, humidity, pressure, alt = ["Err"]*4
	dataPoint = hstack((idx, a, m, g, q, temp, humidity, pressure, alt))
	incrementalData = vstack((incrementalData,dataPoint))
	if idx % saveIdx == 0:
		write_to_csv(incrementalData)
		incrementalData = empty((0,len(headers)))
	idx += 1
	time.sleep(0.05)
