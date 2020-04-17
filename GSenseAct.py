from __future__ import print_function
from bluepy import btle
from struct import unpack, pack

GraceMac = "f6:73:bf:ab:c2:aa"
dev = btle.Peripheral(GraceMac)
dev.discoverServices()

#print(dev.getState())
battService = dev.getServiceByUUID("180f")
battLevel = battService.getCharacteristics()[0]
ActService = dev.getServiceByUUID('DAC27000-E8F2-537E-4F6C-DC04768A1210')
for serv in ActService.getCharacteristics():
	if "a000" in str(serv.uuid):
		massCharac = serv
	if "a001" in str(serv.uuid):
		servoCharac = serv
	if "a002" in str(serv.uuid):
		pumpCharac = serv
SensoryService = dev.getServiceByUUID('DAC27000-E8F2-537E-4F6C-DC04768A1212')
for serv in SensoryService.getCharacteristics():
	if "e000" in str(serv.uuid):
		pressureCharac = serv
	if "e001" in str(serv.uuid):
		temperatureCharac = serv
	if "e002" in str(serv.uuid):
		depthCharac = serv
	if "e003" in str(serv.uuid):
		humidityCharac = serv
IMUService = dev.getServiceByUUID('DAC27000-E8F2-537E-4F6C-DC04768A1211')
for serv in IMUService.getCharacteristics():
	if "f000" in str(serv.uuid):
		gyroCharac = serv
	if "f001" in str(serv.uuid):
		magCharac = serv
	if "f002" in str(serv.uuid):
		accelCharac = serv
	if "f003" in str(serv.uuid):
		eulerCharac = serv



def readBatt():
	return ord(unpack('c',battLevel.read())[0])
    #return int(battLevel.read().hex())/1024.0*100
	
def readMassPos():
	#x=massCharac.read()
	#print("mass-",x,len(x),str(massCharac.uuid))
	return round(unpack('f',massCharac.read())[0],2)
	
def setMassPos(pos):
    pos = max(min(100,pos),-100)
    dev.writeCharacteristic(massCharac.handle,"P".encode('utf-8')+pack('f',pos))
	
	
def setMassSpd(spd):
    spd = max(min(255,spd),-255)
    dev.writeCharacteristic(massCharac.handle,"S".encode('utf-8')+pack('i',spd))
	
	
def readPumpPos():
	#x=pumpCharac.read()
	#print("pump-",x,len(x),str(pumpCharac.uuid))
	return round(unpack('f',pumpCharac.read())[0],2)
	
def setPumpPos(pos):
    pos = max(min(100,pos),-100)
    dev.writeCharacteristic(pumpCharac.handle,"P".encode('utf-8')+pack('f',pos))
	#bytes('P',"utf-8")+pack('f',44.4)
	#pumpCharac.write(bytes('P',"utf-8")+pack('f',44.4))
	
def setPumpSpd(spd):
    spd = max(min(255,spd),-255)
    dev.writeCharacteristic(pumpCharac.handle,"S".encode('utf-8')+pack('b',spd))
	#pumpCharac.write("Sa".encode('utf-8'))#+pack('c',bytes('a',"utf-8")))
	#pumpCharac.write(bytes('S',"utf-8")+bytes(50,"utf-8"))

def setServoPos(pos):
    #pos = max(min(90,pos),-90)
    dev.writeCharacteristic(servoCharac.handle,pack("i",pos))

def readServoPos():
    return unpack('i',servoCharac.read())[0]
	#return int(servoCharac.read().hex())
		
def readDepth():
	#x=depthCharac.read()
	#print("depth-",x,len(x),str(depthCharac.uuid))
	return round(unpack('f',depthCharac.read())[0],3)
	
def readHumidity():
	#x=humidityCharac.read()
	#print("humid-",x,len(x),str(humidityCharac.uuid))
	return round(unpack('f',humidityCharac.read())[0],3)
	
def readPressure():
	x=pressureCharac.read()
	#print("press-",x,len(x),str(pressureCharac.uuid))
	return round(unpack('f',x[0:4])[0],2),round(unpack('f',x[4:8])[0],2)
	
def readTemperature():
	x=temperatureCharac.read()
	#print("temp-",x,len(x),str(temperatureCharac.uuid))
	return round(unpack('f',x[0:4])[0],2),round(unpack('f',x[4:8])[0],2)

def readGyro():
	x=gyroCharac.read()
	#print("gyro-",x,len(x),str(gyroCharac.uuid))
	return unpack('f',x[0:4])[0],unpack('f',x[4:8])[0],unpack('f',x[8:12])[0]
	
def readMag():
	x=magCharac.read()
	#print("mag-",x,len(x),str(magCharac.uuid))
	return unpack('f',x[0:4])[0],unpack('f',x[4:8])[0],unpack('f',x[8:12])[0]
	
def readAccel():
	x=accelCharac.read()
	#print("accel-",x,len(x),str(accelCharac.uuid))
	return unpack('f',x[0:4])[0],unpack('f',x[4:8])[0],unpack('f',x[8:12])[0]
	
def readEuler():
	x=eulerCharac.read()
	#print("accel-",x,len(x),str(eulerCharac.uuid))
	return unpack('f',x[0:4])[0],unpack('f',x[4:8])[0],unpack('f',x[8:12])[0]	

if __name__ == '__main__':	
    print(readBatt())
    print(readMassPos())
    print(readPumpPos())
    print(readTemperature())
    print(readHumidity())
    print(readPressure())
    print(readServoPos())
    print(readDepth())
    print(readGyro())
    print(readMag())
    print(readAccel())
    setPumpPos(10.5)
    setPumpSpd(55)
    setMassPos(95.5)
    setMassSpd(95)
    setServoPos(-45)
