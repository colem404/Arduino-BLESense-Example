from __future__ import print_function
from bluepy import btle
import binascii

scanner  = btle.Scanner()
scanner.scan()
device_dict = scanner.scanned
GraceMac = ""

for MAC_addr, device in (device_dict.items()):
	name = device.scanData.get(9)
	print(MAC_addr,name)
	if name != None and "GSenseAct" in str(name):
		print("found")
		GraceMac = MAC_addr
if GraceMac != "":
	dev = btle.Peripheral(GraceMac)
	#dev.discoverServices()
	x=48#len(dev.getCharacteristics())
	#print(x)
	for i in range(x):
		try:
			print(str(dev.readCharacteristic(i+1)))
		except:
			pass
	i=1
	j=1
	for srv in dev.services:
		print("service: ",i)
		i+=1
		print(str(srv))
		for ch in srv.getCharacteristics():
			print("\t",j,". ",str(ch),"--",ch.uuid)
			j+=1
		j=1
	#s1 = list(dev.services)[1]
	#a=s1.getCharacteristics()[0]
	#print(a.read())
