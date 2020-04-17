#include<Servo.h>
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include <Wire.h>
#include "MS5837.h"
#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
#include <MadgwickAHRS.h>

 // BLE Battery Service
BLEService batteryService("180F");
BLEService ActService("DAC27000-E8F2-537E-4F6C-DC04768A1210");  // create services
BLEService IMUService("DAC27000-E8F2-537E-4F6C-DC04768A1211");
BLEService SensoryService("DAC27000-E8F2-537E-4F6C-DC04768A1212");
// create characteristics and allow remote device to read and write
BLECharacteristic PressureCharacteristic("E000", BLERead,8);
BLEDescriptor PressureCharacDescriptor("2901","Pressure-In,Out");
BLECharacteristic TemperatureCharacteristic("E001", BLERead,8);
BLEDescriptor TemperatureCharacDescriptor("2901","Temperature-In,Out");
BLEFloatCharacteristic DepthCharacteristic("E002", BLERead);
BLEDescriptor DepthCharacDescriptor("2901","Depth");
BLEFloatCharacteristic HumidityCharacteristic("E003", BLERead);
BLEDescriptor HumidityCharacDescriptor("2901","Humidity");

BLECharacteristic IMUGyroCharacteristic("F000", BLERead,12);
BLEDescriptor IMUGyroCharacDescriptor("2901","Gyro(float)-gx,gy,gz");
BLECharacteristic IMUMagCharacteristic("F001", BLERead,12);
BLEDescriptor IMUMagCharacDescriptor("2901","Mag(float)-mx,my,mz");
BLECharacteristic IMUAccelCharacteristic("F002", BLERead,12);
BLEDescriptor IMUAccelCharacDescriptor("2901","Accel(float)-ax,ay,az");
BLECharacteristic IMUYPRCharacteristic("F003", BLERead,12);
BLEDescriptor IMUYPRCharacDescriptor("2901","Euler(degrees)-Y,P,R");
// BLE Battery Level Characteristic
BLEUnsignedCharCharacteristic batteryLevelChar("2A19",  // standard 16-bit characteristic UUID
    BLERead ); // remote clients will be able to get notifications if this characteristic changes

BLECharacteristic massMoveCharacteristic("A000", BLERead | BLEWrite,5);
BLEDescriptor massMoveCharacDescriptor("2901","Mass Control - S,dir,spd or P,pos");
BLEIntCharacteristic servoMoveCharacteristic("A001", BLERead | BLEWrite);
BLEDescriptor servoMoveCharacDescriptor("2901","Servo Control - angle");
BLECharacteristic pumpMoveCharacteristic("A002", BLERead | BLEWrite,5);
BLEDescriptor pumpMoveCharacDescriptor("2901","Pump Control - S,dir,spd or P,pos");
int oldBatteryLevel = 0;  // last battery level reading from analog input
long previousMillis[] = {0,0,0,0,0};  // last time the battery level was checked, in ms

const int ledPin = LED_BUILTIN; // pin to use for the LED
bool BTConnected = false;
String centralAddress = "";
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
Madgwick filter;
Madgwick filter2;
unsigned long microsPerReading, microsPrevious;

float roll, pitch, heading;
unsigned long microsNow;
  
MS5837 pressSensor;
bool pressSensorConnected = false;

int massPos = 0;
int massSpdFlag = 0;
bool massPosFlag = false;
int enB = 5; //connect to PWM pin for speed control
int in3 = 6; // connect to digital pin
int in4 = 7; // connect to digital pins
int massAn = A2; // analogpin

int pumpPos = 0;
int pumpSpdFlag = 0;
bool pumpPosFlag = false;
int enA = 10; //connect to PWM pin for speed control
int in1 = 9; // connect to digital pin
int in2 = 8; // connect to digital pin
int pumpAn = A1; // analogpin

Servo tail;
const int TAIL_ZERO = 90;
int tailPos = TAIL_ZERO;


void setup() {
  Serial.begin(9600);    // initialize serial communication
  //while (!Serial);
  tail.attach(4);
  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected
  Wire.begin();

  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  pressSensorConnected = pressSensor.init();
//  while (!pressSensorConnected) {
//    Serial.println("Pressure Sensor init failed!");
//    Serial.println("Are SDA/SCL connected correctly?");
//    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
//    Serial.println("\n\n\n");
//    delay(5000);
//    pressSensorConnected = pressSensor.init();
//  }
  if(pressSensorConnected){
    pressSensor.setModel(MS5837::MS5837_30BA);
    pressSensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
  }
  
  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  int samplerate = 100;
  filter.begin(samplerate);
  microsPerReading = 1000000 / samplerate;
  microsPrevious = micros();
  if (!HTS.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }

  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("GSenseAct");
  

  BLE.setAdvertisedService(batteryService); // add the service UUID
  batteryService.addCharacteristic(batteryLevelChar); // add the battery level characteristic
  BLE.addService(batteryService); // Add the battery service
  batteryLevelChar.writeValue(oldBatteryLevel); // set initial value for this characteristic

  BLE.setAdvertisedService(ActService); // add the service UUID
  ActService.addCharacteristic(massMoveCharacteristic); // add characteristic
  massMoveCharacteristic.addDescriptor(massMoveCharacDescriptor);
  ActService.addCharacteristic(servoMoveCharacteristic);
  servoMoveCharacteristic.addDescriptor(servoMoveCharacDescriptor);
  ActService.addCharacteristic(pumpMoveCharacteristic);
  pumpMoveCharacteristic.addDescriptor(pumpMoveCharacDescriptor);
  BLE.addService(ActService); // Add the battery service
  massMoveCharacteristic.setValue("mass"); // set initial value for this characteristic to remote BLE central devices
  pumpMoveCharacteristic.setValue("pump");
 
  BLE.setAdvertisedService(IMUService);
  IMUService.addCharacteristic(IMUGyroCharacteristic);
  IMUGyroCharacteristic.addDescriptor(IMUGyroCharacDescriptor);
  IMUService.addCharacteristic(IMUMagCharacteristic);
  IMUMagCharacteristic.addDescriptor(IMUMagCharacDescriptor);
  IMUService.addCharacteristic(IMUAccelCharacteristic);
  IMUAccelCharacteristic.addDescriptor(IMUAccelCharacDescriptor);
  IMUService.addCharacteristic(IMUYPRCharacteristic);
  IMUYPRCharacteristic.addDescriptor(IMUYPRCharacDescriptor);
  BLE.addService(IMUService);
  
  BLE.setAdvertisedService(SensoryService);
  SensoryService.addCharacteristic(TemperatureCharacteristic); 
  TemperatureCharacteristic.addDescriptor(TemperatureCharacDescriptor);
  SensoryService.addCharacteristic(HumidityCharacteristic); 
  HumidityCharacteristic.addDescriptor(HumidityCharacDescriptor);
  SensoryService.addCharacteristic(PressureCharacteristic);
  PressureCharacteristic.addDescriptor(PressureCharacDescriptor);
  SensoryService.addCharacteristic(DepthCharacteristic);
  DepthCharacteristic.addDescriptor(DepthCharacDescriptor);
  BLE.addService(SensoryService);
  
   // assign event handlers for characteristic
  massMoveCharacteristic.setEventHandler(BLEWritten, massMoveCharacteristicWritten);
  pumpMoveCharacteristic.setEventHandler(BLEWritten, pumpMoveCharacteristicWritten);
  servoMoveCharacteristic.setEventHandler(BLEWritten, servoMoveCharacteristicWritten);
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // start advertising
  BLE.advertise();
  tail.write(TAIL_ZERO);
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  // wait for a BLE central
 // poll for BLE events
  BLE.poll();
  // if a central is connected to the peripheral:
  if (BTConnected) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(centralAddress);
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);
    microsPrevious = micros();
    while (BTConnected) {
      BLE.poll();
      long currentMillis = millis();
      // if x ms have passed, update sensory data:
      if (currentMillis - previousMillis[0] >= 100) {
        previousMillis[0] = currentMillis;
        updateAnalogValues();
        float outTemp = 0;
        float pressureOut = 0;
        if(pressSensorConnected){
          pressSensor.read();
          DepthCharacteristic.setValue(pressSensor.depth());
          //Serial.print("Depth - ");Serial.println(pressSensor.depth());
          outTemp = pressSensor.temperature();
          float pressureConversionFactor = 1;
          pressureOut=pressSensor.pressure(pressureConversionFactor);
        }
        float pressInOut[] = {BARO.readPressure(),pressureOut};
        unsigned char *val = (unsigned char *)&pressInOut;
        PressureCharacteristic.setValue(val,8);
        //float tempInOut[] = {HTS.readTemperature(),outTemp};
        float tempInOut[] = {0,outTemp};
        val = (unsigned char *)&tempInOut;
        TemperatureCharacteristic.setValue(val,8);
        HumidityCharacteristic.setValue(0);//HTS.readHumidity());
        //Serial.println(HumidityCharacteristic.value());
        //Serial.println(HTS.readHumidity());
      }
      if (IMU.accelerationAvailable()) {
        //previousMillis[1] = currentMillis;
        IMU.readAcceleration(ax, ay, az);
        float axyz[] = {ax,ay,az};
        unsigned char *acc = (unsigned char *)&axyz;
        IMUAccelCharacteristic.setValue(acc,12);
        //Serial.print("Accel - ");Serial.print(ax);Serial.print('\t');Serial.print(ay);Serial.print('\t');Serial.println(az);
      }
      if (IMU.gyroscopeAvailable()) {
        //previousMillis[2] = currentMillis;
        IMU.readGyroscope(gx, gy, gz);
        float gxyz[] = {gx,gy,gz}; 
        unsigned char *gyr = (unsigned char *)&gxyz;
        IMUGyroCharacteristic.setValue(gyr,12);
        //Serial.println("Gyro - ");Serial.print(gx);Serial.print('\t');Serial.print(gy);Serial.print('\t');Serial.println(gz);
      }
      if (IMU.magneticFieldAvailable() ) {
        //previousMillis[3] = currentMillis;
        IMU.readMagneticField(mx, my, mz);
        float mxyz[] = {mx,my,mz}; 
        unsigned char *mag = (unsigned char *)&mxyz;
        IMUMagCharacteristic.setValue(mag,12);
        //Serial.println("Mag - ");Serial.print(mx);Serial.print('\t');Serial.print(my);Serial.print('\t');Serial.println(mz);
      }
      microsNow = micros();
      if (microsNow - microsPrevious >= microsPerReading) {
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);
        IMU.readMagneticField(mx, my, mz);
        filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
        roll = filter.getRoll();
        pitch = filter.getPitch();
        heading = filter.getYaw();
        float ypr[] = {heading,pitch,roll};
        unsigned char *eul = (unsigned char *)&ypr;
        IMUYPRCharacteristic.setValue(eul,12);
      }
      pumpHandler();
      massHandler();
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    //Serial.println();
    //Serial.print(ax);Serial.print(",");Serial.print(","); Serial.print(az);
    //Serial.println("-ax,ay,az");
    //Serial.print(gx);Serial.print(","); Serial.print(gy);Serial.print(","); Serial.print(gz);
    //Serial.println("-gx,gy,gz");
    //Serial.print(mx);Serial.print(",");Serial.print(my);Serial.print(",");Serial.print(mz);
    //Serial.println("-mx,my,mz");
    //Serial.print(HTS.readTemperature());
    //Serial.println("-intemp");
    //Serial.print(pressSensor.temperature());
    //Serial.println("-outtemp");
  }
}

void updateAnalogValues() {
  float massPos = analogRead(massAn)/1023.0*100;
  unsigned char *val = (unsigned char *)&massPos;
  massMoveCharacteristic.setValue(val,4);
  
  float pumpPos = analogRead(pumpAn)/1023.0*100;
  val = (unsigned char *)&pumpPos;
  pumpMoveCharacteristic.setValue(val,4);
  
  int battery = analogRead(A0);
  int batteryLevel = map(battery, 0, 1023, 0, 100);
  if (batteryLevel != oldBatteryLevel) {      // if the battery level has changed
    if(abs(batteryLevel-oldBatteryLevel)>3){
      Serial.print("Battery Level % is now: "); // print it
      Serial.println(batteryLevel);
    }
    batteryLevelChar.setValue(batteryLevel);  // and update the battery level characteristic
    oldBatteryLevel = batteryLevel;           // save the level for next comparison
  }
}

void pumpMoveIn(){
  digitalWrite(in1, LOW);  
  digitalWrite(in2, HIGH); 
}
void pumpMoveOut(){
  digitalWrite(in1, HIGH);  
  digitalWrite(in2, LOW); 
}
void pumpStop(){
  digitalWrite(in1, LOW);  
  digitalWrite(in2, LOW);
}
void massMoveIn(){
  digitalWrite(in3, LOW);  
  digitalWrite(in4, HIGH);
}
void massMoveOut(){
  digitalWrite(in3, HIGH);  
  digitalWrite(in4, LOW); 
}
void massStop(){
  digitalWrite(in3, LOW);  
  digitalWrite(in4, LOW);
}

void massHandler(){
  int currentPos = analogRead(massAn);
  int gap = 4;
  if (massPosFlag & (massPos > currentPos)){//retract
    massMoveIn();
    analogWrite(enB,255);
  }
  if(massPosFlag & (massPos < currentPos)){//extend
    massMoveOut();
    analogWrite(enB,255);
  }
  if (massPosFlag & (currentPos <= massPos + gap && currentPos >= massPos - gap)){
    massStop();
    massPosFlag =false;
  }
  if(massSpdFlag<0){
    massMoveOut();
  }
  if(massSpdFlag>0){
    massMoveIn();
  }
  if((massSpdFlag>0) && (currentPos>1020)){
    massStop();
    massSpdFlag=0;
    //Serial.print("Forward stop");
  }
  if((massSpdFlag<0) && (currentPos<4)){
    massStop();
    massSpdFlag=0;
    //Serial.print("Backward stop ");
  }
}

void pumpHandler(){
  int currentPos = analogRead(pumpAn);
  int gap = 4;
  if (pumpPosFlag & (pumpPos > currentPos)){//retract
    pumpMoveIn();
    analogWrite(enA,255);
  }
  if(pumpPosFlag & (pumpPos < currentPos)){//extend
    pumpMoveOut();
    analogWrite(enA,255);
  }
  if (pumpPosFlag & (currentPos <= pumpPos + gap && currentPos >= pumpPos - gap)){
    pumpStop();
    pumpPosFlag =false;
  }
  if(pumpSpdFlag<0){
    pumpMoveIn();
  }
  if(pumpSpdFlag>0){
    pumpMoveOut();
  }
  if(pumpSpdFlag>0 && currentPos>1020){
    pumpStop();
    pumpSpdFlag=0;
  }
  if(pumpSpdFlag<0 && currentPos<4){
    pumpStop();
    pumpSpdFlag=0;
  }
}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  centralAddress=central.address();
  Serial.println(centralAddress);
  BTConnected = true;
  //digitalWrite(ledPin, HIGH);
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  BTConnected = false;
  //digitalWrite(ledPin, LOW);
}

void massMoveCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  unsigned char* cmd = (unsigned char*)massMoveCharacteristic.value();
  //Serial.println((char *)cmd);   
  massSpdFlag=0;
  if(cmd[0]=='S'){
    massPosFlag =false;
    Serial.print("Setting mass speed to ");   
    int spd = *((int *)(cmd+1));//(signed char)cmd[1];
    if (spd>=0){
      analogWrite(enB,spd);
      //massMoveOut();
      massSpdFlag=1;
      Serial.print("Forward ");
    }
    else{
       analogWrite(enB,-spd);
       //massMoveIn();
       massSpdFlag=-1;
        Serial.print("Backward ");
    }
    Serial.println(spd);
  }
  if(cmd[0]=='P'){
    Serial.print("Setting mass position to ");
    massPos=*((float *)(cmd+1))/100.0*1023;
    massPosFlag =true;
    Serial.println(*((float *)(cmd+1)));
  }
  float mPos = analogRead(massAn)/1023.0*100;
  unsigned char *val = (unsigned char *)&mPos;
  massMoveCharacteristic.setValue(val,4);
}

void pumpMoveCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  unsigned char* cmd = (unsigned char*)pumpMoveCharacteristic.value();
  
  //Serial.println((char *)cmd); 
  pumpSpdFlag=0;
  if(cmd[0]=='S'){
    Serial.print("Setting pump speed to ");
    int spd = *((int *)(cmd+1));//(signed char)cmd[1];
    if (spd>=0){
      analogWrite(enA,spd);
      pumpSpdFlag=1;
    }
    else{
       analogWrite(enA,-spd);
       pumpSpdFlag=-1;
    }
    Serial.println(spd);
  }
  if(cmd[0]=='P'){
    //Serial.print("Setting pump position to ");
    pumpPos=*((float *)(cmd+1))/100.0*1023;
    pumpPosFlag =true;
    //Serial.println(*((float *)(cmd+1)));
  }
  float pPos = analogRead(pumpAn)/1023.0*100;
  unsigned char *val = (unsigned char *)&pPos;
  pumpMoveCharacteristic.setValue(val,4);
}

void servoMoveCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  //Serial.print("Setting tail to ");
  //Serial.print(servoMoveCharacteristic.value());
  //Serial.println(" degrees");
  tail.write(TAIL_ZERO+map(servoMoveCharacteristic.value(), -90, 90, -60, 60));
  //tail.write(TAIL_ZERO+servoMoveCharacteristic.value());
}
