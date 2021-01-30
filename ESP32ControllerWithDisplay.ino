//CONTROLLER IS ARDUINO UNO ON COM 8

#include <Wire.h>
#include <SPI.h>
//#include <nRF24L01.h>
#include <RF24.h>
#include "BluetoothSerial.h"
#include "TFT_eSPI.h"
TFT_eSPI tft = TFT_eSPI();

#define powerswitch 33
#define pot 32
#define zerobutton 2 // Push Button Input to Calibrate Orientation
#define autoToggle 27 // Switch to toggle autonomous flight
bool debugoverride = false;
#define T 37     // Thrust - Left Joystick Y Axis
#define Y 38        // Yaw - Left Joystick X Axis
#define P 35      // Pitch - Right Joystick Y Axis
#define R 34       // Roll - Right Joystick X Axis
#define nrfCE 12     
#define nrfCSN 5

#define pi 3.14159265359

/*
 * Wiring ILI9341 Display:
 * RESET -> 13
 * CS -> 0
 * DC -> 4
 * VCC -> 5V
 * GND -> GND
 * LED -> 3V3
 * SCK -> 18
 * MOSI -> 23
 * MISO -> NC
*/

RF24 controllerRadio(nrfCE,nrfCSN); // CE, CSN
BluetoothSerial SerialBT;


float freq = 7; //frequency of sending serial data back to matlab to be plotted
float deltaT = 1000/freq;
int reset = 0;
bool powerOn;
bool autoPilot;
int time1,time2;

int Yinv=1,Pinv=1,Rinv=1; //used to invert joystick axes - need to add functionality to change these later

int TCali = 0,YCali = 0,PCali = 0,RCali = 0; //initialize calibration offsets for Thrust, Yaw, Pitch, Roll analog inputs

// need 2 pipe addresses for 2 way communications
// 00001 - used to send data from Controller to Drone
// 00002 - used to send data from Drone to Controller
const byte address[][6] = {0xF0F0F0F0E1LL,0xF0F0F0F0D2LL};

//Data structure to send altitude and orientation data from the drone to the controller
struct altOri{
  float quatw;
  float quatx;
  float quaty;
  float quatz;
  int distance;
  float battery;
};

//Data structure to send input data from the controller to the drone
struct controls{
  int yaw;
  int pitch;
  int roll;
  int thrust;
  int knob;
};

//create global instances of data structures 
//these will be updated each time there is new data being sent/received
controls controllerInput;
altOri receiveDroneData;

void setup() {
  SerialBT.begin("RC Controller");
  Serial.begin(9600);
  tft.init();
  setDisplay();
  
  pinMode(powerswitch, INPUT);
  pinMode(zerobutton, INPUT);
  pinMode(autoToggle, INPUT);
  pinMode(pot, INPUT);
  pinMode(T, INPUT);
  pinMode(Y, INPUT);
  pinMode(P, INPUT);
  pinMode(R, INPUT);
  
  delay(1000);
  TCali = analogRead(T);
  YCali = analogRead(Y);
  PCali = analogRead(P);
  RCali = analogRead(R);
  
  controllerRadio.begin();
  controllerRadio.openWritingPipe(address[0]);
  controllerRadio.openReadingPipe(1, address[1]);
  controllerRadio.setDataRate(RF24_250KBPS);
  controllerRadio.setPALevel(RF24_PA_MAX);
  controllerRadio.setChannel(0x34);
  controllerRadio.enableDynamicPayloads();
  controllerRadio.enableAckPayload();
  controllerRadio.setRetries(0,4);
  controllerRadio.setAutoAck(true);
  controllerRadio.powerUp();
  
  time1 = millis();
}

void loop() {
  reset = 0;
  //If zero button is pressed this will reset the midpoints for the joysticks and reset the orientation of the MATLAB plot
  Calibrate();
    
  //Send Info to Drone
  powerOn = digitalRead(powerswitch);
  autoPilot = digitalRead(autoToggle);
  if(!debugoverride && !powerOn){
    controllerTransmit(-1000,0,0,0,0); //set thrust to -1000 to indicate that controller power is off
  }
  else if(debugoverride || !autoPilot){
    controllerTransmit(getThrust(), getYaw(), getPitch(), getRoll(), getPot());  
  }
  else{
    controllerTransmit(-2000,0,0,0,0); // set thrust to -2000 to indicate that autopilot is enabled
  }

  //Receive Info From Drone
  controllerReceive();

  time2 = millis();
  
  //Send Orientation & Distance Data to the PC to be plotted in Matlab
  //sendBTSerial(); 

  //update TFT display
  //updateAltitudeDisplay();
  //updateBatteryDisplay();
  //updateJoystickDisplay();

  //debugPrint();
  
}

//Receives Quaternion Orientation Data and ToF Distance Measurement
void controllerReceive(){
  if (controllerRadio.available()) {
    controllerRadio.read(&receiveDroneData, sizeof(receiveDroneData));
  }
}

//Transmits Yaw, Pitch, Roll, and Thrust values 
void controllerTransmit(int t, int y, int p, int r, int k){
  
  controllerInput.thrust = t;
  controllerInput.yaw = y;
  controllerInput.pitch = p;
  controllerInput.roll = r;
  controllerInput.knob = k;
  
  controllerRadio.stopListening();
  controllerRadio.write(&controllerInput, sizeof(controllerInput));
  //delay(5);
  controllerRadio.startListening();
}


int getThrust(){
  return analogRead(T);
}

int getYaw(){
  return Yinv*(-analogRead(Y)+YCali);
}

int getPitch(){
  return Pinv*(analogRead(P)-PCali);
}

int getRoll(){
  return Rinv*(analogRead(R)-RCali);
}

int getPot(){
  return 4095-analogRead(pot);
}

void Calibrate(){
  if(digitalRead(zerobutton) == HIGH && !debugoverride){
    reset = 1;
    TCali = analogRead(T);
    YCali = analogRead(Y);
    PCali = analogRead(P);
    RCali = analogRead(R);
  }
  else{
    return;
  }
}

void sendBTSerial(){
  if(time2-time1>=deltaT){
    SerialBT.print(receiveDroneData.quatw);
    SerialBT.print(",");
    SerialBT.print(receiveDroneData.quatx);
    SerialBT.print(",");
    SerialBT.print(receiveDroneData.quaty);
    SerialBT.print(",");
    SerialBT.print(receiveDroneData.quatz);
    SerialBT.print(",");
    SerialBT.print(receiveDroneData.distance);
    SerialBT.print(",");
    SerialBT.println(reset);
    time1 = millis();
  }
}

void debugPrint(){
  Serial.print(getThrust());
  Serial.print(", ");
  Serial.print(getYaw());
  Serial.print(", ");
  Serial.print(getPitch());
  Serial.print(", ");
  Serial.print(getRoll());
  Serial.print(", ");
  Serial.print(getPot());
  Serial.print(", ");
  Serial.print(digitalRead(powerswitch));
  Serial.println("");
}

void setDisplay(){
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(25,25);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.print("Altitude: ");
  tft.setCursor(25,50);
  tft.print("Battery: ");
  tft.setCursor(25,75);
  tft.print("Thrust: ");
  tft.setCursor(25,100);
  tft.print("Yaw: ");
  tft.setCursor(25,125);
  tft.print("Pitch: ");
  tft.setCursor(25,150);
  tft.print("Roll: ");
  tft.setCursor(25,175);
  tft.setTextColor(TFT_BLUE);
  tft.print("Actual Pitch: ");
  tft.setCursor(25,200);
  tft.print("Actual Roll: ");
  tft.setCursor(25,225);
  tft.setTextColor(TFT_WHITE);
  tft.print("ADC: ");
}

void updateAltitudeDisplay(){
  tft.setRotation(2);
  tft.fillRect(90,25,50,10,TFT_BLACK);
  tft.setCursor(90,25);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.print(receiveDroneData.distance/10);
  tft.print(" cm");
}
void updateBatteryDisplay(){
  if(receiveDroneData.battery == -1) return;
  tft.setRotation(2);
  tft.fillRect(90,50,50,10,TFT_BLACK);
  if(receiveDroneData.battery > 80){
    tft.setTextColor(TFT_GREEN);
  }
  else if(receiveDroneData.battery > 35){
    tft.setTextColor(TFT_GREEN);
  }
  else{
    tft.setTextColor(TFT_RED);  
  }
  tft.setTextSize(1);
  tft.setCursor(90,50);
  tft.print(receiveDroneData.battery);
  tft.print("%");
}

void updateJoystickDisplay(){
  tft.setRotation(2);
  tft.fillRect(85,75,55,95,TFT_BLACK);
  tft.fillRect(120,175,70,65,TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);
  tft.setCursor(90,75);
  tft.print(controllerInput.thrust*100/4095);
  tft.print(" %");
  tft.setCursor(90,100);
  tft.print(controllerInput.yaw*100/4095);
  tft.print(" %");
  tft.setCursor(90,125);
  tft.print(-controllerInput.pitch*100/4095);
  tft.print(" %");
  tft.setCursor(90,150);
  tft.print(controllerInput.roll*100/4095);
  tft.print(" %");
  tft.setCursor(130,175); 
  tft.setTextColor(TFT_BLUE); 
  tft.print(asin(2*(receiveDroneData.quatw*receiveDroneData.quaty-receiveDroneData.quatz*receiveDroneData.quatx))*180/pi); //pitch
  tft.print(" deg");
  tft.setCursor(130,200);
  tft.print(atan2(2*(receiveDroneData.quatw*receiveDroneData.quatx+receiveDroneData.quaty*receiveDroneData.quatz),1-2*(receiveDroneData.quatx*receiveDroneData.quatx+receiveDroneData.quaty*receiveDroneData.quaty))*180/pi); //roll
  tft.print(" deg");
  tft.setCursor(130,225);
  tft.setTextColor(TFT_WHITE);
  tft.print(getPot()); //roll
  
}
