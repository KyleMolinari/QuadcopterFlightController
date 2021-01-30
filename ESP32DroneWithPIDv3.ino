#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <VL53L1X.h>
#include <ESP32Servo.h>
#include "BluetoothSerial.h"

//Any digital IO pins
#define CE 12
#define CSN 5

//These pins must have PWM capability
#define Motor1 4 
#define Motor2 2
#define Motor3 15
#define Motor4 13
 
#define batteryRead 34 //reads battery level
#define pi 3.14159265359
#define MOTOR_IDLE 1000
#define MOTOR_MIN 1000
#define MOTOR_MAX 2000
long waitTime = 2000; //time (in ms) before autopilot takeover once drone is out of range

Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x28); //ADR pin to ground
VL53L1X ToF; //Time of Flight Sensor
RF24 droneRadio(CE, CSN);
BluetoothSerial SerialBT; //only for debugging - sends motor speed data to Processing via BT

float qw,qx,qy,qz; //quaternion orientation data
float dist;

Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

bool inRange = false;
bool sleeping = false;
bool autoPilot = false;
int time1,time2;
int batterytime1=0, batterytime2;

float M1,M2,M3,M4;
float motorFix = 0;

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

struct pid{
  float input;
  float output;
  float setpoint;
  float P;
  float I;
  float D;
  float oldError;
  float Kp;
  float Ki;
  float Kd;
  float timeold;
  float timenew;
  float deltaT;
  float ratio;
  float offset;
};


//create global instances of data structures 
//these will be updated each time there is new data being sent/received
controls receiveControllerData;
altOri drone;

//create instances of PID controller sturctures
pid Thrust;
pid Yaw;
pid Pitch;
pid Roll;

void setup() {
  Serial.begin(9600);
  SerialBT.begin("Motor Speed BT");
  Wire.begin();
  Wire.setClock(400000);
  ToF.setTimeout(500);

  if(!bno1.begin() || !ToF.init()) 
  {
    SerialBT.print("BNO055 or VL53L1X Error");
    while(1);
  }

  bno1.setExtCrystalUse(true);
  ToF.setDistanceMode(VL53L1X::Long);
  ToF.setMeasurementTimingBudget(50000);
  ToF.startContinuous(50);

  //Set Pins to control PWM input to each ESC
  // Pin , minimum pulse width (us), maximum pulse width (us)
  // Minimum and Maximum pulse widths may need to be calibrated for each individual ESC
  ESC1.attach(Motor1,1000,2000); 
  ESC2.attach(Motor2,1000,2000);
  ESC3.attach(Motor3,1000,2000);
  ESC4.attach(Motor4,1000,2000);

  //Temporary PWM output to LEDs to indicate motor speeds while debugging
  pinMode(Motor1,OUTPUT);
  pinMode(Motor2,OUTPUT);
  pinMode(Motor3,OUTPUT);
  pinMode(Motor4,OUTPUT);

  pinMode(batteryRead, INPUT);

  //Set PID coefficients
  //Thrust
  Thrust.Kp = 0.25; //1
  Thrust.Ki = 0; //0.000001
  Thrust.Kd = 0; //300
  //Pitch
  Pitch.Kp = 0; //10 //1.7
  Pitch.Ki = 0; //0.00001
  Pitch.Kd = 0; //1000
  //Roll
  Roll.Kp = 0; 
  Roll.Ki = 0; 
  Roll.Kd = 0; 
  //Yaw
  Yaw.Kp = 0; 
  Yaw.Ki = 0; 
  Yaw.Kd = 0;
  
  //numerator is the change in pulse width that is sent to the ESCs (eg baseline is 1500ms, and thrust can change that by +-400ms)
  //denominator is the range of values that the PID controller outputs
  Thrust.ratio = (float)1; //400/4000
  Yaw.ratio = (float)1; //100/200
  Pitch.ratio = (float)1; //200/1300
  Roll.ratio = (float)1; //200/1300

  //Offsets based on joystick errors - centred joysick may not be read as exactly 2048 on ADC pin
  Thrust.offset = 0;
  Yaw.offset = 0;
  Pitch.offset = 0;
  Roll.offset = 0;
  
  droneRadio.begin();
  droneRadio.openWritingPipe(address[1]);
  droneRadio.openReadingPipe(1, address[0]);
  droneRadio.setDataRate(RF24_250KBPS);
  droneRadio.setPALevel(RF24_PA_MAX);
  droneRadio.setChannel(0x34);
  droneRadio.enableDynamicPayloads();
  droneRadio.enableAckPayload();
  droneRadio.setRetries(0,4);
  droneRadio.setAutoAck(true);
  droneRadio.powerUp();

}

void loop() {
  int time1 = millis();
  imu::Quaternion quat = bno1.getQuat();
  ToF.read();

  updateOrientation(quat.w(),quat.x(),quat.y(),quat.z(),ToF.ranging_data.range_mm); 
  
  //Send Info Back to Controller  
  droneTransmit(qw,qx,qy,qz,dist, 0); //getBattery()  

  //Receive Info From Controller
  droneReceive(); 

  // Use this line only when PID tuning
  Pitch.Kd = 0.05+((float)receiveControllerData.knob)/4095*50;
  
  //Update Setpoints
  UpdateSetpoints();
  
  //Use PID Controllers
  PIDoutput(&Thrust);
  PIDoutput(&Yaw);
  PIDoutput(&Pitch);
  PIDoutput(&Roll);
  
  //Use Info From Controller to Adjust Motor Speeds
  flightControl();

  //Print Info Received from Controller + Motor Speed Data
  //debugPrint();
}

//Receives Yaw, Pitch, Roll, and Thrust values
void droneReceive(){
  if(droneRadio.available()){
    droneRadio.read(&receiveControllerData, sizeof(receiveControllerData));
    inRange = true;
    sleeping = false;
    autoPilot = false;
    time1 = millis();
    if(receiveControllerData.thrust == -1000){
      powerOff();
      sleeping = true;
    }
    else if(receiveControllerData.thrust == -2000){
      autoPilot = true;
    }
    return;
  }
  time2 = millis();
  if(time2-time1>waitTime){
    inRange = false;
    powerOff();
    return;
  }
}

//Transmits Quaternion Orientation Data and ToF Distance Measurement
void droneTransmit(float w, float x, float y, float z, int d, float b){
  
  drone.quatw = w;
  drone.quatx = x;
  drone.quaty = y;
  drone.quatz = z;
  drone.distance = d;
  drone.battery = b;
  
  droneRadio.stopListening();
  droneRadio.write(&drone, sizeof(drone));
  droneRadio.startListening();
}

void debugPrint(){

//  Serial.print(Thrust.output);
//  Serial.print(", ");
//  Serial.print(Yaw.output);
//  Serial.print(", ");
//  Serial.print(Pitch.output);
//  Serial.print(", ");
//  Serial.print(Roll.output);
//  Serial.println("");

  SerialBT.print(M1);
  SerialBT.print(", ");
  SerialBT.print(M2);
  SerialBT.print(", ");
  SerialBT.print(M3);
  SerialBT.print(", ");
  SerialBT.print(M4);
  SerialBT.print(", ");
  SerialBT.print(sleeping);
  SerialBT.print(", ");
  SerialBT.print(!inRange);
  SerialBT.print(", ");
  SerialBT.print(time2-time1);
  SerialBT.println("");
}

void flightControl(){
    /*  
   *   
   *    Need to use the global data structure receiveControllerData to control the motors
   *      
   *    Drone Layout - Top View - X-Frame Quadcopter Design
   *    
   *                  Front
   *    (CCW) Motor1  O  O  Motor2 (CW)
   *                   \/              
   *                   /\ 
   *     (CW) Motor3  O  O  Motor4 (CCW)
   *                  Back
   *                
   *    
   *    Thrust is positively proportional to all 4 motor speeds
   *    Yaw is positively proportional to the speed of motors 1 and 4 and negatively proportional to the speed of motors 2 and 3 (or vice versa)
   *    Pitch is positively proportional to the speed of motors 1 and 2 and negatively proportional to the speed of motors 3 and 4 (or vice versa)
   *    Roll is positively proportional to the speed of motors 1 and 3 and negatively proportional to the speed of motors 2 and 4 (or vice versa)
   *            
   */ 
  
  if(sleeping || !inRange){ 
    M1 = 0;
    M2 = 0;
    M3 = 0;
    M4 = 0;

    ESC1.write(M1);
    ESC2.write(M2);
    ESC3.write(M3);
    ESC4.write(M4);
  
    return;
  }
  else{
    M1 = MOTOR_IDLE+Thrust.output+Yaw.output+Pitch.output-Roll.output;
    M2 = MOTOR_IDLE+Thrust.output-Yaw.output+Pitch.output+Roll.output;
    M3 = MOTOR_IDLE+Thrust.output-Yaw.output-Pitch.output-Roll.output;
    M4 = MOTOR_IDLE+Thrust.output+Yaw.output-Pitch.output+Roll.output;

    float motormin = minimum(M1,M2,M3,M4);
    float motormax = maximum(M1,M2,M3,M4);
    
    motorFix=0;
    if(motormin < MOTOR_MIN) motorFix = MOTOR_MIN - motormin;
    else if(motormax > MOTOR_MAX) motorFix = MOTOR_MAX - motormax;

    M1=M1+motorFix;
    M2=M2+motorFix;
    M3=M3+motorFix;
    M4=M4+motorFix;
    
    ESC1.write(M1);
    ESC2.write(M2);
    ESC3.write(M3);
    ESC4.write(M4);
  }
}

void updateOrientation(float q0, float q1, float q2, float q3, float vertDistance){
  qw = q0;
  qx = q1;
  qy = q2;
  qz = q3;
  dist = vertDistance;
}

//float getBattery(){
//  //4S lipo has max charge of 16.8V and min charge of 14.8V
//  //Use a voltage divider of 100k and 20k ohm resistors to step the voltage down to be read by the ADC
//  //only need to check battery every 10 seconds, no need to constantly check multiple times per second
//  //This returns the battery percentage (0-100) to the nearest 5%, or returns -1 if the ADC is waiting/not working
//  batterytime2 = millis();
//  if(batterytime2-batterytime1>10000){
//    batterytime1 = millis();
//    double reading = analogRead(batteryRead);
//    if(reading<1 || reading >4095) return -1;
//    double ADCvoltage = -0.000000000000016*pow(reading,4)+0.000000000118171*pow(reading,3)-0.000000301211691*pow(reading,2)+0.001109019271794*reading+0.034143524634089; //polynomial accurate within 1%
//    double batteryVoltage = ADCvoltage*6;
//    if(batteryVoltage > 16.8){
//      return 100;
//    }
//    else if(batteryVoltage > 16.6){
//      return 95;
//    }
//    else if(batteryVoltage > 16.45){
//      return 90;
//    }
//    else if(batteryVoltage > 16.33){
//      return 85;
//    }
//    else if(batteryVoltage > 16.09){
//      return 80;
//    }
//    else if(batteryVoltage > 15.93){
//      return 75;
//    }
//    else if(batteryVoltage > 15.81){
//      return 70;
//    }
//    else if(batteryVoltage > 15.66){
//      return 65;
//    }
//    else if(batteryVoltage > 15.5){
//      return 60;
//    }
//    else if(batteryVoltage > 15.42){
//      return 55;
//    }
//    else if(batteryVoltage > 15.34){
//      return 50;
//    }
//    else if(batteryVoltage > 15.26){
//      return 45;
//    }
//    else if(batteryVoltage > 15.18){
//      return 40;
//    }
//    else if(batteryVoltage > 15.14){
//      return 35;
//    }
//    else if(batteryVoltage > 15.06){
//      return 30;
//    }
//    else if(batteryVoltage > 14.99){
//      return 25;
//    }
//    else if(batteryVoltage > 14.91){
//      return 20;
//    }
//    else if(batteryVoltage > 14.83){
//      return 15;
//    }
//    else if(batteryVoltage > 14.75){
//      return 10;
//    }
//    else if(batteryVoltage > 14.43){
//      return 5;
//    }
//    else{
//      return 0;
//    }
//  }
//  //if it has not been 10s since the last measurement, return -1
//  return -1;
//}

void powerOff(){
  ESC1.write(0);
  ESC2.write(0);
  ESC3.write(0);
  ESC4.write(0);
  M1=0;
  M2=0;
  M3=0;
  M4=0;
}

float mod(float a, float b){
  int sign = a>0?1:-1;
  a=sign*a;
  while(a>=b){
    a=a-b;
  }
  return sign*a;
}

void PIDoutput(struct pid *instance){
  instance->timeold = instance->timenew;
  instance->timenew = millis();
  instance->deltaT = instance->timenew-instance->timeold;
  
  float error = instance->setpoint-instance->input;
  
  instance->P = (instance->Kp)*(error);
  instance->I = instance->I+(instance->Ki)*(error)*(instance->deltaT);
  instance->D=(instance->Kd)*(error-instance->oldError)/(instance->deltaT);

  //set I term to 0 to limit overshoot if the setpoint has been reached
  if(error*(instance->oldError)<=0){
    instance->I=0;
  }
  
  instance->oldError=error;

  instance->output=(instance->ratio)*(instance->P+instance->I+instance->D)+instance->offset;
  
}

void UpdateSetpoints(){
  if(autoPilot){
    Thrust.setpoint = 1000; //choose to fly at an altitude of 1m
    Pitch.setpoint = 0;
    Roll.setpoint = 0;
    Yaw.setpoint = Yaw.input;
    //when autopilot is enabled use altitude to control thrust instead of using controller input
    Thrust.input = dist*4095/4000;
  }
  else if(!inRange){
    Thrust.setpoint = Thrust.setpoint>0?Thrust.setpoint-1:0; //gradually drop altitude target down to 0
    Pitch.setpoint = 0;
    Roll.setpoint = 0;
    Yaw.setpoint = Yaw.input;
    //when not in range 
    Thrust.input = dist*4095/4000;
  }
  else{
    Thrust.setpoint = receiveControllerData.thrust; 
    Pitch.setpoint = receiveControllerData.pitch*45/2048; //scale down pitch to range from +- 45 degrees
    Roll.setpoint = receiveControllerData.roll*45/2048; //scale down roll to range from +- 45 degrees
    Yaw.setpoint = mod(Yaw.input+receiveControllerData.yaw*45/2048,360); //scale down yaw to range from +- 10 degrees
    Thrust.input = 0; // When controlling the drone manually, use only the controller setpoint to control thrust
  }

  //These approximations only work for angles <90 degrees
  Pitch.input = asin(2*(qw*qy-qz*qx))*180/pi; //give PID the current pitch angle as input
  Roll.input = atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy))*180/pi; //give PID the current roll angle as input
  Yaw.input = atan2(2*(qy*qz+qw*qx),-1+2*(qw*qw+qx*qx))*180/pi; //give PID the current yaw angle as input
}

float maximum(float a, float b, float c, float d){
  float maximum = a>b?a:b;
  maximum = maximum>c?maximum:c;
  maximum = maximum>d?maximum:d;
  return maximum;
}

float minimum(float a, float b, float c, float d){
  float minimum = a<b?a:b;
  minimum = minimum<c?minimum:c;
  minimum = minimum<d?minimum:d;
  return minimum;
}
