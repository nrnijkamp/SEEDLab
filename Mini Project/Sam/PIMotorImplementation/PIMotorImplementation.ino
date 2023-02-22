//PID Motor Implementation - Mini Project 4.7
//Sam Leonard, Dawson J. Gullickson

#include <Wire.h>
#include "DualMC33926MotorShield.h"

#define PERIPHERAL_ADDRESS 0x08

DualMC33926MotorShield md;

//PID Control Gains 
double Kp = 15.3786175942488;
double Ke = .5;
double Ki = 2.37803426483209;
double Kd = 0;

//Given from document
double I = 0; //integral
double D = 0; //deriv
double e_past = 0; //prev val
double Ts = 0;
double Tc = millis() / 1000;

unsigned int angle = 0; // As a multiple of pi/2s
double r = 0; //radians
double umax = 7.8; //max voltage that can be supplied
void setup() {
  // Start serial for output
  Serial.begin(31250);

  // Initialize i2c as peripheral
  Wire.begin(PERIPHERAL_ADDRESS);

  // Set I2C callbacks
  Wire.onReceive(receiveData);

  Serial.println("Ready!");
}

void loop() {
  //Given pseudocode translated to code

  // Convert angle to radians
  r = angle*PI/2;

  //[read y - where the motor is at] radians, use encoder to find position?
  double y = 0;

  //calc error
  double e = r-y; //find where it needs to move from where it is

  if(Ts>0)
  {
    D = (e-e_past)/Ts; //derivative
    e_past = e; //update val to get other vals
  }
  else 
  {
    D = 0;
  }

  I = I+Ts*e; //integral

  //Calc controller output -- output voltage uses PID
  double u = Kp*e+Ki*I+Kd*D;
  //deals with actuator saturation i.e. if trying to write a voltage too high for board to supply
  if(abs(u)>umax) 
  {
    u = sgn(u)*umax;
    e = sgn(e)*min(umax/Kp, abs(e));
    I = (u-Kp*e-Kd*D)/Ki; 
  }
  
  // Convert voltage to speed
  int speed = u*400/umax;
  // Set speed
  md.setM1Speed(speed);
  stopIfFault();

  //part 3 implementation
  double currentTime = millis() / 1000;
  Ts = currentTime - Tc;
  Tc = currentTime;
  Serial.print("Time: ");
  Serial.print(currentTime);
  Serial.print("\tR: ");
  Serial.println(r);
}

void receiveData(int _byte_ount) {
  angle = Wire.read();
}

int sgn(double v) {
  if (v >= 0) return 1;
  else return -1;
}

void stopIfFault() {
  if (md.getFault()) {
    Serial.println("fault");
    while (true);
  }
}