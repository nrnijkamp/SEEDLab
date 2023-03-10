// SEED Lab Mini Project Group 3
// Sam Leonard, Dawson J. Gullickson, Julia Kaase

#include <DualMC33926MotorShield.h>
#include <Encoder.h>
#include <Wire.h>

#define PERIPHERAL_ADDRESS 0x08

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knobRight(3, 8);
//   avoid using pins with LEDs attached

DualMC33926MotorShield md;

// PID Control Gains 
double Kp = 15.3786175942488; // V/rad
double Ki = 2.37803426483209; // V/(rad*s)
double Kd = 0;                // V*s/rad

// Given from document
double I = 0; // integral
double D = 0; // derivative
double e_past = 0; // previous value
double Ts = 0; // Time step
double Tc = millis() / 1000.0; // Running time

unsigned int angle = 0; // As a multiple of pi/2
double r = 0; // radians
double umax = 7.8; // max voltage that can be supplied
void setup() {
  // Start serial for output
  Serial.begin(31250);
  
  // Initialize motor
  md.init();

  // Initialize i2c as peripheral
  Wire.begin(PERIPHERAL_ADDRESS);

  // Set I2C callbacks
  Wire.onReceive(receiveData);

  Serial.println("Ready!");
}

void loop() {
  // Convert angle to radians
  r = angle*PI/2;
  // Serial.print("Desired: ");
  // Serial.print(r);
  // Serial.print(" ");

  // Get motor radians
  long newRight = knobRight.read();
  double y = ((double)newRight/3200)*2*PI;
  // Serial.print("Radians: ");
  // Serial.print(y);
  // Serial.print(" ");

  // calc error
  double e = r-y; // find where it needs to move from where it is

  if (Ts > 0) {
    D = (e-e_past)/Ts; // derivative
    e_past = e; // update val to get other vals
  } else {
    D = 0;
  }

  I = I+Ts*e; // integral

  // Calc controller output -- output voltage uses PID
  double u = Kp*e+Ki*I+Kd*D;
  // deals with actuator saturation
  // i.e. if trying to write a voltage too high for board to supply
  if (abs(u) > umax) {
    u = sgn(u)*umax;
    e = sgn(e)*min(umax/Kp, abs(e));
    I = (u-Kp*e-Kd*D)/Ki; 
  }
  
  // Convert voltage to speed
  int speed = -u*400/umax;
  // Serial.print("Speed: ");
  // Serial.println(speed);
  // Set speed
  md.setM1Speed(speed);
  stopIfFault();

  // Output data (part 3)
  double currentTime = millis() / 1000.0;
  Ts = currentTime - Tc;
  Tc = currentTime;
  Serial.print("Time: ");
  Serial.print(currentTime);
  Serial.print("\tAngle: ");
  Serial.println(y);
}

// Get angle from raspberry pi
void receiveData(int _byte_ount) {
  angle = Wire.read();
}

int sgn(double v) {
  if (v >= 0) return 1;
  else return -1;
}

// Halts the program if the motor faults
void stopIfFault() {
  if (md.getFault()) {
    Serial.println("fault");
    while (true);
  }
}