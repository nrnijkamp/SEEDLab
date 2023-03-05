#include <DualMC33926MotorShield.h>
#include <Encoder.h>
#include <Wire.h>

Encoder knobRight(3, 8);
//encoder knobLeft()
DualMC33926MotorShield md;

double Kp1 = 55.2518209912998 // V/rad - get from simulink models - First outerloop PD control
double Ki1 = 0 // V/(rad*s)
double Kd1 = 1.74562338241798;                // V*s/rad
  

double Kp2 = 44.9539024669916 // V/rad - get from simulink  - Second outerloop PD control
double Ki2 = 0 // V/(rad*s)
double Kd2 = 2.54579413878844;                // V*s/rad

double I = 0; // integral
double D = 0; // derivative
double e1_past = 0; // previous value
double e2_past = 0; // previous value
double Ts = 0; // Time step
double Tc = millis() / 1000.0; // Running time

unsigned int angle = 0; // As a multiple of pi/2
double r=0; // radians
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
  
  //take in motor1 voltage as input
  //motor2 voltage input

  r = angle*PI/2;
  long newRight = knobRight.read();
  double y = ((double)newRight/3200)*2*PI;

  double e1 = r-y; // find where it needs to move from where it is

//Do PID for first controller
  if (Ts > 0) {
  D = (e1-e1_past)/Ts; // derivative implementation
  e1_past = e1; // update val to get other vals
  } else {
    D = 0;
  }
  I = I+Ts*e2; // integral implementation

  // Calc controller output -- output voltage uses PID
  double uM1 = Kp1*e1+Ki1*I+Kd1*D;
  double uM2=Kp1*e1+Ki1*I+Kd1*D;



//Do PID for second controller
  if (Ts > 0) {
  D = (e1-e2_past)/Ts; // derivative implementation
  e2_past = e2; // update val to get other vals
  } else {
    D = 0;
  }
  I = I+Ts*e1; // integral implementation

   uM1 = Kp2*e1+Ki2*I+Kd2*D;
   uM2=Kp2*e1+Ki2*I+Kd2*D;

//Saturation Checking
   if (abs(uM1) > umax) {
    u = sgn(uM1)*umax;
    e = sgn(e1)*min(umax/Kp, abs(e1));
    IM1 = (uM1-Kp2*e1-Kd2*D)/Ki2; 
  }


   if (abs(uM2) > umax) {
    u = sgn(uM2)*umax;
    e = sgn(e1)*min(umax/Kp, abs(e1));
    IM2 = (uM2-Kp2*e1-Kd2*D)/Ki2; 
  }

  // Convert voltage to speed
  int speedM1 = -uM1*400/umax;
  int speedM2 = uM2*400/umax;
  // Serial.print("Speed: ");
  // Serial.println(speed);
  // Set speed
  md.setM1Speed(speed);
  stopIfFault();









}


int sgn(double v) {
  if (v >= 0) return 1;
  else return -1;
}
void stopIfFault()
{
  if (md.getFault())
  {
    Serial.println("fault");
    while(1);
  }
}

