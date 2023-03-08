#include <Encoder.h>

// encoder for left wheel and right wheel 
int clkLeft = 2;
int dtLeft = 6;
int clkRight = 3;
int dtRight = 8;

//initialize variables
float r = 7.5;  // radius of the wheel
float d = 28; // distance between the wheels
float dRight, dLeft = 0;
float xnew, xold, phiold, velLeft, velRight, ynew, yold, phinew = 0;
float deltaTLeft, deltaTRight = 0;
float rhoDot, phiDot = 0;

double Kp = 15.3786175942488; // V/rad
double Ki = 2.37803426483209; // V/(rad*s)
double Kd = 0;                // V*s/rad
double umax = 7.8; // max voltage that can be supplied

int phiDotDes, rhoDotDes = 0;
int toldLeft, tnewLeft, toldRight, tnewRight = 0;
int previousStateLeft, currentStateLeft;
int previousStateRight, currentStateRight;
int count = 0;
int radians = 0;

void setup() {
  Serial.begin(9600);
  pinMode(clkLeft, INPUT_PULLUP); 
  pinMode(dtLeft, INPUT_PULLUP);
  pinMode(clkRight, INPUT_PULLUP);
  pinMode(dtRight, INPUT_PULLUP);
  previousStateLeft = digitalRead(clkLeft); 
  previousStateRight = digitalRead(clkRight);
  attachInterrupt(digitalPinToInterrupt(clkLeft), changePinALeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(clkRight), changePinARight, CHANGE);
}

// correlates to left wheel
void changePinALeft() { 
  tnewLeft = millis();
  deltaTLeft = tnewLeft - toldLeft;
  currentStateLeft = digitalRead(clkLeft);

  //verify which direction, if B changes, it is CCW
if(currentStateLeft != previousStateLeft) {
    if (digitalRead(dtLeft) != currentStateLeft) {
      velLeft = r*((4*3.14)/(deltaTLeft*30));
    } else {
      velLeft = -r*((4*3.14)/(deltaTLeft*30));
    }
  previousStateLeft = currentStateLeft;
  dLeft = velLeft*deltaTLeft;
  output();
  toldLeft = tnewRight;
 }
}

//correlates with right wheel
void changePinARight() { 
  tnewRight = millis();
  deltaTRight = tnewRight - toldRight;
  currentStateRight = digitalRead(clkRight);

  //verify which direction, if B changes, it is CCW
if(currentStateRight != previousStateRight) {
    if (digitalRead(dtRight) != currentStateRight) {
      velRight = r*((4*3.14)/(deltaTRight*30));
    } else {
      velRight = -r*((4*3.14)/(deltaTRight*30));
    }
  previousStateRight = currentStateRight;
  dRight = velRight*deltaTRight;
  output();
  toldRight = tnewRight;
 }
}

void loop() {
  // resets both knobs to zero if a character is sent to the serial monitor
  ephi = phiDotDes - phiDot;
  erho = rhoDotDes - rhoDot;
  if (Serial.available()) {
    Serial.read();
    Serial.println(" Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }

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
  uBar
  uDiff
  uM1 = (uBar + uDiff)/2;
  uM2 = (uBar - uDiff)/2;
  int speed1 = -uM1*400/umax;
  int speed2 = uM2*400/umax;
  md.setM1Speed(speed1);
  md.setM2Speed(speed2);
  stopIfFault();
}

void output() {
  velLeft = velLeft*100000;
  velRight = velRight*100000;
  rhoDot = (r*(velLeft + velRight))/2;
  phiDot = (r*(velLeft - velRight))/d;
  Serial.print("Forward Velocity: ");
  Serial.print(rhoDot);
  Serial.pring('\t');
  Serial.print("Rotational Velocity: ");
  Serial.print(phiDot);
  Serial.print('\t');
}

void stopIfFault() {
  if (md.getFault()) {
    Serial.println("fault");
    while (true);
  }
}
