#include <DualMC33926MotorShield.h>
#include <Encoder.h>
#include <Wire.h>

#define PERIPHERAL_ADDRESS 0x08

Encoder knobLeft(2, 6);
Encoder knobRight(3, 8);

DualMC33926MotorShield md;

// PID variables
double Kp_phi = 55.2518209912998  // Hz - get from simulink models - First outerloop PD control
double Ki_phi = 0;                // Hz^2
double Kd_phi = 1.74562338241798; // [unit]
double I_phi = 0; // integral
double D_phi = 0; // derivative
double e_phi_past = 0; // previous value
  
double Kp_phi_dot = 44.9539024669916  // V*s/rad - get from simulink  - Second outerloop PD control
double Ki_phi_dot = 0;                // V/rad
double Kd_phi_dot = 2.54579413878844; // V*s^2/rad
double I_phi_dot = 0; // integral
double D_phi_dot = 0; // derivative
double e_phi_dot_past = 0; // previous value
  
double Kp_rho_dot = 44.9539024669916  // V*s/m - get from simulink  - Second outerloop PD control
double Ki_rho_dot = 0;                // V/m
double Kd_rho_dot = 2.54579413878844; // V*s^2/m
double I_rho_dot = 0; // integral
double D_rho_dot = 0; // derivative
double e_rho_dot_past = 0; // previous value

double Ts = 0; // Time step
// double Tc = millis() / 1000.0; // Running time

// Max voltage that can be supplied
double umax = 7.8;

// encoder for left wheel and right wheel 
int clkLeft = 2;
int dtLeft = 6;
int clkRight = 3;
int dtRight = 8;

//initialize variables
float r = 7.5;  // radius of the wheel
float d = 28; // distance between the wheels
float dRight, dLeft = 0;
float velLeft, velRight = 0;
float deltaTLeft, deltaTRight = 0;

int toldLeft, tnewLeft, toldRight, tnewRight = 0;
int previousStateLeft, currentStateLeft;
int previousStateRight, currentStateRight;

void setup() {
  pinMode(clkLeft, INPUT_PULLUP); 
  pinMode(dtLeft, INPUT_PULLUP);
  pinMode(clkRight, INPUT_PULLUP);
  pinMode(dtRight, INPUT_PULLUP);
  previousStateLeft = digitalRead(clkLeft); 
  previousStateRight = digitalRead(clkRight);
  attachInterrupt(digitalPinToInterrupt(clkLeft), changePinALeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(clkRight), changePinARight, CHANGE);
  
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

// correlates to left wheel
void changePinALeft() { 
  tnewLeft = millis();
  deltaTLeft = tnewLeft - toldLeft;
  currentStateLeft = digitalRead(clkLeft);

  //verify which direction, if B changes, it is CCW
if(currentStateLeft != previousStateLeft) {
  if (digitalRead(dtLeft) != currentStateLeft) {
    velLeft = r*((4*PI)/(deltaTLeft*30));
  } else {
    velLeft = -r*((4*PI)/(deltaTLeft*30));
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
    velRight = r*((4*PI)/(deltaTRight*30));
  } else {
    velRight = -r*((4*{PI})/(deltaTRight*30));
  }
  previousStateRight = currentStateRight;
  dRight = velRight*deltaTRight;
  output();
  toldRight = tnewRight;
 }
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

void loop() {
  //TODO get desired angle
  double phi_desired = 0;

  //TODO get desired forward speed
  double rho_dot_desired = 0;

  // Get motor radians
  long ticks1 = knobLeft.read(); //NOTE left may be 2
  long ticks2 = knobRight.read(); //NOTE right may be 1
  double theta1 = ((double)ticks1/3200)*2*PI;
  double theta2 = ((double)ticks2/3200)*2*PI;
  
  // Get motor radian speed
  double theta1_dot = velLeft;
  double theta2_dot = velRight;

  // Calculate phi
  double phi = r*(theta1 - theta2)/d;


  // Calculate error
  double e_phi = phi_des - phi;

  // PID for phi
  if (Ts > 0) {
    D_phi = (e_phi-e_phi_past)/Ts; // derivative implementation
    e_phi_past = e_phi; // update val to get other vals
  } else {
    D_phi = 0;
  }
  I_phi = I_phi + Ts*e_phi; // integral implementation

  // PID Output
  double phi_dot_des = Kp_phi*e_phi + Ki_phi*I_phi + Kd_phi*D_phi;


  // Get phi_dot
  phi_dot = r*(theta1_dot - theta2_dot)/d;

  // Calculate error
  double e_phi_dot = phi_dot_desired - phi_dot;
  
  // PID for phi_dot
  if (Ts > 0) {
    D_phi_dot = (e_phi_dot-e_phi_dot_past)/Ts; // derivative implementation
    e_phi_dot_past = e_phi_dot; // update val to get other vals
  } else {
    D_phi_dot = 0;
  }
  I_phi_dot = I_phi_dot+Ts*e_phi_dot; // integral implementation


  // Get rho_dot
  double rho_dot = r*(theta1_dot + theta2_dot)/d;

  // Calculate error
  double e_rho_dot = rho_dot_desired - rho_dot

  // PID for rho_dot
  if (Ts > 0) {
    D_rho_dot = (e_rho_dot-e_rho_dot_past)/Ts; // derivative implementation
    e_rho_dot_past = e_rho_dot; // update val to get other vals
  } else {
    D_rho_dot = 0;
  }
  I_rho_dot = I_rho_dot + Ts*e_rho_dot; // integral implementation

  // PID Outputs
  double u_diff = Kp_phi_dot*e_phi_dot + Ki_phi_dot*I_phi_dot + Kd_phi_dot*D_phi_dot;
  double u_bar = Kp_rho_dot*e_rho_dot + Ki_rho_dot*I_rho_dot + Kd_rho_dot*D_rho_dot;

  // Saturation Checking
  // (u_diff can't be more than twice the range)
  if (abs(u_diff) > 2*umax) {
    u_diff = sgn(u_diff)*2*umax;
    e_phi_dot = sgn(e_phi_dot)*min(umax/Kp_phi_dot, abs(e_phi_dot));
    I_phi_dot = (u_diff-Kp_phi_dot*e_phi_dot-Kd_phi_dot*D_phi_dot)/Ki_phi_dot; 
  }
  if (abs(uM2) > umax) {
    u_bar = sgn(u_diff)*2*umax;
    e_rho_dot = sgn(e_rho_dot)*min(umax/Kp_rho_dot, abs(e_rho_dot));
    I_rho_dot = (u_diff-Kp_rho_dot*e_rho_dot-Kd_rho_dot*D_rho_dot)/Ki_rho_dot;
  }


  // Convert u_bar and u_diff to motor voltages
  uM1 = (u_bar + u_diff)/2;
  uM2 = (u_bar - u_diff)/2;

  // Convert to speeds and send to motor
  int speed1 = -uM1*400/umax;
  int speed2 = uM2*400/umax;
  md.setM1Speed(speed1);
  md.setM2Speed(speed2);
  stopIfFault();
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