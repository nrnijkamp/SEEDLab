#include <DualMC33926MotorShield.h>
#include <Encoder.h>
// #include <Wire.h>

// #define PERIPHERAL_ADDRESS 0x08

Encoder knobLeft(2, 6);
Encoder knobRight(3, 8);

DualMC33926MotorShield md;

// PID variables
double Kp_phi = 55.2518209912998; // Hz - get from simulink models - First outerloop PD control
double Ki_phi = 0;                // Hz^2
double Kd_phi = 1.74562338241798; // [unit]
double I_phi = 0; // integral
double D_phi = 0; // derivative
double e_phi_past = 0; // previous value
  
double Kp_phi_dot = 44.9539024669916; // V*s/rad - get from simulink  - Second outerloop PD control
double Ki_phi_dot = 0;                // V/rad
double Kd_phi_dot = 2.54579413878844; // V*s^2/rad
double I_phi_dot = 0; // integral
double D_phi_dot = 0; // derivative
double e_phi_dot_past = 0; // previous value
  
double Kp_rho_dot = 44.9539024669916; // V*s/m - get from simulink  - Second outerloop PD control
double Ki_rho_dot = 0;                // V/m
double Kd_rho_dot = 2.54579413878844; // V*s^2/m
double I_rho_dot = 0; // integral
double D_rho_dot = 0; // derivative
double e_rho_dot_past = 0; // previous value

double Ts = 0.1; // Time step
double Tc = millis() / 1000.0; // Running time

// Max voltage that can be supplied
const double umax = 7.8;

//initialize variables
const float r = 7.5;  // radius of the wheel
const float d = 28; // distance between the wheels

void setup() {
  // Start serial for output
  Serial.begin(31250);
  
  // Initialize motor
  md.init();

  // // Initialize i2c as peripheral
  // Wire.begin(PERIPHERAL_ADDRESS);

  // // Set I2C callbacks
  // Wire.onReceive(receiveData);

  Serial.println("Ready!");
}

// ---== TASKS ==---
const double turn_to_angle = 0; // radians
const double forward_distance = 121.92; // cm
const double move_time = 3; // s
bool should_turn = false;
bool should_move = true;

bool is_moving = false;
double end_time = 0;

double theta1_old = 0;
double theta2_old = 0;

void loop() {
  // Set phi_desired based on task settings
  double phi_desired;
  if (should_turn) {
    phi_desired = turn_to_angle;
  } else {
    phi_desired = 0;
  }


  // //TODO get desired angle
  // double phi_desired = 0;

  // //TODO get desired forward speed
  // double rho_dot_desired = 0;

  // Get motor radians
  long ticks1 = knobLeft.read(); //NOTE left may be 2
  long ticks2 = knobRight.read(); //NOTE right may be 1
  double theta1 = -((double)ticks1/3200)*2*PI;
  // double theta2 = ((double)ticks2/3200)*2*PI;
  double theta2 = theta1;
  Serial.print("\tTheta1: ");
  Serial.print(theta1);
  Serial.print("\tTheta2: ");
  Serial.print(theta2);
  
  // Get motor radian speed
  double theta1_dot = (theta1 - theta1_old)/Ts;
  double theta2_dot = (theta2 - theta2_old)/Ts;
  // Serial.print("\tTheta1_dot: ");
  // Serial.print(theta1_dot);
  // Serial.print("\tTheta2_dot: ");
  // Serial.print(theta2_dot);


  // Calculate phi
  double phi = r*(theta1 - theta2)/d;
  // Serial.print("\tPhi: ");
  // Serial.print(phi);
  // Serial.print("\tPhi_des: ");
  // Serial.print(phi_desired);

  // Calculate error
  double e_phi = phi_desired - phi;

  // PID for phi
  if (Ts > 0) {
    D_phi = (e_phi-e_phi_past)/Ts; // derivative implementation
  } else {
    D_phi = 0;
  }
  e_phi_past = e_phi; // update val to get other vals
  // I_phi = I_phi + Ts*e_phi; // integral implementation

  // PID Output
  // double phi_dot_desired = Kp_phi*e_phi + Ki_phi*I_phi + Kd_phi*D_phi;
  double phi_dot_desired = Kp_phi*e_phi + Kd_phi*D_phi;


  // Get phi_dot
  double phi_dot = r*(theta1_dot - theta2_dot)/d;
  Serial.print("\tPhi_dot: ");
  Serial.print(phi_dot);
  Serial.print("\tPhi_dot_des: ");
  Serial.print(phi_dot_desired);

  // Calculate error
  double e_phi_dot = phi_dot_desired - phi_dot;
  
  // PID for phi_dot
  if (Ts > 0) {
    D_phi_dot = (e_phi_dot-e_phi_dot_past)/Ts; // derivative implementation
  } else {
    D_phi_dot = 0;
  }
  e_phi_dot_past = e_phi_dot; // update val to get other vals
  // I_phi_dot = I_phi_dot+Ts*e_phi_dot; // integral implementation


  // Set rho_dot_desired based on task settings and turning progress
  double rho_dot_desired;
  double current_time = millis() / 1000.0;
  if (should_move && !is_moving && e_phi < 0.1) {
    is_moving = true;
    end_time = current_time + move_time;
  }
  if (is_moving && current_time < end_time) {
    rho_dot_desired = forward_distance/move_time;
  } else {
    rho_dot_desired = 0;
  }
  // Serial.print("\tCurrent time: ");
  // Serial.print(current_time);
  // Serial.print("\tEnd time: ");
  // Serial.print(end_time);


  // Get rho_dot
  double rho_dot = r*(theta1_dot + theta2_dot)/2;
  Serial.print("\tRho_dot: ");
  Serial.print(rho_dot);
  Serial.print("\tRho_dot_des: ");
  Serial.print(rho_dot_desired);

  // Calculate error
  double e_rho_dot = rho_dot_desired - rho_dot;

  // PID for rho_dot
  if (Ts > 0) {
    D_rho_dot = (e_rho_dot-e_rho_dot_past)/Ts; // derivative implementation
  } else {
    D_rho_dot = 0;
  }
  e_rho_dot_past = e_rho_dot; // update val to get other vals
  // I_rho_dot = I_rho_dot + Ts*e_rho_dot; // integral implementation

  // PID Outputs
  // double u_diff = Kp_phi_dot*e_phi_dot + Ki_phi_dot*I_phi_dot + Kd_phi_dot*D_phi_dot;
  // double u_bar = Kp_rho_dot*e_rho_dot + Ki_rho_dot*I_rho_dot + Kd_rho_dot*D_rho_dot;
  double u_diff = Kp_phi_dot*e_phi_dot + Kd_phi_dot*D_phi_dot;
  double u_bar = Kp_rho_dot*e_rho_dot + Kd_rho_dot*D_rho_dot;

  Serial.print("\tu_bar: ");
  Serial.print(u_bar);
  Serial.print("\tu_diff: ");
  Serial.print(u_diff);

  // // Saturation Checking
  // // (u_diff can't be more than twice the range)
  // if (abs(u_diff) > umax) {
  //   u_diff = sgn(u_diff)*umax;
  //   e_phi_dot = sgn(e_phi_dot)*min(umax/Kp_phi_dot, abs(e_phi_dot));
  //   // I_phi_dot = (u_diff-Kp_phi_dot*e_phi_dot-Kd_phi_dot*D_phi_dot)/Ki_phi_dot; 
  // }
  // if (abs(u_bar) > umax) {
  //   u_bar = sgn(u_bar)*umax;
  //   e_rho_dot = sgn(e_rho_dot)*min(umax/Kp_rho_dot, abs(e_rho_dot));
  //   // I_rho_dot = (u_bar-Kp_rho_dot*e_rho_dot-Kd_rho_dot*D_rho_dot)/Ki_rho_dot;
  // }

  // Update Tc and Ts
  current_time = millis() / 1000.0;
  Ts = current_time - Tc;
  Tc = current_time;

  // Update theta1_old and theta2_old
  theta1_old = theta1;
  theta2_old = theta2;

  // Convert u_bar and u_diff to motor voltages
  double uM1 = (u_bar + u_diff)/2;
  double uM2 = (u_bar - u_diff)/2;
  
  // Saturation Checking
  if (abs(uM1) > umax) {
    uM1 = sgn(uM1)*umax;
  }
  if (abs(uM2) > umax) {
    uM2 = sgn(uM2)*umax;
  }
  u_bar = uM1 + uM2;
  u_diff = uM1 - uM2;
  e_phi_dot = sgn(e_phi_dot)*min(u_diff/Kp_phi_dot, abs(e_phi_dot));
  I_phi_dot = (u_diff-Kp_phi_dot*e_phi_dot-Kd_phi_dot*D_phi_dot)/Ki_phi_dot; 
  e_rho_dot = sgn(e_rho_dot)*min(u_bar/Kp_rho_dot, abs(e_rho_dot));
  I_rho_dot = (u_bar-Kp_rho_dot*e_rho_dot-Kd_rho_dot*D_rho_dot)/Ki_rho_dot;
  

  Serial.print("\tuM1: ");
  Serial.print(uM1);
  Serial.print("\tuM2: ");
  Serial.print(uM2);

  // Convert to speeds and send to motor
  int speed1 = -uM1*400/umax;
  int speed2 = uM2*400/umax;
  md.setM1Speed(speed1);
  md.setM2Speed(speed2);
  stopIfFault();

  Serial.print("\n");
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