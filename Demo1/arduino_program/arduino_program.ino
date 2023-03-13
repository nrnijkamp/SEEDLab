#include <DualMC33926MotorShield.h>
#include <Encoder.h>
// #include <Wire.h>

// #define PERIPHERAL_ADDRESS 0x08

Encoder knobLeft(2, 6);
Encoder knobRight(3, 5);

DualMC33926MotorShield md;

const double in_per_cm = 0.3937008;
const double ft_per_cm = in_per_cm/12;

// PID variables
// double Kp_phi = 1.59690166461499;  // Hz - get from simulink models - First outerloop PD control
// double Ki_phi = 0.263957132904492; // Hz^2
// double Kd_phi = 0.402292425042345; // [unit]
// double I_phi = 0; // integral
// double D_phi = 0; // derivative
// double e_phi_past = 0; // previous value
const Kp_phi = 2; //NOTE for manual control
  
double Kp_phi_dot = 6.79421833155554; // V*s/rad - get from simulink  - Second outerloop PD control
double Ki_phi_dot = 1.11680781548276; // V/rad
double Kd_phi_dot = 0; // V*s^2/rad
double I_phi_dot = 0; // integral
double D_phi_dot = 0; // derivative
double e_phi_dot_past = 0; // previous value
  
double Kp_rho_dot = 10.9495569713465; // V*s/m - get from simulink  - Second outerloop PD control
double Ki_rho_dot = 164.243354570197; // V/m
double Kd_rho_dot = 0;                // V*s^2/m
double I_rho_dot = 0; // integral
double D_rho_dot = 0; // derivative
double e_rho_dot_past = 0; // previous value

double Ts = 0.1; // Time step
double Tc = currentTime(); // Running time

//initialize variables
const double r = 7.5 * ft_per_cm;  // radius of the wheel
const double d = 28 * ft_per_cm; // distance between the wheels
const double LEFT_MOTOR_WEIGHT = 0.85;
const double RIGHT_MOTOR_WEIGHT = 1;

// Max voltage that can be supplied
// NOTE should be set depending on charge
// const double umax = 7.8;
const double umax = 7;

// ---== TASKS ==---
const double turn_to_angle = 90.0*PI/180.0; // radians
const double forward_distance = 3; // ft
const double move_speed = 2; // ft/s
bool should_turn = true;
bool should_move = true;

bool is_moving = false;
double end_time = 0;
bool paused = false;
double pause_time = 0;

double theta1_old = 0;
double theta2_old = 0;
double u_diff_old = 0;
double u_bar_old = 0;

void setup() {
  // Start serial for output
  Serial.begin(115200);
  
  // Initialize motor
  md.init();

  // // Initialize i2c as peripheral
  // Wire.begin(PERIPHERAL_ADDRESS);

  // // Set I2C callbacks
  // Wire.onReceive(receiveData);

  Serial.println("Ready!");
}

void loop() {
  // Set phi_desired based on task settings
  double phi_desired = turn_to_angle;

  // Get motor radians
  long ticks1 = knobRight.read(); //NOTE left
  long ticks2 = knobLeft.read(); //NOTE right
  double theta1 = -((double)ticks1/3200)*2*PI;
  double theta2 = ((double)ticks2/3200)*2*PI;
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
  Serial.print("\tPhi: ");
  Serial.print(phi);
  Serial.print("\tPhi_des: ");
  Serial.print(phi_desired);

  // Calculate error
  double e_phi = phi_desired - phi;

  // If error is off enough, start turning again
  if (is_moving && !paused && e_phi > 5.0*PI/180.0) {
    paused = true;
    pause_time = currentTime();
    is_moving = false;
  }

  //NOTE Manual ph_dot_desired control further below
  // // PID for phi
  // if (Ts > 0) {
  //   D_phi = (e_phi-e_phi_past)/Ts; // derivative implementation
  // } else {
  //   D_phi = 0;
  // }
  // e_phi_past = e_phi; // update val to get other vals
  // I_phi = I_phi + Ts*e_phi; // integral implementation

  // // PID Output
  // double phi_dot_desired = Kp_phi*e_phi + Ki_phi*I_phi + Kd_phi*D_phi;


  // Get phi_dot
  double phi_dot = r*(theta1_dot - theta2_dot)/d;
  
  // Manual phi_dot_desired control
  double phi_dot_desired = phi_dot;
  if (e_phi > 0.1) {
    phi_dot_desired = e_phi;
  }
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
  //NOTE uncomment if needed
  // if (sgn(e_phi_dot_past) != sgn(e_phi_dot)) I_phi_dot = 0; // Reset integral on direction switch
  I_phi_dot = I_phi_dot + Ts*e_phi_dot; // integral implementation
  e_phi_dot_past = e_phi_dot; // update val to get other vals


  // Set rho_dot_desired based on task settings and turning progress
  double rho_dot_desired;
  double current_time = currentTime();
  if (should_move && !is_moving && e_phi < 0.1) {
    is_moving = true;
    double move_time = forward_distance/move_speed;
    if (paused) {
      // Resume moving if was paused
      paused = false;
      double paused_for = current_time - pause_time;
      end_time += paused_for;
    } else {
      end_time = current_time + move_time;
    }
  }
  if (is_moving && current_time < end_time) {
    rho_dot_desired = move_speed;
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
  if (sgn(e_rho_dot_past) != sgn(e_rho_dot)) I_rho_dot = 0; // Reset integral on direction switch
  I_rho_dot = I_rho_dot + Ts*e_rho_dot; // integral implementation
  e_rho_dot_past = e_rho_dot; // update val to get other vals
  // Serial.print("\tI_rho_dot:  ");
  // Serial.print(I_rho_dot);
  // Serial.print("\te_rho_dot: ");
  // Serial.print(e_rho_dot);
  // Serial.print("\tD_rho_dot: ");
  // Serial.print(D_rho_dot);
  

  // PID Outputs
  double u_diff = Kp_phi_dot*e_phi_dot + Ki_phi_dot*I_phi_dot + Kd_phi_dot*D_phi_dot;
  double u_bar = Kp_rho_dot*e_rho_dot + Ki_rho_dot*I_rho_dot + Kd_rho_dot*D_rho_dot;
  // double u_diff = Kp_phi_dot*e_phi_dot + Kd_phi_dot*D_phi_dot;
  // double u_bar = Kp_rho_dot*e_rho_dot + Kd_rho_dot*D_rho_dot;

  // Prevent sudden switching
  //NOTE uncomment if needed
  // if (sgn(u_diff) != sgn(u_diff_old) && abs(u_diff_old) > 1) {
  //   u_bar = u_bar_old / 2;
  // }
  // u_diff_old = u_diff;
  if (sgn(u_bar) != sgn(u_bar_old) && abs(u_bar_old) > 1) {
    u_bar = u_bar_old / 2;
  }
  u_bar_old = u_bar;

  Serial.print("\tu_diff: ");
  Serial.print(u_diff);
  Serial.print("\tu_bar: ");
  Serial.print(u_bar);

  //NOTE Should saturation check actual voltages, not u_bar and u_diff
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
  current_time = currentTime();
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
  //NOTE uncomment if needed
  // u_bar = uM1 + uM2;
  // u_diff = uM1 - uM2;
  // e_phi_dot = sgn(e_phi_dot)*min(u_diff/Kp_phi_dot, abs(e_phi_dot));
  // I_phi_dot = (u_diff-Kp_phi_dot*e_phi_dot-Kd_phi_dot*D_phi_dot)/Ki_phi_dot; 
  // e_rho_dot = sgn(e_rho_dot)*min(u_bar/Kp_rho_dot, abs(e_rho_dot));
  // I_rho_dot = (u_bar-Kp_rho_dot*e_rho_dot-Kd_rho_dot*D_rho_dot)/Ki_rho_dot;
  

  Serial.print("\tuM1: ");
  Serial.print(uM1);
  Serial.print("\tuM2: ");
  Serial.print(uM2);

  // Convert to speeds and send to motor
  int speed1 = uM1*400/umax;
  int speed2 = -uM2*400/umax;
  if (is_moving) {
    speed1 *= LEFT_MOTOR_WEIGHT;
    speed2 *= RIGHT_MOTOR_WEIGHT;
  }
  md.setM1Speed(speed1); // left
  md.setM2Speed(speed2); // right
  stopIfFault();

  Serial.print("\n");
}

double currentTime() {
  return millis()/1000.0;
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