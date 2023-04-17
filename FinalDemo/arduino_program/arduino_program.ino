// SEED Lab, Team 3, Demo 2

#include <DualMC33926MotorShield.h>
#include <Encoder.h>
#include <Wire.h>

#define PERIPHERAL_ADDRESS 0x08

Encoder knobLeft(2, 6);
Encoder knobRight(3, 5);

DualMC33926MotorShield md;

const double in_per_cm = 0.3937008;
const double ft_per_cm = in_per_cm/12;

double currentTime() {
  return millis()/1000.0;
}

// PID variables
double Kp_phi = 10.0051896084746; // Hz
  
double Kp_phi_dot = 10.22597888102; // V*s/rad
double Kd_phi_dot = 0; // V*s^2/rad
double D_phi_dot = 0; // derivative
double D_phi_dot_old1 = 0; // previous derivative
double D_phi_dot_old2 = 0; // previous derivative
double e_phi_dot_past = 0; // previous value
  
double Kp_rho_dot = 7.14216735267938; // V*s/m
double Ki_rho_dot = 119.884076500748; // V/m
double I_rho_dot = 0; // integral
// double e_rho_dot_past = 0; // previous value

double Ts = 0.1; // Time step
double Tc = currentTime(); // Running time

//initialize variables
const double r = 3.0 / 12.0;  // radius of the wheel
const double d = 28 * ft_per_cm; // distance between the wheels
const double LEFT_MOTOR_WEIGHT = 1;
const double RIGHT_MOTOR_WEIGHT = 1;

// Max voltage that can be supplied
// NOTE should be set depending on charge
// const double umax = 7.8;
const double umax = 6;

// ---== TASKS ==---
bool should_turn = true;
bool should_move = true;
double turn_by_angle = 0.0*PI/180.0; // radians
double forward_distance = 0; // ft
const double move_speed = 1; // ft/s
const double search_speed = 50.0*PI/180.0; // radians/s
const double max_turn_speed = 90.0*PI/180.0; // radians/s

bool is_searching = false;
bool is_moving = false;
bool at_destination = false;
double distance_traveled = 0;
double turn_to_angle = 0;

double theta1_old = 0;
double theta2_old = 0;
double u_diff_old = 0;
double u_bar_old = 0;

void setup() {
  // Start serial for output
  Serial.begin(115200);
  
  // Initialize motor
  md.init();

  // Initialize i2c as peripheral
  Wire.begin(PERIPHERAL_ADDRESS);

  // Set I2C callbacks
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  Serial.println("Ready!");
}

void loop() {
  // Update target angle
  turn_to_angle += turn_by_angle;
  turn_by_angle = 0;

  // Set phi_desired based on task settings
  double phi_desired;
  if (should_turn) {
    phi_desired = turn_to_angle;
  } else {
    phi_desired = 0;
  }

  // Get motor radians
  long ticks1 = knobRight.read(); //NOTE left
  long ticks2 = knobLeft.read(); //NOTE right
  double theta1 = -((double)ticks1/3200)*2*PI;
  double theta2 = ((double)ticks2/3200)*2*PI;
  // Serial.print("\tTheta1: ");
  // Serial.print(theta1);
  // Serial.print("\tTheta2: ");
  // Serial.print(theta2);
  
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

  // PID Output
  double phi_dot_desired;
  if (!is_searching) phi_dot_desired = Kp_phi*e_phi; // P
  else {
    phi_dot_desired = search_speed;
    // Update the angle we're at.
    turn_to_angle = phi;
  }
  // Bound phi_dot_desired
  if (abs(phi_dot_desired) > max_turn_speed)
    phi_dot_desired = sgn(phi_dot_desired)*max_turn_speed;


  // Get phi_dot
  double phi_dot = r*(theta1_dot - theta2_dot)/d;
  Serial.print("\tPhi_dot: ");
  Serial.print(phi_dot);
  Serial.print("\tPhi_dot_des: ");
  Serial.print(phi_dot_desired);

  // Calculate error
  double e_phi_dot = phi_dot_desired - phi_dot;
  // Prevent jittering
  if (abs(phi_dot) > abs(phi_dot_desired) && abs(phi_dot_desired) < 1.5*abs(phi_dot))
    e_phi_dot = 0;
  
  // PID for phi_dot
  D_phi_dot_old2 = D_phi_dot_old1;
  D_phi_dot_old1 = D_phi_dot;
  D_phi_dot = (e_phi_dot-e_phi_dot_past)/Ts; // derivative implementation
  double D_phi_dot_lowpass = (D_phi_dot + D_phi_dot_old1 + D_phi_dot_old2)/3;
  e_phi_dot_past = e_phi_dot; // update val to get other vals


  // Set rho_dot_desired based on task settings and turning progress
  double rho_dot_desired;
  double current_time = currentTime();
  if (should_move && !is_moving && abs(e_phi) < 0.02 && abs(e_phi_dot) < 0.02) {
    is_moving = true;
  }
  if (is_moving && !is_searching && distance_traveled < forward_distance) {
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
  distance_traveled += rho_dot*Ts;
  // Serial.print("\tRho_dot: ");
  // Serial.print(rho_dot);
  // Serial.print("\tRho_dot_des: ");
  // Serial.print(rho_dot_desired);
  // Serial.print("\tDistance: ");
  // Serial.print(distance_traveled);

  // Calculate error
  double e_rho_dot = rho_dot_desired - rho_dot;

  // PID for rho_dot
  // if (sgn(e_rho_dot_past) != sgn(e_rho_dot)) I_rho_dot = 0; // Reset integral on direction switch
  I_rho_dot += Ts*e_rho_dot; // integral implementation
  // e_rho_dot_past = e_rho_dot; // update val to get other vals
  // Serial.print("\tI_rho_dot:  ");
  // Serial.print(I_rho_dot);
  // Serial.print("\te_rho_dot: ");
  // Serial.print(e_rho_dot);
  

  // PID Outputs
  // double u_diff = Kp_phi_dot*e_phi_dot + Kd_phi_dot*D_phi_dot; // PD
  double u_diff = Kp_phi_dot*e_phi_dot + Kd_phi_dot*D_phi_dot_lowpass; // PD (filtered)

  double u_bar;
  if (is_moving) {
    u_bar = Kp_rho_dot*e_rho_dot + Ki_rho_dot*I_rho_dot; // PI
  } else u_bar = 0;

  Serial.print("\tu_diff: ");
  Serial.print(u_diff);
  Serial.print("\tu_bar: ");
  Serial.print(u_bar);

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
  // e_rho_dot = sgn(e_rho_dot)*min(u_bar/Kp_rho_dot, abs(e_rho_dot));
  // I_rho_dot = (u_bar-Kp_rho_dot*e_rho_dot)/Ki_rho_dot; // PI
  

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

  // Inform pi when we've finished moving
  at_destination = is_moving && !is_searching && distance_traveled >= forward_distance;
}

// Get data from raspberry pi
const double MAX_DIST = 5;
const double ANGLE_MIN = -PI/3;
const double ANGLE_MAX = PI/3;
const double ANGLE_RANGE = ANGLE_MAX - ANGLE_MIN;
void receiveData(int _byte_count) {
  byte command = Wire.read();
  Serial.print(command);

  if (command == 0) {
    // Search for marker
    is_searching = true;
  } else if (command == 1) {
    // Update angle and distance
    byte angle_byte = Wire.read();
    byte distance_byte = Wire.read();
    Serial.print("\t");
    Serial.print(angle_byte);
    Serial.print("\t");
    Serial.print(distance_byte);
    Serial.print("\t");
    
    double angle = -(angle_byte*ANGLE_RANGE/255 + ANGLE_MIN);
    double distance = distance_byte*MAX_DIST/255; 
    Serial.print(angle);
    Serial.print("\t");
    Serial.print(distance);

    // The first angle may be sent late, so decrease it by the search speed times the delay
    if (is_searching) turn_by_angle += angle - search_speed * 0.2;
    else turn_by_angle += angle;
    distance_traveled = 0;
    forward_distance = distance - 0.5; // Fudged
    is_searching = false;
    is_moving = false;
    at_destination = false;
  } else {
    Wire.write(at_destination);
    Serial.print("Sent ");
    Serial.print(at_destination);
    Serial.print("\n");
  }
  Serial.print("\n");
}

// Callback for sending data
void sendData() {
}

void stopIfFault() {
  if (md.getFault()) {
    Serial.println("fault");
    while (true);
  }
}

int sgn(double v) {
  if (v >= 0) return 1;
  else return -1;
}