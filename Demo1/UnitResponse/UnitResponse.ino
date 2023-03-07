// SEED Lab Mini Project Group 3
// Sam Leonard, Dawson J. Gullickson, Julia Kaase

#include <DualMC33926MotorShield.h>
#include <Encoder.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knobLeft(2, 6);
Encoder knobRight(3, 8);
//   avoid using pins with LEDs attached

DualMC33926MotorShield md;

double umax = 7.8; // max voltage that can be supplied
void setup() {
  // Start serial for output
  Serial.begin(31250);
  
  // Initialize motor
  md.init();

  Serial.println("Ready!");
}

void loop() {
  // Set voltage
  double u1 = 0.5;
  double u2 = 0.5;
  
  // Convert voltage to speed
  int speed1 = -(int)(u1*400.0)/umax;
  int speed2 = (int)(u2*400.0)/umax;
  // Set speed
  md.setM1Speed(speed1);
  md.setM2Speed(speed2);
  stopIfFault();

  // Get motor radians
  //NOTE maybe switched
  long left_ticks = knobLeft.read();
  long right_ticks = knobRight.read();
  double a1 = ((double)left_ticks/3200)*2*PI;
  double a2 = ((double)right_ticks/3200)*2*PI;

  // Output data
  double currentTime = millis() / 1000.0;
  Serial.print(currentTime);
  Serial.print("\t");
  Serial.print(a1);
  Serial.print("\t");
  Serial.println(a2);
}

// Halts the program if the motor faults
void stopIfFault() {
  if (md.getFault()) {
    Serial.println("fault");
    while (true);
  }
}