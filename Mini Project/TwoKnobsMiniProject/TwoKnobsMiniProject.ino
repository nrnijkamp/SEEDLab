/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knobLeft(2, 6);
Encoder knobRight(3, 8);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("TwoKnobs Encoder Test:");
}

float countLeft, countRight = 0;
long positionLeft  = 0;
long positionRight = 0;
long phiRightOld = 0;
long phiLeftOld = 0;
long phiLeftNew, phiRightNew;

void loop() {
  float newLeft, newRight;
  float angLeft, angRight;
  newLeft = knobLeft.read();
  newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    if (newLeft > positionLeft) {
      countLeft = countLeft + 1;
    } else if(newLeft < positionLeft) {
      countLeft = countLeft - 1;
    } else if (newRight > positionRight) {
      countRight = countRight + 1;
    } else if (newRight < positionRight) {
      countRight = countRight - 1;
    }
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    phiLeftNew = phiLeftOld + ((newLeft - positionLeft)/0.1);
    phiRightNew = phiRightOld + ((newRight - positionRight)/0.1);
    angLeft = newLeft/3200;
    angRight = newRight/3200;
    //angRight = countRight*((2*3.14)/3200)*20;
    Serial.print("Radians Left = ");
    Serial.print(angLeft);
    Serial.print(", Radians Right = ");
    Serial.print(angRight);
    positionLeft = newLeft;
    positionRight = newRight;
    phiRightOld = phiRightNew;
    phiLeftOld = phiLeftNew;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println(" Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }
}
