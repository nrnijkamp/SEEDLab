//Motor Driver Mini Project - 4.3
// Sam Leonard
// 2/15/23

//Pin outs:
int PWM1=9; //PWM for 1 (speed)
int PWM2=10;
int sign1=7;//determines direction of rotation for 1
int sign2=8; 

//need new name
int output =4;
int input =12;


void setup() {
  // put your setup code here, to run once:
  pinMode(PWM1,OUTPUT); //Use analog write with PWMs
  pinMode(PWM2,OUTPUT);
  pinMode(sign1,OUTPUT);
  pinMode(sign2,OUTPUT);
  pinMode(output,OUTPUT);
  pinMode(input,INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(sign1,1); //move in 1 direction
  analogWrite(PWM1,63); //25% on

}
