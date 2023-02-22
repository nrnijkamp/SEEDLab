//PI Motor Implementation - Mini Project 4.7
//Sam Leonard
// 2/15/23
//PID Control Gains 
double Kp = 15.3786175942488;
double Ke = .5;
double Ki = 2.37803426483209;
double Kd = 0;

//Given from document
double I = 0; //integral
double D = 0; //deriv
double e_past = 0; //prev val
double Ts = 0;
double Tc = millis() / 1000; //[Do they want me to reference a built in clock, or am I overthinking?]

double r = 0; //radians
double umax = 7.8; //max voltage that can be supplied
void setup() {}

void loop() {
  //Given pseudocode translated to code

  //[read r - user input] radians [Speak to nick to learn how the system gets input]

  //[read y - where the motor is at] radians, use encoder to find position?

  //calc error
  double e = r-y; //find where it needs to move from where it is

  if(Ts>0)
  {
    D = (e-e_past)/Ts; //derivative
    e_past = e; //update val to get other vals
  }
  else 
  {
    D = 0;
  }

  I = I+Ts*e; //integral

  //Calc controller output -- output voltage uses PID
  double u = Kp*e+Ki*I+Kd*D;
  //deals with actuator saturation i.e. if trying to write a voltage too high for board to supply
  if(abs(u)>umax) 
  {
    u = sgn(u)*umax;
    e = sgn(e)*min(umax/Kp, abs(e));
    I = (u-Kp*e-Kd*D)/Ki; 
  }
    //[write u to ouput]

    //part 3 implementation
    double currentTime = millis() / 1000;
    Ts = currentTime - Tc;
    Tc = currentTime;
  Serial.print("Time: ");
  Serial.print(currentTime);
  Serial.print("R: ");
  Serial.println(r);
  }
