//PI Motor Implementation - Mini Project 4.7
//Sam Leonard
// 2/15/23
//PID Control Gains 
int Kp=;
int Ke=;
int Ki=;

//Given from document
int I=0; //integral
int D=0; //deriv
int e_past=0; //prev val
int Ts=0;
int Tc=currentTime; //[Do they want me to reference a built in clock, or am I overthinking?]

int r=0; //radians
int umax= 10; //max voltage that can be supplied
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

  //Given pseudocode translated to code

  //[read r - user input] radians [Speak to nick to learn how the system gets input]

  //[read y - where the motor is at] radians, use encoder to find position?

  //calc error
  int e = r-y; //find where it needs to move from where it is

  if(Ts>0)
  {
    D=(e-e_past)/Ts; //derivative
    e_past=e; //update val to get other vals
  }
  else 
  {
    D=0;
  }

  I=I+Ts*e; //integral

  //Calc controller output -- output voltage uses PID
 int u=Kp*e+Ki*I+Kd*D;
  //deals with actuator saturation i.e. if trying to write a voltage too high for board to supply
  if(abs(u)>umax) 
  {
    u=sgn(u)*umax;
    e=sgn(e)*min(umax/Kp, abs(e));
    I=(u-Kp*e-Kd*D)/Ki; 
  }
    //[write u to ouput]
    
    Ts=currentTime-Tc;
    Tc=currentTime;

  }
