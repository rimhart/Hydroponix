
/*
NOTES: beware of intterupt -->  se can make error for all the actuator to work
*/

#include <util/atomic.h>

//PIN FOR FLOWMETER
#define flowmeter1  D21


//PIN FOR MOTOR DRIVER

//pwm
#define pwm1   D34 //ena1


//in
/*
onHigh: should on all the time when we want to use pump
lowAlways: should off all the time when we want to use pump
this is because pump is just one way, different with motor 
*/
#define onHigh1  D35 //in 1
#define lowAlways1  D32 //in2


//VARIABLE

//flowmeter
double volume1;


volatile double pulse1;


double converter = 2.663; //isi value 

//setpoint 
double setpoint1 =1000; 


//for pid 
long prevT = 0;
float eprev = 0;
float eintegral = 0;

//sequential system, butuh uji coba lagi
bool oneDone = false;


void setup() {
  // put your setup code here, to run once:
  // Serial 
  Serial.begin(9600); 

  // Motor driver
  pinMode(pwm1, OUTPUT);
  pinMode(onHigh1, OUTPUT);
  pinMode(lowAlways1, OUTPUT);

  //Flowmeter
  pinMode(flowmeter1, INPUT);

  //rawan error tapi gatau jalan atau ngga, kalo ngga jalan pulseCounternya buat 3
  attachInterrupt(digitalPinToInterrupt(flowmeter1), pulseCounter1, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:

while(!oneDone){
     // set target position --> using setpoint1
  

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    volume1 = converter*pulse1;
    pos = volume1;
  }
  
  // error
  int e = setpoint1 - pos;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }


  // signal the motor
  setMotor(dir,pwr,pwm1,onHigh1,lowAlways1);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();

    //IF DONE
    if(volume1 > setpoint1){
      oneDone=true;
    }
  }
}

void setMotor(int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
}

void pulseCounter1(){
  pulse1++;
}
