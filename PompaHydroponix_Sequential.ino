/*
NOTES: beware of intterupt -->  se can make error for all the actuator to work
*/


//PIN FOR FLOWMETER
#define flowmeter1  D21;
#define flowmeter2  D19;
#define flowmeter3  D18; 

//PIN FOR MOTOR DRIVER

//pwm
#define pwm1   D34; //ena1
#define pwm2   D26; //ena2
#define pwm3   D12; //ena3 

//in
/*
onHigh: should on all the time when we want to use pump
lowAlways: should off all the time when we want to use pump
this is because pump is just one way, different with motor 
*/
#define onHigh1  D35; //in 1
#define lowAlways1  D32; //in2
#define onHigh2 D33; //in3
#define lowAlways2 D25 ; //in4
#define onHigh3 D27; //in5
#define lowAlways3 D14; //in6

//VARIABLE

//flowmeter
double volume1;
double volume2;
double volume3; 

volatile double pulse1;
volatile double pulse2;
volatile double pulse3; 

double converter = 2.663; //isi value 

//setpoint 
double setpoint1 ; 
double setpoint2;
double setpoint3; 

//for pid 
long prevT = 0;
float eprev = 0;
float eintegral = 0;

//sequential system, butuh uji coba lagi
bool oneDone = false;
bool twoDone = false;
bool threeDone= false;




void setup() {
  // Serial 
  Serial.begin(9600); 

  // Motor driver
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(onHigh1, OUTPUT);
  pinMode(lowAlways1, OUTPUT);
  pinMode(onHigh2, OUTPUT);
  pinMode(lowAlways2, OUTPUT);
  pinMode(onHigh3, OUTPUT);
  pinMode(lowAlways3, OUTPUT);

  //Flowmeter
  pinMode(flowmeter1, INPUT);
  pinMode(flowmeter2, INPUT);
  pinMode(flowmeter3, INPUT);

  //rawan error tapi gatau jalan atau ngga, kalo ngga jalan pulseCounternya buat 3
  attachInterrupt(digitalPinToInterrupt(flowmeter1), pulseCounter1, RISING);
  attachInterrupt(digitalPinToInterrupt(flowmeter2), pulseCounter2, RISING);
  attachInterrupt(digitalPinToInterrupt(flowmeter3), pulseCounter3, RISING);
}


void loop() {
  
//first pump
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

//second pumps
  while(!twoDone){
     // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    volume2 = converter*pulse2;
    pos = volume2;
  }
  
  // error
  int e = setpoint2 - pos;

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
  setMotor(dir,pwr,pwm2,onHigh2,lowAlways2);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();

    //IF DONE
    if(volume2 > setpoint2){
      twoDone=true;
    }
  }

//third pump
  while(!threeDone){
  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    volume3 = converter*pulse3;
    pos = volume3;
  }
  
  // error
  int e = setpoint3 - pos;

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

if(volume3 > setpoint3){
      threeDone=true;
    }
  }


}

void setMotor(int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
}
//
//counting pulse from flowmeter
void pulseCounter1(){
  pulse1++;
}

void pulseCounter2(){
  pulse2++;
}

void pulseCounter3(){
  pulse3++;
}

