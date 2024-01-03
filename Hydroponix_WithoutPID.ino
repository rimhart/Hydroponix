/*
i think this concurrent method is just dangerous and don't know if it's work
*/


//PIN FOR FLOWMETER
#define flowmeter1 = D21;
#define flowmeter2 = D19;
#define flowmeter3 = D18; 

//PIN FOR MOTOR DRIVER

//pwm
#define pwm1 =  D34; //ena1
#define pwm2 =  D26; //ena2
#define pwm3 =  D12; //ena3 

//in
#define onHigh1 = D35 ; //in 1
#define lowAlways1 = D32; //in2
#define onHigh2 = D33 ; //in3
#define lowAlways2 =  D25; //in4
#define onHigh3 = D27; //in5
#define lowAlways3 = D14; //in6

//PIN FOR BUZZER
#define buzzer = D15; 

//VARIABLE

//flowmeter
double volume1;
double volume2;
double volume3; 

double pulse1;
double pulse2;
double pulse3; 

double setpoint1;
double setpoint2;
double setpoint3; 


//try sequential 
bool oneDone = false;
bool twoDone = false;
bool threeDone = false;

//to start the system 
bool start = false;

//convert from pulse to volume
double converter =  2.663; 


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

  //buzzer 
  pinMode(buzzer,OUTPUT);

  //rawan error tapi gatau jalan atau ngga, kalo ngga jalan pulseCounternya buat 3
  attachInterrupt(digitalPinToInterrupt(flowmeter1), pulseCounter1, RISING);
  attachInterrupt(digitalPinToInterrupt(flowmeter2), pulseCounter2, RISING);
  attachInterrupt(digitalPinToInterrupt(flowmeter3), pulseCounter3, RISING);
}


void loop() {
  //for counting the volume
  volume1 = converter*pulse1;
  volume2 = converter*pulse2;
  volume3 = converter*pulse3;

  //if the volume same as target, stop the water. 

  /*
  //this system delay is worst, because the proccess is sequential. should search for more robust without PID
  if(volume1<setpoint1){
    digitalWrite(onHigh1, HIGH);
    digitalWrite(lowAlways1, LOW);
    digitalWrite(pwm1, 125);
  }
  else{
    digitalWrite(onHigh1, LOW);
    digitalWrite(lowAlways1, LOW);
    digitalWrite(pwm1, 0);
  }

  if(volume2<setpoint2){
    digitalWrite(onHigh2, HIGH);
    digitalWrite(lowAlways2, LOW);
    digitalWrite(pwm2, 125);
  }
  else{
    digitalWrite(onHigh2, LOW);
    digitalWrite(lowAlways2, LOW);
    digitalWrite(pwm2, 0);
  }

  if(volume3<setpoint3){
    digitalWrite(onHigh3, HIGH);
    digitalWrite(lowAlways3, LOW);
    digitalWrite(pwm3, 125);
  }
  else{
    digitalWrite(onHigh3, LOW);
    digitalWrite(lowAlways3, LOW);
    digitalWrite(pwm3, 0);
  } */


  //MAYBE MORE ROBUST METHOD SO FOCUS ON ONE PUMP AT A TIME

  //start
if(start){
  //one
    if(volume1<setpoint1){
    digitalWrite(onHigh1, HIGH);
    digitalWrite(lowAlways1, LOW);
    analogWrite(pwm1, 125);
  }
  else{
    digitalWrite(onHigh1, LOW);
    digitalWrite(lowAlways1, LOW);
    analogWrite(pwm1, 0);
    oneDone = true; 
  }}
else{
    oneDone = false;
    twoDone = false;
    threeDone = false;
}

//two
  if((volume2<setpoint2) && oneDone){
    digitalWrite(onHigh2, HIGH);
    digitalWrite(lowAlways2, LOW);
    analogWrite(pwm2, 125);
  }
  else if(volume2>=setpoint2){
    twoDone = true;
    digitalWrite(onHigh2, LOW);
    digitalWrite(lowAlways2, LOW);
    analogWrite(pwm2, 0);
  }
  else{
    digitalWrite(onHigh2, LOW);
    digitalWrite(lowAlways2, LOW);
    analogWrite(pwm2, 0);
  }

//three
   if(volume3<setpoint3 && oneDone && twoDone){
    digitalWrite(onHigh3, HIGH);
    digitalWrite(lowAlways3, LOW);
    analogWrite(pwm3, 125);
  }
  else if(volume3>=setpoint3){
    threeDone = true;
    digitalWrite(onHigh2, LOW);
    digitalWrite(lowAlways2, LOW);
    analogWrite(pwm2, 0);
  }
  else{
    digitalWrite(onHigh3, LOW);
    digitalWrite(lowAlways3, LOW);
    analogWrite(pwm3, 0);
  }



  //make system if the pulse is zero but there is setpoint it would 
  if(pulse1==0 && setpoint1 != 0){
    digitalWrite(buzzer, HIGH);
  }
  if(pulse2==0 && setpoint2 != 0){
    digitalWrite(buzzer, HIGH);
  }
  if(pulse3==0 && setpoint3 != 0){
    digitalWrite(buzzer, HIGH);
  }
  
  //if all the system is done, stop the work + refresh the variable
  if(oneDone && twoDone && threeDone ){
    oneDone = false;
    twoDone = false;
    threeDone = false;
    start = false;
  }
}

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
