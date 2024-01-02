/*
i think this concurrent method is just dangerous and don't know if it's work
*/


//PIN FOR FLOWMETER
const int flowmeter1 = D21;
const int flowmeter2 = D19;
const int flowmeter3 = D18; 

//PIN FOR MOTOR DRIVER

//pwm
const int pwm1 =  D34; //ena1
const int pwm2 =  D26; //ena2
const int pwm3 =  D12; //ena3 

//in
const int onHigh1 = ; //in 1
const int lowAlways1 = ; //in2
const int onHigh2 = ; //in3
const int lowAlways2 = ; //in4
const int onHigh3 = ; //in5
const int lowAlways3 = ; //in6

//PIN FOR BUZZER
const int buzzer = D15; 

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


//try robust
bool oneDone = false;
bool twoDone = false;
bool threeDone = false;


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
  volume1 = 2.663*pulse1;
  volume2 = 2.663*pulse2;
  volume3 = 2.663*pulse3;

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

  //one
    if(volume1<setpoint1){
    digitalWrite(onHigh1, HIGH);
    digitalWrite(lowAlways1, LOW);
    digitalWrite(pwm1, 125);
  }
  else{
    digitalWrite(onHigh1, LOW);
    digitalWrite(lowAlways1, LOW);
    digitalWrite(pwm1, 0);
    oneDone = true; 
  }

//two
  if((volume2<setpoint2) && oneDone){
    digitalWrite(onHigh2, HIGH);
    digitalWrite(lowAlways2, LOW);
    digitalWrite(pwm2, 125);
  }
  else{
    digitalWrite(onHigh2, LOW);
    digitalWrite(lowAlways2, LOW);
    digitalWrite(pwm2, 0);
    twoDone = true;

  }

//three
   if(volume3<setpoint3 && oneDone && twoDone){
    digitalWrite(onHigh3, HIGH);
    digitalWrite(lowAlways3, LOW);
    digitalWrite(pwm3, 125);
  }
  else{
    digitalWrite(onHigh3, LOW);
    digitalWrite(lowAlways3, LOW);
    digitalWrite(pwm3, 0);




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
