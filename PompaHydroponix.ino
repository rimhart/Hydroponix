/*
NOTES: beware of intterupt -->  se can make error for all the actuator to work
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

//VARIABLE

//flowmeter
double volume1;
double volume2;
double volume3; 

double pulse1;
double pulse2;
double pulse3; 

//for pid 
double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd;


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
  //for counting the volume


  //if the volume same as target, stop the water

  // we will use 3 error to do, i think we should do it sekuensial first if there is no problem we can do simultaneously
  
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


//for pid using error from flowmeter
double pid(double error){
  double proportional = error; //proportional is just for error
  integral += error * dt; //integral is summing the error
  double derivative = (error - previous) / dt; //know that the error will be reduce overtime, but if the rate is big, the previous is have big error, so it would have BIG difference negative. but if the rate is low, the previous and error at the time will have low negative difference 
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative); //pid
  return output;
}
}