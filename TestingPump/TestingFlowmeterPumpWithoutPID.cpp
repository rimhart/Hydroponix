//define pin
#define flowmeter 3
#define pwm 6 
#define alwaysOn 4
#define alwaysOff 5

//variable 
volatile double volume, pulse;
const int setpoint=10000; //cek sama ngga kay pengukuran asli 1000 ml

void setup() {
  Serial.begin(9600);
  pinMode(pwm, OUTPUT);
  pinMode(alwaysOn, OUTPUT);
  pinMode(alwaysOff, OUTPUT);
  pinMode(flowmeter, INPUT);

  //interrupt
  attachInterrupt(digitalPinToInterrupt(flowmeter), pulseCounter, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  volume= 2.663 * pulse ;//2.663 is number that you get from experiment, cari dulu aja sebenernya

  Serial.print(pulse);
  Serial.print(" , ");
  Serial.print(volume);
  Serial.println("");

  if(setpoint>volume){
    digitalWrite(alwaysOn, HIGH);
    digitalWrite(alwaysOff, LOW);
    digitalWrite(pwm, 40);
  }
  else{
  digitalWrite(alwaysOn, LOW);
    digitalWrite(alwaysOff, LOW);
    digitalWrite(pwm, 0);}
}


void pulseCounter(){
  pulse++;
}
