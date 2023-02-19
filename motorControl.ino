#define ENCA 2 // Encoder A PIN
#define ENCB 3 // Encoder B PIN

#define PWM 6 // Motor PWM PIN
#define IN1 7 // Motor IN1 PIN
#define IN2 8 // Motor IN2 PIN

// CONST


// VAR

volatile int pos = 0;

long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  // put your setup code here, to run once:

  Serial .begin(115200);
  
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB),readEncoderB, CHANGE);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT); 
  
}

void loop() {
  // put your main code here, to run repeatedly:

  int target = 0;

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position
  int posi = pos;

  // Not sure about this part (only if corrupted data is read for posi)
  // int posi = pos;
  // noInterrupts(); // disable interrupts temporarily while reading
  // posi = pos;
  // interrupts(); // turn interrupts back on


  // PID COMPUTE
  // error
  int e = posi - target;
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

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);

}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoderA() {
  bool a = digitalRead(ENCA);
  bool b = digitalRead(ENCB);
  if(a != b){
    pos++;
  }
  else{
    pos--;
    }
}

void readEncoderB() {
  bool a = digitalRead(ENCA);
  bool b = digitalRead(ENCB);
  if(a = b){
    pos++;
  }
  else{
    pos--;
    }
}
