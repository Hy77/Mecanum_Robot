#include <Servo.h>

//Motor A
#define PWMA 2  //Speed control
#define AIN1 22 //Direction
#define AIN2 23 //Direction
//Motor B
#define PWMB 3 //Speed control
#define BIN1 24 //Direction
#define BIN2 25 //Direction
//Motor C
#define PWMC 4 //Speed control
#define CIN1 27 //Direction
#define CIN2 26 //Direction
//Motor D
#define PWMD 5 //Speed control
#define DIN1 29 //Direction
#define DIN2 28 //Direction
//Servo A
#define servoA 6
Servo MG_servoA;
//Servo B
#define servoB 7
Servo MG_servoB;
//Servo C
#define servoC 8
Servo MG_servoC;

const int CLOCKWISE = 500; // Servo rorate CW
const int STOP = 1500; // Servo stop
const int C_CLOCKWISE = 2000; // Servo rorate CCW

bool run_flag = true;

void setup() {
  // motor A
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  // motor B
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  // motor C
  pinMode(PWMC, OUTPUT);
  pinMode(CIN1, OUTPUT);
  pinMode(CIN2, OUTPUT);
  // motor D
  pinMode(PWMD, OUTPUT);
  pinMode(DIN1, OUTPUT);
  pinMode(DIN2, OUTPUT);
  // servo A & reset
  MG_servoA.attach(servoA);
  MG_servoA.write(0);
  // servo B & reset
  MG_servoB.attach(servoB);
  MG_servoB.write(0);
  // servo C & reset
  MG_servoC.attach(servoC);
  MG_servoC.write(0);
  Serial.begin(9600);
}

void loop() {
  /*if (run_flag == true) {
    for (int i = 0; i < 1; i++) {
      move_right(500);
      if (i == 0) run_flag = false;
    }
    }
    stop();*/
  if (run_flag == true) {
    for (int i = 0; i < 1; i++) {
      
      // Rotate cw
      MG_servoA.writeMicroseconds(CLOCKWISE);
      
      // Wait enough time for the servo to rotate at the desired Angle
      delay(800);

      // stop it
      MG_servoA.writeMicroseconds(STOP);
      
      if (i == 0) run_flag = false;
    }
  }

}

void armExtend() {
  MG_servoA.write(0);
}

void move(int motor, int speed, int direction) {
  //Move specific motor at speed and direction
  //motor: 1 for A; 2 for B; 3 for C; 4 for D;
  //speed: 0 is off, and 255 is full speed
  //direction: 0 clockwise/forward, 1 counter-clockwise/backward

  // default forward
  bool inPin1 = LOW;
  bool inPin2 = HIGH;
  // if backward needed
  if (direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  // check which motor should be run
  switch (motor) {
    case 1:
      digitalWrite(AIN1, inPin1);
      digitalWrite(AIN2, inPin2);
      analogWrite(PWMA, speed);
      break;
    case 2:
      digitalWrite(BIN1, inPin1);
      digitalWrite(BIN2, inPin2);
      analogWrite(PWMB, speed);
      break;
    case 3:
      digitalWrite(CIN1, inPin1);
      digitalWrite(CIN2, inPin2);
      analogWrite(PWMC, speed);
      break;
    case 4:
      digitalWrite(DIN1, inPin1);
      digitalWrite(DIN2, inPin2);
      analogWrite(PWMD, speed);
      break;
    default:
      stop();
      break;
  }
}

void move_forward(int duration) {
  move(1, 128, 0); // A backward
  move(2, 128, 0); // B forward
  move(3, 128, 0); // C backward
  move(4, 128, 0); // D forward
  delay(duration);
}

void move_left(int duration) {
  move(1, 128, 1); // A backward
  move(2, 128, 0); // B forward
  move(3, 128, 1); // C backward
  move(4, 128, 0); // D forward
  delay(duration);
}

void move_right(int duration) {
  move(1, 128, 0); // A forward
  move(2, 128, 1); // B backward
  move(3, 128, 0); // C forward
  move(4, 128, 1); // D backward
  delay(duration);
}

void rotate_cw(int duration) {
  move(1, 128, 0); // A forward
  move(2, 128, 0); // B forward
  move(3, 128, 1); // C backward
  move(4, 128, 1); // D backward
  delay(duration);
}

void stop() {
  //enable standby
  //  digitalWrite(AIN1, LOW);
  //  digitalWrite(AIN2, LOW);
  //  digitalWrite(BIN1, LOW);
  //  digitalWrite(BIN2, LOW);
  //  digitalWrite(CIN1, LOW);
  //  digitalWrite(CIN2, LOW);
  //  digitalWrite(DIN1, LOW);
  //  digitalWrite(DIN2, LOW);
  for (int i = 22; i < 30; i++) {
    digitalWrite(i, LOW);
  }
}
