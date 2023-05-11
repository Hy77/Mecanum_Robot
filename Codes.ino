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
//Anti_rollover_RHS_sg90
#define sg_R 9
Servo sgServo_R;

int direct1 = 999;
int direct2 = 999;
int direct3 = 999;
int direct4 = 999;
const int roboSpeed = 78;
const int CLOCKWISE = 500; // Servo rorate CW
const int STOP = 1500; // Servo stop
const int C_CLOCKWISE = 2000; // Servo rorate CCW
unsigned long currentTime = 0; // cur time
unsigned long previousTime = 0; // pre time
bool anti_roll_extend_flag = false;
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
  //MG_servoA.write(0);
  // servo B & reset
  MG_servoB.attach(servoB);
  //MG_servoB.write(0);
  // servo C & reset
  MG_servoC.attach(servoC);
  //MG_servoC.write(0);
  sgServo_R.attach(sg_R);
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
    for (int i = 0; i < 2; i++) {

      antiRollover(1300); // antiRollOver protection retract/extend

      //move_forward(600);
      //move_backward(300);
      //move_right(600);
      //rotate_cw(905);//180deg
      /*move_forward(1200);
      stop(direct1, direct2, direct3, direct4);
      fakeDelay(500);
      
      move_left(700);
      stop(direct1, direct2, direct3, direct4);
      fakeDelay(500);
      
      rotate_cw(1570);
      stop(direct1, direct2, direct3, direct4);
      fakeDelay(500);
      
      move_left(800);*/

      if (i == 1) {
        run_flag = false;
        //stop(direct1, direct2, direct3, direct4);
      }
    }
  }

}

void fakeDelay(int delayTime) {
  currentTime = millis(); // get cur time

  // 如果距离上一次旋转已经超过指定的延迟时间，就返回
  if (currentTime - previousTime >= delayTime) {
    previousTime = currentTime; // update pre time
    return;
  }
  // wait if...
  while (currentTime - previousTime < delayTime) {
    currentTime = millis(); // update cur
  }
  previousTime = currentTime; // update pre
}

void armExtend() {
  MG_servoA.write(0);
}

void antiRollover(int rotateTime) {
  if (anti_roll_extend_flag == false) {
    sgServo_R.writeMicroseconds(C_CLOCKWISE);
    fakeDelay(rotateTime);
    sgServo_R.writeMicroseconds(STOP);
    anti_roll_extend_flag = true;
  }
  else {
    sgServo_R.writeMicroseconds(CLOCKWISE);
    fakeDelay(rotateTime);
    sgServo_R.writeMicroseconds(STOP);
  }
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

      break;
  }
}

void move_forward(int duration) {
  direct1 = 1; direct2 = 1; direct3 = 1; direct4 = 1;
  move(1, roboSpeed, direct1); // A forward
  move(2, roboSpeed, direct2); // B forward
  move(3, roboSpeed, direct3); // C forward
  move(4, roboSpeed, direct4); // D forward
  fakeDelay(duration);
}

void move_backward(int duration) {
  direct1 = 0; direct2 = 0; direct3 = 0; direct4 = 0;
  move(1, roboSpeed, direct1); // A backward
  move(2, roboSpeed, direct2); // B backward
  move(3, roboSpeed, direct3); // C backward
  move(4, roboSpeed, direct4); // D backward
  fakeDelay(duration);
}

void move_right(int duration) {
  direct1 = 1; direct2 = 0; direct3 = 1; direct4 = 0;
  move(1, roboSpeed, direct1); // A backward
  move(2, roboSpeed, direct2); // B forward
  move(3, roboSpeed, direct3); // C backward
  move(4, roboSpeed, direct4); // D forward
  fakeDelay(duration);
}

void move_left(int duration) {
  direct1 = 0; direct2 = 1; direct3 = 0; direct4 = 1;
  move(1, roboSpeed, direct1); // A forward
  move(2, roboSpeed, direct2); // B backward
  move(3, roboSpeed, direct3); // C forward
  move(4, roboSpeed, direct4); // D backward
  fakeDelay(duration);
}

void rotate_cw(int duration) {
  direct1 = 0; direct2 = 0; direct3 = 1; direct4 = 1;
  move(1, roboSpeed, direct1); // A forward
  move(2, roboSpeed, direct2); // B forward
  move(3, roboSpeed, direct3); // C backward
  move(4, roboSpeed, direct4); // D backward
  fakeDelay(duration);
}

void stop(int direct1, int direct2, int direct3, int direct4) {
  //enable standby
  /*for (int i = 22; i < 30; i++) {
    digitalWrite(i, LOW);
    }*/
  if (direct1 == 1) direct1 = 0;
  else direct1 = 1;
  if (direct2 == 1) direct2 = 0;
  else direct2 = 1;
  if (direct3 == 1) direct3 = 0;
  else direct3 = 1;
  if (direct4 == 1) direct4 = 0;
  else direct4 = 1;
  move(1, 0, direct1); // A moves opps
  move(2, 0, direct2); // B moves opps
  move(3, 0, direct3); // C moves opps
  move(4, 0, direct4); // D moves opps
}
