#include <Servo.h>
/*
  Timer 0: PWM 4、13
  Timer 1: PWM 11、12
  Timer 2: PWM 9、10
  Timer 3: PWM 2、3、5
  Timer 4: PWM 6、7、8*/

// servoA will share its timer with motor C
// timer3 -> pin235
// timer0 -> pin4
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
#define servoA 13 // timer0
Servo MG_servoA;
//Servo B
#define servoB 12 // timer1
Servo MG_servoB;
//Servo C
#define servoC 10 // timer2
Servo MG_servoC;
//Servo D
#define servoD 8 // timer4
Servo MG_servoD;
//Anti_rollover_RHS_sg90
#define sg_R 44
Servo sgServo_R;
//Anti_rollover_LHS_sg90
#define sg_L 45
Servo sgServo_L;

int direct1 = 999;
int direct2 = 999;
int direct3 = 999;
int direct4 = 999;
const int roboSpeed = 78; // 0~255
const int CLOCKWISE = 500; // Servo rorate CW
const int STOP = 1500; // Servo stop
const int C_CLOCKWISE = 2000; // Servo rorate CCW
const int motorStep = 1;
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

  // servo A
  MG_servoA.attach(servoA);
  // servo B
  MG_servoB.attach(servoB);
  // servo C
  MG_servoC.attach(servoC);
  // servo D
  MG_servoD.attach(servoD);

  sgServo_R.attach(sg_R);

  sgServo_L.attach(sg_L);

  Serial.begin(9600);
}


void loop() {
  if (run_flag == true) {
    for (int i = 0; i < 1; i++) {
      //antiRollover(1300); // antiRollOver protection retract/extend
      initialArm();
      //mission();
      pickUp();
      //fakeDelay(3000);
      //putBalls();
      if (i == 0) {
        fakeDelay(5000);
        //temp_pickBackInit();
        //initialArm();
        //pickUp();
        run_flag = false;
        //stop(direct1, direct2, direct3, direct4);
      }
    }
  }

}

void temp_pickBackInit() {

  MG_servoB.write(0);
  MG_servoC.write(0);
  MG_servoD.write(0);
  servoAA(0, 200, 80);
}

void initialArm() {
  //MG_servoA.write(20);
  //MG_servoB.write(0);
  servoAA(1, 400, 80);
  servoBB(0, 0, 5);
  MG_servoC.write(0);
  MG_servoD.write(0);
}

void pickUp() {

  //MG_servoA.write(20);
  //MG_servoB.write(20);
  
  servoBB(1, 300, 5);
  MG_servoC.write(30);
  MG_servoD.write(95);
  //fakeDelay(500);
  servoAA(1, 100, 20);
}

void putBalls() {
  servoAA(1, 500, 100);

  MG_servoB.write(180);

  MG_servoC.write(60);
  fakeDelay(1000);
  MG_servoD.write(180);
}

void mission() {

  move_forward(1850);
  rotate_cw(80);
  stop(direct1, direct2, direct3, direct4);
  fakeDelay(500);
  pickUp();

  move_left(1400);
  stop(direct1, direct2, direct3, direct4);
  fakeDelay(500);
  /*
    rotate_cw(1570);
    stop(direct1, direct2, direct3, direct4);
    fakeDelay(500);

    move_left(800);*/
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
/*
  void servoAA(int preAngle, int nextAngle) { //0 for CW; 1 for CCW
  int pos = 0;
  if (nextAngle > preAngle) {
    for (pos = preAngle; pos <= nextAngle; pos += motorStep) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      MG_servoA.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
  else {
    for (pos = preAngle; pos >= nextAngle; pos -= motorStep) { // goes from 180 degrees to 0 degrees
      MG_servoA.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
  }*/
void servoAA(int direct, int duration, int lock) { //0 for CW; 1 for CCW
  int servoSpeedDirect;
  if (direct == 1) {
    servoSpeedDirect = C_CLOCKWISE;
    lock += 1500;
  }
  else {
    servoSpeedDirect = CLOCKWISE;
    lock = 1500 - lock;
  }
  //MG_servoB.detach();
 /* MG_servoC.detach();
  MG_servoD.detach();*/

  MG_servoA.writeMicroseconds(servoSpeedDirect);
  fakeDelay(duration);
  MG_servoA.writeMicroseconds(lock); // lock
  
  /*//MG_servoB.attach(servoB);
  MG_servoC.attach(servoC);
  MG_servoD.attach(servoD);*/
}

void servoBB(int direct, int duration, int lock) { //0 for CW; 1 for CCW
  int servoSpeedDirect;
  if (direct == 1) {
    servoSpeedDirect = C_CLOCKWISE;
    lock += 1500;
  }
  else {
    servoSpeedDirect = CLOCKWISE;
    lock = 1500 - lock;
  }

  
  //MG_servoA.detach();
 /* MG_servoC.detach();
  MG_servoD.detach();*/

  MG_servoB.writeMicroseconds(servoSpeedDirect);
  fakeDelay(duration);
  MG_servoB.writeMicroseconds(lock); // lock
  
  /*//MG_servoB.attach(servoA);
  MG_servoC.attach(servoC);
  MG_servoD.attach(servoD);*/
}
//void servoBB(int preAngle, int nextAngle) { //0 for CW; 1 for CCW
//  int pos = 0;
//  if (nextAngle > preAngle) {
//    for (pos = preAngle; pos <= nextAngle; pos += motorStep) { // goes from 0 degrees to 180 degrees
//      // in steps of 1 degree
//      MG_servoB.write(pos);              // tell servo to go to position in variable 'pos'
//      delay(15);                       // waits 15ms for the servo to reach the position
//    }
//  }
//  else {
//    for (pos = preAngle; pos >= nextAngle; pos -= motorStep) { // goes from 180 degrees to 0 degrees
//      MG_servoB.write(pos);              // tell servo to go to position in variable 'pos'
//      delay(15);                       // waits 15ms for the servo to reach the position
//    }
//  }
//}

void servoCC(int direct, int duration, int lock) { //0 for CW; 1 for CCW
  int servoSpeedDirect;
  if (direct == 1) {
    servoSpeedDirect = C_CLOCKWISE;
    lock += 1500;
  }
  else {
    servoSpeedDirect = CLOCKWISE;
    lock = 1500 - lock;
  }
  MG_servoC.writeMicroseconds(servoSpeedDirect);
  fakeDelay(duration);
  MG_servoC.writeMicroseconds(lock); // lock
}

void servoDD(int direct, int duration, int lock) { //0 for CW; 1 for CCW
  int servoSpeedDirect;
  if (direct == 1) {
    servoSpeedDirect = C_CLOCKWISE;
    lock += 1500;
  }
  else {
    servoSpeedDirect = CLOCKWISE;
    lock = 1500 - lock;
  }
  MG_servoD.writeMicroseconds(servoSpeedDirect);
  fakeDelay(duration);
  MG_servoD.writeMicroseconds(lock); // lock
}

void armExtend() {

  MG_servoA.writeMicroseconds(1530); // lock
  MG_servoB.writeMicroseconds(1520); // lock
}

void antiRollover(int rotateTime) {
  if (anti_roll_extend_flag == false) {
    sgServo_R.writeMicroseconds(C_CLOCKWISE);
    sgServo_L.writeMicroseconds(CLOCKWISE);
    fakeDelay(rotateTime);
    sgServo_R.writeMicroseconds(STOP);
    sgServo_L.writeMicroseconds(STOP);
    anti_roll_extend_flag = true;
  }
  else {
    sgServo_R.writeMicroseconds(CLOCKWISE);
    sgServo_L.writeMicroseconds(C_CLOCKWISE);
    fakeDelay(rotateTime);
    sgServo_R.writeMicroseconds(STOP);
    sgServo_L.writeMicroseconds(STOP);
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

void rotate_ccw(int duration) {
  direct1 = 1; direct2 = 1; direct3 = 0; direct4 = 0;
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
