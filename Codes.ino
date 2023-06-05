#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  650 // this is the 'maximum' pulse length count (out of 4096)

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
#define CIN1 26 //Direction
#define CIN2 27 //Direction
//Motor D
#define PWMD 5 //Speed control
#define DIN1 28 //Direction
#define DIN2 29 //Direction

//button
#define buttonPin 30

int buttonState = 0;
int direct1 = 999;
int direct2 = 999;
int direct3 = 999;
int direct4 = 999;
const int roboSpeed = 78; // 0~255
const int motorStep = 1;
unsigned long currentTime = 0; // cur time
unsigned long previousTime = 0; // pre time
bool run_flag = true;
bool mission_flag = false;

void setup() {

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(9600);

}


void loop() {

  missionButton();
  
  if (mission_flag == true) {
    fakeDelay(3000);

    while (run_flag == true) {
      //run_flag = false;
      mission();
    }

  }
  else {

    initialArm();
    stop(direct1, direct2, direct3, direct4);
  }


  /*if (run_flag == true) {

    for (int i = 0; i < 1; i ++) {

      //mission();
      //initialArm();

      //temp();
       move_backward(150);

        move_right(800);

        rotate_ccw(1685);

        pickUP_squash();

        move_left(1550);

        pwm.setPWM(3, 0, angleToPulse(50)); //100

        move_forward(150);

        fakeDelay(500);

        move_right(350);

        pwm.setPWM(0, 0, angleToPulse(130)); //100

        // 前往silo
        move_forward(1380);

        // 调节大臂
        pwm.setPWM(0, 0, angleToPulse(150)); //100
        pwm.setPWM(2, 0, angleToPulse(0));

        // 向silo放置squash ball
        move_left(1470);

        pwm.setPWM(1, 0, angleToPulse(200));

        for (int i = 50; i < 230; i ++ )
        {
        pwm.setPWM(3, 0, angleToPulse(i));
        }


      if (i == 0)
      {
        fakeDelay(5000);
        stop(direct1, direct2, direct3, direct4);
        temp();
        run_flag = false;
      }

    }
    }

  */
}


void temp() {

  pwm.setPWM(0, 0, angleToPulse(130)); //100
  pwm.setPWM(1, 0, angleToPulse(130)); //100
  pwm.setPWM(2, 0, angleToPulse(105)); //140
  pwm.setPWM(3, 0, angleToPulse(205)); //100
}

void missionButton() {

  buttonState = digitalRead(buttonPin);

  if (buttonState == LOW) {  // 如果按钮被按下
    mission_flag = true;
  }
  else
  { // 如果按钮未被按下
    mission_flag = false;
  }
}


void initialArm() {
  pwm.setPWM(0, 0, angleToPulse(0));
  pwm.setPWM(1, 0, angleToPulse(230));
  pwm.setPWM(2, 0, angleToPulse(150));
  pwm.setPWM(3, 0, angleToPulse(255)); // 50 ~ 290
}

void pickUp() {
  pwm.setPWM(3, 0, angleToPulse(60));
  fakeDelay(500);
  pwm.setPWM(0, 0, angleToPulse(150));
  fakeDelay(500);
  pwm.setPWM(2, 0, angleToPulse(140));
  fakeDelay(500);
  pwm.setPWM(1, 0, angleToPulse(230));
}

void lvlUpBalls() {

  pwm.setPWM(3, 0, angleToPulse(50));
  fakeDelay(500);

  move_right(300);
  stop(direct1, direct2, direct3, direct4);

  for (int i = 150; i > 100; i--) {

    pwm.setPWM(0, 0, angleToPulse(i)); //150 -> 100
  }

  //pwm.setPWM(3, 0, angleToPulse(100));
  pwm.setPWM(1, 0, angleToPulse(190));
  pwm.setPWM(2, 0, angleToPulse(140));

}

void putBalls() {
  bool numFour = false;

  for (int i = 100; i < 120; i += 5) {
    pwm.setPWM(0, 0, angleToPulse(i)); //100
  }

  for (int j = 190; j > 140; j -= 5) {
    pwm.setPWM(1, 0, angleToPulse(j)); //100
  }

  for (int k = 140; k > 100; k -= 5) {
    pwm.setPWM(2, 0, angleToPulse(k)); //140

  }

  fakeDelay(2000);
  pwm.setPWM(3, 0, angleToPulse(180)); //100

}

void pickUP_squash() {
  pwm.setPWM(3, 0, angleToPulse(60));
  fakeDelay(500);
  pwm.setPWM(0, 0, angleToPulse(150));
  fakeDelay(100);
  pwm.setPWM(2, 0, angleToPulse(140));
  fakeDelay(100);
  pwm.setPWM(1, 0, angleToPulse(230));
}

void mission() {


  // 初始化arm
  initialArm();

  // 前进与silo平行
  //move_forward(1600);
  move_forward(1750); // 1680
  //rotate_ccw(40);
  stop(direct1, direct2, direct3, direct4);
  fakeDelay(500);

  // 抓取球
  pickUp();

  // 前进
  move_left(1500);
  stop(direct1, direct2, direct3, direct4);
  fakeDelay(500);

  // 抬升球体
  lvlUpBalls();

  // 放置球体姿态
  putBalls();

  //moving to silo
  move_left(1500);

  //moving forward
  move_forward(210);
  stop(direct1, direct2, direct3, direct4);

  //压低pipe
  pwm.setPWM(0, 0, angleToPulse(130)); //100
  pwm.setPWM(1, 0, angleToPulse(130)); //100

  fakeDelay(1000);
  for (int i = 180; i < 205; i++)
  {
    pwm.setPWM(3, 0, angleToPulse(i)); //100
  }

  /*--------------------------------------------------TENNIS BALL DONE-------------------------------------*/
  fakeDelay(1500);

  move_backward(150); //150 full charge 

  move_right(800);

  rotate_ccw(1695); // 1685 full charge

  pickUP_squash();

  move_left(1550);

  pwm.setPWM(3, 0, angleToPulse(50)); //100

  move_forward(150);

  fakeDelay(500);

  move_right(350);

  pwm.setPWM(0, 0, angleToPulse(130)); //100

  // 前往silo
  move_forward(1350); // 1380 full charge

  // 调节大臂
  pwm.setPWM(0, 0, angleToPulse(150)); //100
  pwm.setPWM(2, 0, angleToPulse(0));

  // 向silo放置squash ball
  move_left(1470);

  pwm.setPWM(1, 0, angleToPulse(200));

  fakeDelay(500);
  for (int i = 50; i < 240; i++ )
  {
    pwm.setPWM(3, 0, angleToPulse(i));
  }

    /*--------------------------------------------------TENNIS BALL DONE-------------------------------------*/
  fakeDelay(1500);
  
  move_right(777);
  move_forward(300);
  initialArm();
  
  run_flag = false;
}


/*-----------------------------------------------------------------------------------------*/

int angleToPulse(int ang) {
  int pulse = map(ang, 0, 270, SERVOMIN, SERVOMAX); // map angle of 0 to 180 to Servo min and Servo max
  //Serial.print("Angle: "); Serial.print(ang);
  //Serial.print(" pulse: "); Serial.println(pulse);
  return pulse;
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
  stop(direct1, direct2, direct3, direct4);
}

void move_backward(int duration) {
  direct1 = 0; direct2 = 0; direct3 = 0; direct4 = 0;
  move(1, roboSpeed, direct1); // A backward
  move(2, roboSpeed, direct2); // B backward
  move(3, roboSpeed, direct3); // C backward
  move(4, roboSpeed, direct4); // D backward
  fakeDelay(duration);
  stop(direct1, direct2, direct3, direct4);
}

void move_right(int duration) {
  direct1 = 1; direct2 = 0; direct3 = 1; direct4 = 0;
  move(1, roboSpeed, direct1); // A backward
  move(2, roboSpeed, direct2); // B forward
  move(3, roboSpeed, direct3); // C backward
  move(4, roboSpeed, direct4); // D forward
  fakeDelay(duration);
  stop(direct1, direct2, direct3, direct4);
}

void move_left(int duration) {
  direct1 = 0; direct2 = 1; direct3 = 0; direct4 = 1;
  move(1, roboSpeed, direct1); // A forward
  move(2, roboSpeed, direct2); // B backward
  move(3, roboSpeed, direct3); // C forward
  move(4, roboSpeed, direct4); // D backward
  fakeDelay(duration);
  stop(direct1, direct2, direct3, direct4);
}

void rotate_cw(int duration) {
  direct1 = 0; direct2 = 0; direct3 = 1; direct4 = 1;
  move(1, roboSpeed, direct1); // A forward
  move(2, roboSpeed, direct2); // B forward
  move(3, roboSpeed, direct3); // C backward
  move(4, roboSpeed, direct4); // D backward
  fakeDelay(duration);
  stop(direct1, direct2, direct3, direct4);
}

void rotate_ccw(int duration) {
  direct1 = 1; direct2 = 1; direct3 = 0; direct4 = 0;
  move(1, roboSpeed, direct1); // A forward
  move(2, roboSpeed, direct2); // B forward
  move(3, roboSpeed, direct3); // C backward
  move(4, roboSpeed, direct4); // D backward
  fakeDelay(duration);
  stop(direct1, direct2, direct3, direct4);
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
