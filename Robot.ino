

#include <ESP32Servo.h>
#include <Ps3Controller.h>


#define baseServoPin 5
#define arm1ServoPin 18
#define arm2ServoPin 19
#define armRotationServoPin 21

#define gripperPin1 22
#define gripperPin2 23

#define wheelRPin1 12
#define wheelRPin2 14

#define wheelLPin1 27
#define wheelLPin2 26



class MyServo {

public:
  MyServo(int servoPin, int initialPosition = 500, int defaultPosition = 500, int stepMs = 10) {
    this->pin = servoPin;
    this->stepMs = stepMs;
    this->defaultPosition = defaultPosition;
    this->initialPosition = initialPosition;
    this->currentPosition = this->initialPosition;
    this->servo = Servo();
    this->servo.attach(servoPin, this->minMs, this->maxMs);                                                       // 2 ostatnie argumenty odpowiadają za zakres ruchu, czas górki w ms

    this->servo.writeMicroseconds(this->currentPosition);
  }

  void moveRight() {
    if (this->currentPosition >= this->maxMs) {
      return;
    }
    this->currentPosition += this->stepMs;
    this->servo.writeMicroseconds(currentPosition);
  }

  void moveLeft() {
    if (currentPosition <= this->minMs) {
      return;
    }
    this->currentPosition -= this->stepMs;
    this->servo.writeMicroseconds(currentPosition);
  }

  void moveDefaultPosition() {
    this->servo.writeMicroseconds(this->defaultPosition);
  }

  void moveInitialPosition() {
    this->servo.writeMicroseconds(this->initialPosition);
  }

  void moveTo(int degree) {
    this->servo.write(degree);
  }


private:
  Servo servo;
  int pin;
  int initialPosition = 500;  // ms
  int defaultPosition;
  int currentPosition;
  int stepMs;

  int minMs = 500;
  int maxMs = 2200;
};


class Engine {
public:
  Engine(int pin1, int pin2) {
    this->pin1 = pin1;
    this->pin2 = pin2;
    this->statePin1 = 0;
    this->statePin2 = 0;

    pinMode(this->pin1, OUTPUT);
    pinMode(this->pin2, OUTPUT);
    this->updateState();
  }

  void right() {
    this->changeState(0, 1);
    this->updateState();
  }


  void left() {
    this->changeState(1, 0);
    this->updateState();
  }


  void stop() {
    this->changeState(0, 0);
    this->updateState();
  }


  void changeState(int s1, int s2) {
    this->statePin1 = s1;
    this->statePin2 = s2;
  }


  void updateState() {
    digitalWrite(this->pin1, this->statePin1);
    digitalWrite(this->pin2, this->statePin2);
  }

private:
  int pin1;
  int pin2;

  int statePin1;
  int statePin2;
};


const int defPosBase = 1350;
const int defPosArm1 = 1630;
const int defPosArm2 = 1630;
const int defPosArmRotation = 1210;


const int neutralPosBase = 1350;
const int neutralPosArm1 = 2200;
const int neutralPosArm2 = 2200;
const int neutralPosArmRotation = 1210;


MyServo baseServo = MyServo(baseServoPin, defPosBase, neutralPosBase);
MyServo arm1Servo = MyServo(arm1ServoPin, defPosArm1, neutralPosArm1);
MyServo arm2Servo = MyServo(arm2ServoPin, defPosArm2, neutralPosArm2);
MyServo armRotationServo = MyServo(armRotationServoPin, defPosArmRotation, neutralPosArmRotation);

Engine gripper(gripperPin1, gripperPin2);

Engine wheelRight(wheelRPin1, wheelRPin2);
Engine wheelLeft(wheelLPin1, wheelLPin2);






void notify() {

  // base movement
  if (Ps3.data.button.right) {
    baseServo.moveLeft();
    Serial.println("RIGHT");
  }
  if (Ps3.data.button.left) {
    baseServo.moveRight();
    Serial.println("LEFT");
  }

  // movement arm 1
  if (Ps3.data.button.up) {
    arm1Servo.moveRight();
    Serial.println("UP");
  }
  if (Ps3.data.button.down) {
    arm1Servo.moveLeft();
    Serial.println("DOWN");
  }


  // movement arm 2
  if (Ps3.data.button.square) {
    arm2Servo.moveRight();
    Serial.println("SQUARE");
  }
  if (Ps3.data.button.circle) {
    arm2Servo.moveLeft();
    Serial.println("CIRCLE");
  }


  // gripper rotation
  if (Ps3.data.button.triangle) {
    armRotationServo.moveRight();
    Serial.println("TRIANGLE");
  }
  if (Ps3.data.button.cross) {
    armRotationServo.moveLeft();
    Serial.println("CROSS");
  }


  // gripper
  if (Ps3.event.button_down.l1) {
    gripper.left();
    Serial.println("Pressed L1");
  }
  if (Ps3.event.button_up.l1) {
    gripper.stop();
    Serial.println("Released L1");
  }
  if (Ps3.event.button_down.r1) {
    gripper.right();
    Serial.println("Pressed R1");
  }
  if (Ps3.event.button_up.r1) {
    gripper.stop();
    Serial.println("Released R1");
  }

  // arm default position
  if (Ps3.event.button_down.l2) {
    baseServo.moveDefaultPosition();
    arm1Servo.moveDefaultPosition();
    arm2Servo.moveDefaultPosition();
    armRotationServo.moveDefaultPosition();
  }

  // arm initial position
  if (Ps3.event.button_down.r2) {
    baseServo.moveInitialPosition();
    arm1Servo.moveInitialPosition();
    arm2Servo.moveInitialPosition();
    armRotationServo.moveInitialPosition();
  }


  // wheels
  if (Ps3.data.analog.stick.lx > 80) {
    wheelLeft.right();
    wheelRight.left();
    Serial.println("Going Right");
  }
  if (Ps3.data.analog.stick.lx < -80) {
    wheelRight.right();
    wheelLeft.left();
    Serial.println("Going Left");
  }

  if (Ps3.data.analog.stick.ry > 80) {
    wheelRight.right();
    wheelLeft.right();
    Serial.println("Going Forward");
  }
  if (Ps3.data.analog.stick.ry < -80) {
    wheelRight.left();
    wheelLeft.left();
    Serial.println("Going Back");
  }

  if (Ps3.data.analog.stick.lx > -60 && Ps3.data.analog.stick.lx < 60
      && Ps3.data.analog.stick.ry > -60 && Ps3.data.analog.stick.ry < 60) {
    wheelRight.stop();
    wheelLeft.stop();
  }
}


void connected() {
  Serial.println("Pad connected");
}


void disconnected() {
  Serial.println("Pad disconnected");
}



void setup() {
  //  Sets bitrate for data transmition                                      Ustawia szybkość transmisji danych w bitach na sekundę
  Serial.begin(115200);
  
  // Allocation of timers for proper functioning
  // of the ESP32Servo library
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Initialization of the Ps3 Controller Host library
  // The 'attach' functions add callback functions
  // to the respective events
  Ps3.begin();
  Ps3.attachOnConnect(connected);
  Ps3.attachOnDisconnect(disconnected);
  Ps3.attach(notify);
}


// Alokacja timerów do poprawnego działania
// biblioteki ESP32Servo

    // Inicjalizacja bilioteki Ps3 Controller Host 
    // funkcje attach dodają callback functions
    // do odpowiednich eventów

void loop() {

}


