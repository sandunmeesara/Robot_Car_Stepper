#include <TimerOne.h>
#include <AccelStepper.h>
#include <Servo.h>

#define MOTOR_INTERFACE_TYPE 4
#define TRIG_PIN A0  // Use A0 as a digital output
#define ECHO_PIN A1  // Use A1 as a digital input

#define RIGHT 1
#define LEFT 0
#define BLOCKED 1
#define UNBLOCKED 0
#define RESET 0
#define TIME_INTERVAL 1000000

#define CM_PER_DEGREE 0.14
#define STEPS_PER_CM 98
#define STEPS_PER_DEGREE 2048.0 / 360.0
#define DISTANCE_FOR_REVOLUTION 21
#define STEPS_FOR_REVOLUTION 2048

//--------------------------------------- Global Variables Section ------------------------------------------

bool isMoving = false;
unsigned long previousMillis = 0;  // Stores last time event occurred
const long interval = 100;         // Time interval in milliseconds (1 second)
bool isLeftBlocked = false;
bool isRightBlocked = false;
int pos = 0;  // variable to store the servo position
const int ledPin = 13;
volatile boolean ledState = LOW;

//--------------------------------------- Global Variables Section End --------------------------------------

//-------------------------------------- Object Creations Section -------------------------------------------

// create Servo object to control a servo
Servo myservo;

// Define some steppers and the pins the will use
AccelStepper stepper1(MOTOR_INTERFACE_TYPE, 2, 3, 4, 5);
AccelStepper stepper2(MOTOR_INTERFACE_TYPE, 6, 7, 8, 9);

//-------------------------------------- Object Creations Section End ---------------------------------------

//---------------------------------------- Main Body Section ------------------------------------------------

//------------------------- Setup function ---------------------------------------
void setup() {
  Serial.begin(9600);

  // Timer Initialize
  // Timer1.initialize(TIME_INTERVAL);
  // Timer1.attachInterrupt(checkForObstacles);

  pinMode(ledPin, OUTPUT);

  // Pin Definitions for Ultrasonic Sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Attaches the servo on pin 9 to the Servo object
  myservo.attach(10);

  // Stepper Motor configurations
  //Left Stepper
  stepper1.setMaxSpeed(1000.0);
  stepper1.setAcceleration(5000.0);
  stepper1.moveTo(1000000);  // This is the maximum distance left stepper motor will run
  //Right Stepper
  stepper2.setMaxSpeed(1000.0);
  stepper2.setAcceleration(5000.0);
  stepper2.moveTo(1000000);  // This is the maximum distance right stepper motor will run
}

//----------------------------- Loop function ---------------------------------------
void loop() {
  processSerialCommands();

  if (isMoving == true) {
    stepper1.run();
    stepper2.run();
  }
}

//---------------------------------------- Main Body Section End ------------------------------------------------

// --------------------------------------- Functions Section ---------------------------------------------

int calculateDistanceFromAngle(float angle) {
  return angle * CM_PER_DEGREE * STEPS_PER_CM;
}

// Timer Interrupt Service Routine
void timerISR() {
  ledState = !ledState;
  digitalWrite(ledPin, ledState);
}

void checkForObstacles() {
  // Serial.println("EF1");
  // Measure the distance
  float distance = measure_distance();

  // Check the distance is less than the pre given value
  if (distance <= 30) {
    stopMotors();
    avoidObstacles(distance);  // Call the function to avoid obstacles
  }

  timerISR();
  // Serial.println("eF1");
}

void writeToServo(int pos) {
  myservo.write(pos);
  delay(50);
}

void avoidObstacles(float distance) {
  for (pos = 90; pos <= 180; pos += 1) {
    writeToServo(pos);
    float angle = pos - (pos / 2);  // calculate angle
    isLeftBlocked = checkDirection(LEFT, distance, angle);
    if (isLeftBlocked == UNBLOCKED) {
      break;
    }
  }

  if (isLeftBlocked == BLOCKED) {
    for (pos = 90; pos >= 0; pos -= 1) {  // goes from 90 degrees to 40 degrees
      // Serial.println(pos);
      writeToServo(pos);
      float angle = pos - (pos / 2);  // calculate angle
      isRightBlocked = checkDirection(RIGHT, distance, angle);
      if (isRightBlocked == UNBLOCKED) {
        break;
      }
    }
  }

  isLeftBlocked = RESET;
  isRightBlocked = RESET;
  writeToServo(90);
}

bool checkDirection(bool direction, float distance, float angle) {
  /// Return: 0-Unblocked , 1-Blocked
  int steps_to_go = distance_to_go_after_turn(distance, angle);
  if (measure_distance() > 60) {
    if (direction == LEFT) {
      turnToAngle(angle, LEFT);
      move_Blocking(steps_to_go);
      turnToAngle(angle * 2, RIGHT);
      move_Blocking(steps_to_go);
      turnToAngle(angle, LEFT);
    } else {
      turnToAngle(angle, RIGHT);
      move_Blocking(steps_to_go);
      turnToAngle(angle * 2, LEFT);
      move_Blocking(steps_to_go);
      turnToAngle(angle, RIGHT);
    }
    return UNBLOCKED;
  } else {
    return BLOCKED;
  }
}

int distance_to_go_after_turn(float distance, float angle) {
  return abs(((distance / cos(angle)) / DISTANCE_FOR_REVOLUTION) * STEPS_FOR_REVOLUTION);
}

// Function to process serial commands
void processSerialCommands() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    executeCommand(command);
  }
}

// Function to execute the commands coming from the serial port
void executeCommand(char command) {
  if (command == '1') {
    // Serial.println("1");
    moveForward(10000000);
  } else if (command == '2') {
    // Serial.println("2");
    moveBackward();
  } else if (command == '3') {
    // Serial.println("3");
    turnLeft();
  } else if (command == '4') {
    // Serial.println("4");
    turnRight();
  } else if (command == '5') {
    // Serial.println("5");
    stopMotors();
  } else if (command == 't') {
    // Serial.println("t command");
    float angle = Serial.parseFloat();
    // Serial.print("C:");
    // Serial.println(angle);
    turnToAngle(angle, LEFT);
    
  }
}

// Function to move the car to forward
void moveForward(long steps) {
  stepper1.moveTo(steps);
  stepper2.moveTo(steps);
  isMoving = true;
}

// Function to move the car to backward
void moveBackward() {
  stepper1.moveTo(-1000000);
  stepper2.moveTo(-1000000);
  isMoving = true;
}

// Function to turn the car to right
void turnRight() {
  stepper1.moveTo(1000000);
  stepper2.moveTo(-1000000);
  isMoving = true;
}

// Function to turn the car to left
void turnLeft() {
  stepper1.moveTo(-1000000);
  stepper2.moveTo(1000000);
  isMoving = true;
}

// Function to stop the car
void stopMotors() {
  stepper1.stop();
  stepper2.stop();
  isMoving = false;
}

void turnToAngle(float angle, bool direction) {
  long steps = calculateDistanceFromAngle(angle);
  if (direction == RIGHT) {
    stepper1.move(steps);
    stepper2.move(-steps);
  } else {
    stepper1.move(-steps);
    stepper2.move(steps);
  }
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
  }
}

// Function to measure the distance using ultrasonic sensor
float measure_distance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2;

  return distance;
}

// Blocking function to move relative distance
void move_Blocking(long steps) {
  stepper1.move(steps);
  stepper2.move(steps);
  while (stepper1.distanceToGo() != 0 && stepper2.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
  }
}

// --------------------------------------- Functions Section End ---------------------------------------------