#include <AccelStepper.h>
#define MOTOR_INTERFACE_TYPE 4

// Define some steppers and the pins the will use
AccelStepper stepper1(MOTOR_INTERFACE_TYPE, 2, 3, 4, 5);
AccelStepper stepper2(MOTOR_INTERFACE_TYPE, 6, 7, 8, 9);

// Define variables
bool isMoving = false;

// Steps per degree (4096 steps per revolution / 360 degrees)
const float STEPS_PER_DEGREE = 4096.0 / 360.0;

void setup()
{  
    Serial.begin(9600);
    stepper1.setMaxSpeed(1000.0);
    stepper1.setAcceleration(1000.0);
    stepper1.moveTo(1000000);
    
    stepper2.setMaxSpeed(1000.0);
    stepper2.setAcceleration(1000.0);
    stepper2.moveTo(1000000);
}

void loop()
{
    // Check for serial commands
  if (Serial.available() > 0) {
    char command = Serial.read();
    executeCommand(command);
  }

  if (isMoving == true){
    stepper1.run();
    stepper2.run();
  } 
}

void executeCommand(char command) {
  if (command == '1'){
    moveForward();
  }else if(command == '2'){
    moveBackward();
  }else if(command == '3'){
    turnLeft();
  }else if(command == '4'){
    turnRight();
  }else if(command == '5'){
    stopMotors();
  }else if(command == 't'){
    int angle = Serial.parseInt();
    turnLeftAngle(angle);
  }else{
    Serial.println("Invalid command");
  }
  
}

void moveForward() {
  stepper1.moveTo(1000000);
  stepper2.moveTo(1000000);
  isMoving = true;
}

void moveBackward() {
  stepper1.moveTo(-1000000);
  stepper2.moveTo(-1000000);
  isMoving = true;
}

void turnRight() {
  stepper1.moveTo(1000000);
  stepper2.moveTo(-1000000);
  isMoving = true;
}

void turnLeft() {
  stepper1.moveTo(-1000000);
  stepper2.moveTo(1000000);
  isMoving = true;
}

void stopMotors() {
  stepper1.stop();
  stepper2.stop();
  isMoving = false;
}

void turnLeftAngle(float angle) {
  long steps = angle * STEPS_PER_DEGREE;
  stepper1.move(-steps);
  stepper2.move(steps);
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
  }
}

void turnRightAngle(float angle) {
  long steps = angle * STEPS_PER_DEGREE;
  Serial.print("steps");
  Serial.println(steps);
  stepper1.move(steps);
  stepper2.move(-steps);
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
  }
}
