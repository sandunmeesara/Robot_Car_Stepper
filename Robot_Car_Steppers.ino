#include <AccelStepper.h>
#include <Servo.h>

Servo myservo;  // create Servo object to control a servo
int pos = 0;    // variable to store the servo position

#define MOTOR_INTERFACE_TYPE 4
#define TRIG_PIN A0  // Use A0 as a digital output
#define ECHO_PIN A1  // Use A1 as a digital input

// Define some steppers and the pins the will use
AccelStepper stepper1(MOTOR_INTERFACE_TYPE, 2, 3, 4, 5);
AccelStepper stepper2(MOTOR_INTERFACE_TYPE, 6, 7, 8, 9);

// Define variables
bool isMoving = false;
unsigned long previousMillis = 0; // Stores last time event occurred
const long interval = 100; // Time interval in milliseconds (1 second)

// Steps per degree (4096 steps per revolution / 360 degrees)
const float STEPS_PER_DEGREE = 4096.0 / 360.0;

void setup()
{  
    Serial.begin(9600);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    myservo.attach(10);  // attaches the servo on pin 9 to the Servo object

    stepper1.setMaxSpeed(1000.0);
    stepper1.setAcceleration(1000.0);
    stepper1.moveTo(1000000);
    
    stepper2.setMaxSpeed(1000.0);
    stepper2.setAcceleration(1000.0);
    stepper2.moveTo(1000000);
}

void loop()
{
    unsigned long currentMillis = millis(); // Get current time

    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis; // Update last event time
      float distance = measure_distance();
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");

      if(distance <= 20){
        for (pos = 40; pos <= 140; pos += 1) { // goes from 0 degrees to 180 degrees
          // in steps of 1 degree
          myservo.write(pos);              // tell servo to go to position in variable 'pos'
          delay(15);                       // waits 15 ms for the servo to reach the position
        }
        for (pos = 140; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
          myservo.write(pos);              // tell servo to go to position in variable 'pos'
          delay(15);                       // waits 15 ms for the servo to reach the position
        }
      }
    }

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

float measure_distance(){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2;

  return distance;
}
