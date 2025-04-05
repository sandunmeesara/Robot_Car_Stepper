#include <AccelStepper.h>
#include <Servo.h>

#define MOTOR_INTERFACE_TYPE 4
#define TRIG_PIN A0  // Use A0 as a digital output
#define ECHO_PIN A1  // Use A1 as a digital input

//--------------------------------------- Global Variables Section ------------------------------------------

bool isMoving = false;
unsigned long previousMillis = 0; // Stores last time event occurred
const long interval = 100; // Time interval in milliseconds (1 second)
bool isLeftBlocked = false;
bool isRightBlocked = false;
int pos = 0;    // variable to store the servo position

// Steps per degree (4096 steps per revolution / 360 degrees)
const float STEPS_PER_DEGREE = 2048.0 / 360.0;
const float distance_for_one_revolution = 21;
const int steps_per_one_revolution = 2048;

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
void setup()
{  
    Serial.begin(9600);

    // Pin Definitions for Ultrasonic Sensor
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Attaches the servo on pin 9 to the Servo object
    myservo.attach(10);  

    // Stepper Motor configurations
    //Left Stepper
    stepper1.setMaxSpeed(1000.0);
    stepper1.setAcceleration(1000.0);
    stepper1.moveTo(1000000); // This is the maximum distance left stepper motor will run 
    //Right Stepper
    stepper2.setMaxSpeed(1000.0);
    stepper2.setAcceleration(1000.0);
    stepper2.moveTo(1000000); // This is the maximum distance right stepper motor will run
}

//----------------------------- Loop function ---------------------------------------
void loop()
{
    unsigned long currentMillis = millis(); // Get current time

    // Check the time interval and execute the measuring functions
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis; // Update last event time

      // Measure the distance
      float distance = measure_distance();
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");

      // Check the distance is less than the pre given value
      if(distance <= 30){
        // Stop motors
        stopMotors();
        //Check the left side of the car for a free space to move
        for (pos = 90; pos <= 180; pos += 1) { // goes from 90 degrees to 140 degrees
          myservo.write(pos);              // tell servo to go to position in variable 'pos'
          delay(100);                       // waits 15 ms for the servo to reach the position

          if(measure_distance() > 60){
            isLeftBlocked = false;
            Serial.println("Left is Okay!");

            float angle = pos -(pos/2);
            turnLeftAngle(angle);
            int distance_to_go_after_turn = abs(((distance / cos(angle)) / distance_for_one_revolution) * steps_per_one_revolution);
            move_Blocking(distance_to_go_after_turn);
            turnRightAngle(angle*2);
            move_Blocking(distance_to_go_after_turn);
            turnLeftAngle(angle);

            break;
          }else{
            isLeftBlocked = true;
            Serial.println("Left is Blocked!");
          }
        }

        // Check the right side of the car for a free space to move
        if(isLeftBlocked){
          for (pos = 90; pos >= 0; pos -= 1) { // goes from 90 degrees to 40 degrees
            myservo.write(pos);              // tell servo to go to position in variable 'pos'
            delay(100);                       // waits 15 ms for the servo to reach the position

            if(measure_distance() > 60){
              isRightBlocked = false;
              Serial.println("Right is okay!");

              float angle = pos-(pos/2);
              turnRightAngle(angle);
              int distance_to_go_after_turn = abs(((distance / cos(angle)) / distance_for_one_revolution) * steps_per_one_revolution);
              move_Blocking(distance_to_go_after_turn);
              turnLeftAngle(angle*2);
              move_Blocking(distance_to_go_after_turn);
              turnRightAngle(angle);

              break;
            }else{
              isRightBlocked = true;
              Serial.println("Right is blocked!");
            }
          }
        }
      }
      isLeftBlocked = false;
      isRightBlocked =false;
      myservo.write(90);     // tell servo to go to home position
      delay(15); 
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

//---------------------------------------- Main Body Section End ------------------------------------------------

// --------------------------------------- Functions Section ---------------------------------------------

// Function to execute the commands coming from the serial port
void executeCommand(char command) {
  if (command == '1'){
    moveForward(10000000);
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

// Function to turn the car to left by a given angle
void turnLeftAngle(float angle) {
  long steps = angle * STEPS_PER_DEGREE;
  stepper1.move(-steps);
  stepper2.move(steps);
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
  }
}

// Function to turn the car to right by a given angle
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

// Function to measure the distance using ultrasonic sensor
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

// Blocking function to move relative distance
void move_Blocking(long steps) {
  stepper1.move(steps);
  stepper2.move(steps);
  while(stepper1.distanceToGo() != 0 && stepper2.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
  }
}

// --------------------------------------- Functions Section End ---------------------------------------------