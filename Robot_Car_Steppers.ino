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
const float STEPS_PER_DEGREE = 4096.0 / 360.0;

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
      if(distance <= 20){

        // Check the left side of the car for a free space to move
        for (pos = 90; pos <= 140; pos += 1) { // goes from 0 degrees to 180 degrees
          myservo.write(pos);              // tell servo to go to position in variable 'pos'
          delay(15);                       // waits 15 ms for the servo to reach the position

          if(measure_distance() < 40){
            isLeftBlocked = true;
            Serial.println("Left is blocked!");
            // turnLeftAngle(pos);
            break;
          }
        }

        // Check the right side of the car for a free space to move
        if(isLeftBlocked){
          for (pos = 0; pos >= 90; pos -= 1) { // goes from 180 degrees to 0 degrees
            myservo.write(pos);              // tell servo to go to position in variable 'pos'
            delay(15);                       // waits 15 ms for the servo to reach the position

            if(measure_distance() < 40){
              isRightBlocked = true;
              Serial.println("Right is blocked!");
              // turnRightAngle(pos);
              break;
            }
          }
        }
      }
      isLeftBlocked = false;
      isRightBlocked =false;
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

// Function to move the car to forward
void moveForward() {
  stepper1.moveTo(1000000);
  stepper2.moveTo(1000000);
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

// --------------------------------------- Functions Section End ---------------------------------------------