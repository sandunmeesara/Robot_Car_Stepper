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
    stepper1.setAcceleration(5000.0);
    stepper1.moveTo(1000000); // This is the maximum distance left stepper motor will run 
    //Right Stepper
    stepper2.setMaxSpeed(1000.0);
    stepper2.setAcceleration(5000.0);
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

    // Check the distance is less than the pre given value
    if(distance <= 30){
      stopMotors();
      avoidObstacles(distance); // Call the function to avoid obstacles
    }
  }

  processSerialCommands();

  if (isMoving == true){
    stepper1.run();
    stepper2.run();
  } 
}

//---------------------------------------- Main Body Section End ------------------------------------------------

// --------------------------------------- Functions Section ---------------------------------------------

void writeToServo(int pos){
  myservo.write(pos);       
  delay(50); 
}

void avoidObstacles(float distance){
  for (pos = 90; pos <= 180; pos += 1) {
    writeToServo(pos);
    float angle = pos-(pos/2); // calculate angle
    isLeftBlocked = checkDirection(LEFT,distance,angle);
    if(isLeftBlocked == UNBLOCKED){
      break;
    }
  }

  if(isLeftBlocked == BLOCKED){
    for (pos = 90; pos >= 0; pos -= 1) { // goes from 90 degrees to 40 degrees
      writeToServo(pos);
      float angle = pos-(pos/2); // calculate angle
      isRightBlocked = checkDirection(RIGHT,distance,angle);
      if(isRightBlocked == UNBLOCKED){
        break;
      }
    }
  }

  isLeftBlocked = RESET;
  isRightBlocked = RESET;
  writeToServo(90);
}

bool checkDirection(bool direction,float distance,float angle){
  /// Return: 0-Unblocked , 1-Blocked
  int steps_to_go = distance_to_go_after_turn(distance, angle);
  if(measure_distance() > 60){
    if(direction == LEFT){
      turnToAngle(angle, LEFT);
      move_Blocking(steps_to_go);
      turnToAngle(angle*2, RIGHT);
      move_Blocking(steps_to_go);
      turnToAngle(angle, LEFT);
    }else{
      turnToAngle(angle, RIGHT);
      move_Blocking(steps_to_go);
      turnToAngle(angle*2, LEFT);
      move_Blocking(steps_to_go);
      turnToAngle(angle, RIGHT);
    }
    return UNBLOCKED;
  }else{
    return BLOCKED;
  }
}

int distance_to_go_after_turn(float distance,float angle){
  return abs(((distance / cos(angle)) / distance_for_one_revolution) * steps_per_one_revolution);
}

// Function to process serial commands
void processSerialCommands(){
  if (Serial.available() > 0) {
    char command = Serial.read();
    executeCommand(command);
  }
}

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
    turnToAngle(angle, LEFT);
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

void turnToAngle(float angle,bool direction){
  long steps = angle * STEPS_PER_DEGREE;
  if(direction == RIGHT){
    stepper1.move(steps);
    stepper2.move(-steps);
  }else{
    stepper1.move(-steps);
    stepper2.move(steps);
  }
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