// Initialize the Pololu QTR Sensors library for Arduino
#include <QTRSensors.h>
QTRSensors qtr;

// Initialize the eight sensors on our robot
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

const int PWMA = 6;   // PWM control for left wheel speed
const int AIN2 = 7;   // Direction control for left wheel
const int AIN1 = 8;   // Direction control for left wheel
const int STBY = 9;   // Standby pin to enable/disable motor driver
const int BIN1 = 10;  // Direction control for right wheel
const int BIN2 = 11;  // Direction control for right wheel
const int PWMB = 12;  // PWM control for right wheel speed
int counter = 0; // keeps count of how many solid lines have been passed 
//set trigger and echo pins
const int leftSensorTriggerPin = 2;
const int leftSensorEchoPin = 3;
const int rightSensorTriggerPin = 4;
const int rightSensorEchoPin = 5;

// Initializing global variables for the 24' checkpoint calculation
unsigned long marker1Time = 0; // Time at first marker 
unsigned long marker2Time = 0; // Time at second marker
float calibratedSpeed = 0; // speed to drive after calibration
bool marker3Detected = false; // third marker detection variable
float distanceAfterMarker3 = 0; // distance after third marker

void setup() {
  // Set motor control pins as outputs
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Initialize sensor pins
  pinMode(leftSensorTriggerPin, OUTPUT);
  pinMode(leftSensorEchoPin, INPUT);
  pinMode(rightSensorTriggerPin, OUTPUT);
  pinMode(rightSensorEchoPin, INPUT);

  Serial.begin(9600);  // Begin serial communication
  delay(500);          // Short delay to ensure everything is initialized properly

  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 39, 37, 35, 33, 31, 29, 27, 25 }, SensorCount);
  qtr.setEmitterPin(23);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn on Arduino's LED to indicate we are in calibration mode

  drive(12, 0);
  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 100; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);  // turn off Arduino's LED to indicate we are through with calibration
  drive(-12,0); // drive straight backward
  for (uint16_t i = 0; i < 100; i++) {
    qtr.calibrate();
  }
  drive(12,0); // drive straight forward
  for (uint16_t i = 0; i < 100; i++) {
    qtr.calibrate();
  }
  drive(25, 0);

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

// The main loop to run our robot and it's functions required for line-tracking,
// obstacle detection, and the various marker stops.
void loop() {

  int currentPosition = linepos(); // robots current position over the line
    // Assuming 0 is the minimum value indicating no line detected
    // if (currentPosition == 0) {
    //   drive(0, 0);          // Stop the robot as it has left the track
    //   return;               // Exit the loop function, stopping any further actions
    //}

// // The desired position is the center of the sensor array.
//     int desiredPosition = 3500;                     // Assuming 7000 is max
//     int error = desiredPosition - currentPosition;  // Calculate the error.
//     float kp = 0.005;                               // Proportional gain.
//     int correction = kp * error;

//     drive(20, correction);  // Drive forward at a base speed of 25, with a correction for direction.

//     delay(100); 

  uint16_t position = qtr.readLineBlack(sensorValues); // the sensors read the line below
  int total=0; // initializing the total sensor summation variable
  for (int i=0; i< 8; i++){
    total=total+=sensorValues[i]; // calculates the summation of sensor values
  }
  int number= 8; // number of sensors
  int average_val= total/number; // average sensor value at a given time
  Serial.print(position);
  
  // If loop to define scenarios for each marker the robot goes over
  if (counter==0 && (average_val > 350)) { // at the first marker
    drive(0, 0); 
    delay(1000); // stops for 1 second
    // drive(20, 0);
    marker1Time = millis(); // defines time at first marker
    counter=1; // acknowledges the first marker has passed
  } else if (counter==1 && (average_val > 350)) { // at the second marker
    marker2Time = millis(); // time at second marker
    counter= 2; // second marker has passed
    drive(0, 0);
    delay(1000); // stops for 1 second
    unsigned long travelTime = marker1Time - marker2Time; // time between marker 1 and 2
    calibratedSpeed = 10.0 / (travelTime); // calculates calibrated speed of robot
    //drive(20, 0); // resumes driving
  }
  if (counter==2 && (average_val > 350)) { // at third marker
    drive(0,0);
    delay(1000); // stops for 1 second
    unsigned long time_elapsed=millis(); // time since passing marker 3
    drive(20,0); // resumes driving
    unsigned long timeaftermarker3 = 24/calibratedSpeed; // distance/speed to calculate time to 24'
    if (time_elapsed==timeaftermarker3){ // stops the car after the calculated 24' drive time
      drive(0,0);
      delay(2000);
      //drive(20, 0);
    }
  }
  // Check for obstacles
  float leftDistance = Distance(0);   // Distance from the left sensor
  float rightDistance = Distance(1);  // Distance from the right sensor

  // Threshold distance to consider an obstacle is present (in cm)
  float obstacleThreshold = 10.0;  // Adjust based on your requirement

  if (leftDistance < obstacleThreshold && rightDistance < obstacleThreshold) {
    // If obstacles are detected on both sides, stop and wait
    drive(0, 0);  // Stop the robot
    delay(2500);  // Wait for 5 seconds
  } 
    // Read the current line position.
// int currentPosition = linepos();
  //   // // Assuming 0 is the minimum value indicating no line detected
 if (average_val <= 75) {
    drive(0, 0);          // Stop the robot as it has left the track
    return;               // Exit the loop function, stopping any further actions
 }
  

    // No obstacles detected or obstacles are only on one side, continue with line following

    // // The desired position is the center of the sensor array.
     int desiredPosition = 3500;                     // Assuming 7000 is max
      int error = desiredPosition - currentPosition;  // Calculate the error.
     float kp = 0.004;                               // Proportional gain.
     int correction = kp * error;

     drive(25, correction);  // Drive forward at a base speed of 25, with a correction for direction.

     delay(100);  // Small delay to make it more controllable
    
    // if (average_val<=50){
    //   drive(0,0);
    //   return;
    // }
    
  
}

void drive(int linspd, int rotspd) {
  // Calculate individual wheel speeds based on linear and rotational speeds
  int rightWheelSpeed = linspd + rotspd;
  int leftWheelSpeed = linspd - rotspd;

  digitalWrite(STBY, HIGH);  // Activate motor driver from standby mode

  // Set wheel directions based on the calculated speeds
  if (rightWheelSpeed > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }

  if (leftWheelSpeed > 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }

  // Apply PWM control to adjust wheel speeds within a 0-255 range
  analogWrite(PWMA, constrain(map(abs(leftWheelSpeed), 0, 100, 0, 255), 0, 255));
  analogWrite(PWMB, constrain(map(abs(rightWheelSpeed), 0, 100, 0, 255), 0, 255));
}


//calculate distance for a given sensor

float Distance(int sensor) {

  int triggerPin, echoPin;

  if (sensor == 0) {  // Left sensor

    triggerPin = leftSensorTriggerPin;

    echoPin = leftSensorEchoPin;

  } else if (sensor == 1) {  // Right sensor

    triggerPin = rightSensorTriggerPin;

    echoPin = rightSensorEchoPin;

  } else {

    return -1;
    // Invalid sensor
  }

  // Send pulse
  digitalWrite(triggerPin, LOW);  //off
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);  //on
  delayMicroseconds(10);           //delay
  digitalWrite(triggerPin, LOW);   //off

  // pulse duration
  float duration = pulseIn(echoPin, HIGH);

  // Calculate distance
  float distance = duration * 0.034 / 2;  // Speed of sound is 34 cm/ms

  return distance;
}

int linepos() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  Serial.println(position);

  delay(250);

  return position;
}
