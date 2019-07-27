//Ben Clark

//Egg Transporter Robot Demonstration

#include <Servo.h>
const int PointingGain = 20; //for adjusting the ultrasonic sensor angle

const int SteeringGain = 50; //for adjusting the motor speeds

const int StandardSpeed = 80; //The speed at which the robot will travel when centered

const int Threshold = 520; //Set the threshold to a number that won't be 
//surpassed when there is no black line, but will be surpassed if there is a
//black line
int disabler = 0;
int RightMotorSpeed = StandardSpeed; //Initialize the motor speeds at the
//standard speed
int LeftMotorSpeed = StandardSpeed; 
int G6 = 1567.98; //Establish frequencies for musical notes for the egg
//ready song
int FS6 = 1479.98;
int F6 = 1396.91;
int E6 = 1318.51;
int D6 = 1174.66;
int C6 = 1046.50;
int B5 = 987.77;
int A = 880.00;
int G5 = 783.99;
int motorA1 = 4; //Declare variables for motor pins
int motorA2 = 5;
int motorB1 = 6;
int motorB2 = 7;
int tonePin = 50; //Declare variable for tone pin
const int trigPin = 40; //Declare constant for trigger pin
const int echoPin = 41; //Declare constant for echo pin
int movementLEDpin = 38;
const int closedAngle = 72;
const int openAngle = 15;


Servo Servo1; //Create servo objects
Servo Servo2;
int Servo1Pin = 11;
int Servo2Pin = 10;

void setup() {
  pinMode(motorA1, OUTPUT); //Initialize motor pins as OUTPUTs
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(trigPin, OUTPUT); //Initialize trigger pin as OUTPUT 
  pinMode(echoPin, INPUT); //Initialize echo pin as INPUT
  pinMode(A15, INPUT); //Initialize analog in pins as INPUTs for sensors
  pinMode(A14, INPUT);
  pinMode(A13, INPUT);
  pinMode(A12, INPUT);
  pinMode(A11, INPUT);
  pinMode(43, OUTPUT); //Initialize sensor LED pins as OUTPUTs
  pinMode(45, OUTPUT);
  pinMode(47, OUTPUT);
  pinMode(49, OUTPUT);
  pinMode(51, OUTPUT);
  pinMode(tonePin, OUTPUT);
  pinMode(movementLEDpin, OUTPUT);
  
  Servo1.attach(Servo1Pin); //Set the servo pin as a servo driver
  Servo2.attach(Servo2Pin);
  Servo1.write(90); //Point the sensor 
  Servo2.write(closedAngle);
  Serial.begin(9600); //Start the serial monitor at 9600 bits per seconds
  }

void loop() {
  float distance = ObstacleDistance(25);
  if (GetPathSensorStates() == 31) {
    stopRobot();
    delay(2500);
    Servo2.write(openAngle); //Open the doors and dispense the egg
    eggReadySong();
    Servo2.write(closedAngle);
    delay(10000);
  }
  else {
    if (distance > 7) {
      GetPathSensorStates(); //Establish the sensor code
      movementSiren();
      float Error = SensePathPositionError(GetPathSensorStates()); //Find the 
      //error distance
      if ((Error >= -2.5) and (Error <= 2.5)) { //If the error is valid
        AdjustMotorSpeeds(Error); //Adjust the motor speeds to keep the robot
        //on track
        UpdatePointingAngle(Error); //Keep the sensor pointing forward
        blinkMoveLED();
      }
      else { //Otherwise, keep the robot moving at the standard speed
        goForward(StandardSpeed, StandardSpeed); 
        Servo1.write(90); //Point the sensor forward
        blinkMoveLED();
      }
    }
    else if (distance < 7) {
      stopRobot(); //Avoid the obstacle
      ObstacleWarningSound();
      disabler = 1;
    }
  }
}

byte GetPathSensorStates() { //This function creates a code with the digital
  //values of all five sensors, to show which sensor is above the black line
  byte SensorCode = 0;  
  bitWrite(SensorCode, 4, ReadLineSensor(A15, 43)); //Get the digital values
  //of each sensor using ReadLineSensor function
  bitWrite(SensorCode, 3, ReadLineSensor(A14, 45));
  bitWrite(SensorCode, 2, ReadLineSensor(A13, 47));
  bitWrite(SensorCode, 1, ReadLineSensor(A12, 49));
  bitWrite(SensorCode, 0, ReadLineSensor(A11, 51));
  return SensorCode;
}

int ReadLineSensor(int SensorAnalogInPin, int SensorDigitalOutPin) {
  //This function converts the analog reading to a digital value for a 
  //specified input and output pin
  int analogValue = analogRead(SensorAnalogInPin); //Get the analog reading
  if (analogValue > Threshold) { //If this is over the threshold
    digitalWrite(SensorDigitalOutPin, HIGH); //Set the output pin to HIGH
    return 1;
  }
  else if (analogValue < Threshold) { //If this is below the threshold
    digitalWrite(SensorDigitalOutPin, LOW); //Set the output pin to LOW
    return 0;
  }
}

float SensePathPositionError(byte PathSensorStates) { //Find out how far 
  //off the line the robot is, measured in sensor widths
  float bogus_value = 10.0; //Some bogus value outside the range
  //Each bit in the path sensor states represents a sensor
  //Distances are based on a scale from -2.0 to 2.0 
  switch (PathSensorStates) {
    case 0b00000: return bogus_value; break; 
    case 0b00001: return 1.75; break;
    case 0b00010: return 1.0; break;
    case 0b00011: return 1.5; break;
    case 0b00100: return 0.0; break;
    case 0b00101: return bogus_value; break;
    case 0b00110: return 0.5; break;
    case 0b00111: return bogus_value; break;
    case 0b01000: return -1.0; break;
    case 0b01001: return bogus_value; break;
    case 0b01010: return bogus_value; break;
    case 0b01011: return bogus_value; break;
    case 0b01100: return -0.5; break;
    case 0b01101: return bogus_value; break;
    case 0b01110: return bogus_value; break;
    case 0b01111: return bogus_value; break;
    case 0b10000: return -1.75; break;
    case 0b10001: return bogus_value; break;
    case 0b10010: return bogus_value; break;
    case 0b10011: return bogus_value; break;
    case 0b10100: return bogus_value; break;
    case 0b10101: return bogus_value; break;
    case 0b10110: return bogus_value; break;
    case 0b10111: return bogus_value; break;
    case 0b11000: return -1.5; break;
    case 0b11001: return bogus_value; break;
    case 0b11010: return bogus_value; break;
    case 0b11011: return bogus_value; break; 
    case 0b11100: return bogus_value; break; 
    case 0b11101: return bogus_value; break;
    case 0b11110: return bogus_value; break;
    case 0b11111: return bogus_value; break;
  }
}

int UpdatePointingAngle(float PathError) { //Adjusts the angle of the sensor
  //based on the error 
  int SensorPointingAngle = 90 + (PointingGain * PathError); 
  Servo1.write(SensorPointingAngle); //Change the sensor to the new angle
  return SensorPointingAngle;
}

void AdjustMotorSpeeds(float PathError) { 
  float SpeedAdjustmentPercent = SteeringGain * PathError; //Create a 
  //percentage based on the error 
  RightMotorSpeed = ((100 - SpeedAdjustmentPercent) * (StandardSpeed/100.0));
  //Subtract that percentage from 100% and multiply it by the standard speed
  LeftMotorSpeed = ((100 + SpeedAdjustmentPercent) * (StandardSpeed/100.0));
  //Add that percentage from 100% and multiply it by the standard speed
  goForward(RightMotorSpeed, LeftMotorSpeed); //Move the robot using the 
  //new speeds
}

void goForward(int rightSpeed, int leftSpeed) { //Moves the robot forward at 
  //the specified speed
  digitalWrite(motorA1, LOW); 
  analogWrite(motorA2, rightSpeed);
  digitalWrite(motorB1, LOW);
  analogWrite(motorB2, leftSpeed);
}
void stopRobot() { //Stops all motors
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

void blinkMoveLED() {
  digitalWrite(movementLEDpin, HIGH); 
}

float ObstacleDistance(float WithinInches) {
  digitalWrite(trigPin, LOW); //Make sure the trigPin is clear of 
  //voltage first
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); //Trigger the ultrasonic sensor to send out 
  //the sound wave from the transmitter 
  delayMicroseconds(10); //Send the trigger for 10 microseconds
  digitalWrite(trigPin, LOW); //Turn the trigger pin off 

  unsigned long echoDelay = pulseIn(echoPin, HIGH, 2000); //Measures how 
  //long it takes for the sound wave to travel to the object and back
  float distance = (0.5) * (echoDelay / 74); //Convert the time into a 
  //distance using speed of sound = 74 microseconds per inch, divide by 
  //2 because the sound wave has to travel to the object AND back
  if (disabler == 1) {
    return 80.0;
  }
  else if (disabler == 0) {
    if (distance > WithinInches or distance == 0) { //If there's no object 
      //present
      return 80.0; //If the function returns 80 that means obects are too far
    }
    else if (distance != 0 and distance < WithinInches) { //If there is an 
      //object present
      return distance;
    }
  }
}

void ObstacleWarningSound() {
  tone(tonePin, 1800);
  delay(500);
  tone(tonePin, 1200);
  delay(500);
  tone(tonePin, 1800);
  delay(500);
  tone(tonePin, 1200);
  delay(500);
  tone(tonePin, 1800);
  delay(500);
  tone(tonePin, 1200);
  delay(500);
  tone(tonePin, 1800);
  delay(500);
  tone(tonePin, 1200);
  delay(500);
  noTone(tonePin);
}

void movementSiren() {
    tone(tonePin, 800);
}

void eggReadySong() { //Play a song when the egg is dispensed
  tone(tonePin, D6);
  delay(500);
  tone(tonePin, G5);
  delay(250);
  tone(tonePin, A);
  delay(250);
  tone(tonePin, B5);
  delay(250);
  tone(tonePin, C6);
  delay(250);
  tone(tonePin, D6);
  delay(500);
  tone(tonePin, G5);
  delay(400);
  noTone(tonePin);
  delay(100);
  tone(tonePin, G5);
  delay(400);

  noTone(tonePin);
  delay(100);
  tone(tonePin, E6);
  delay(500);
  tone(tonePin, C6);
  delay(250);
  tone(tonePin, D6);
  delay(250);
  tone(tonePin, E6);
  delay(250);
  tone(tonePin, FS6);
  delay(250);
  tone(tonePin, G6);
  delay(500);
  tone(tonePin, G5);
  delay(400);
  noTone(tonePin);
  delay(100);
  tone(tonePin, G5);
  delay(400);
  noTone(tonePin);
}
