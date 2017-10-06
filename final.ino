#include <SoftwareSerial.h>
#include <RunningAverage.h>
#include <math.h> //atan2
//Define pins

//Ultrasonic sensor
#define trig 46 //Pin that Triggers the Ultrasonic sensor
#define echo 45 //Pin that receives the echo

//Forward movement
#define RPWM_1  10 // H-bridge leg 2 ->Right Wheel forward
#define enR_1  8 // H-bridge enable pin 2 -> Right Wheel forward enable
#define RPWM_2  5 // H-bridge leg 2 ->Left Wheel forward (its 4 on the Mega but the motor is reversed)
#define enR_2  7 // H-bridge enable pin 2 -> Left Wheel forward enable (its 9 on the Mega but the motor is reversed)

//Backward movement
#define LPWM_1  11 // H-bridge leg 1 -> Right Wheel backwards
#define enL_1  6 // H-bridge enable pin 1 -> Right Wheel backwards enable
#define LPWM_2  4 // H-bridge leg 1 ->Left Wheel backwards (its 5 on the Mega but the motor is reversed)
#define enL_2  9 // H-bridge enable pin 1 -> Left Wheel backwards enable (its 7 on the Mega but the motor is reversed)

//Encoder pins
#define rightEncPinA 18
#define rightEncPinB 19
#define leftEncPinA 2
#define leftEncPinB 3

//Variables

//Distance thresholds
#define distFar 45
#define distClose 25

long duration, distance; //used for distance measuring

//Initialize volatile variables (for Interrupt Service Routines - ISRs)
volatile int rightPos = 0;
volatile int leftPos = 0;
volatile unsigned long int rightSpeedTimer;
volatile unsigned long int leftSpeedTimer;
volatile double rightTempTime;
volatile float rightWheelSpeed;
volatile double leftTempTime;
volatile float leftWheelSpeed;

int prevPos = -1;

RunningAverage errorCorrectionRight(12);
RunningAverage errorCorrectionLeft(12);

//Serup code, run only once
void setup() {

	//ultrasonic sensor
	pinMode(trig, OUTPUT); 
	pinMode(echo, INPUT);
	//pinMode(Buzzer, OUTPUT);
	//pinMode(pinfrontLights , OUTPUT);
	//pinMode(pinbackLights , OUTPUT);
	pinMode(LPWM_1, OUTPUT);
	pinMode(RPWM_1, OUTPUT);
	pinMode(enL_1, OUTPUT);
	pinMode(enR_1, OUTPUT);
	pinMode(LPWM_2, OUTPUT);
	pinMode(RPWM_2, OUTPUT);
	pinMode(enL_2, OUTPUT);
	pinMode(enR_2, OUTPUT);
	pinMode(rightEncPinA,INPUT);
	digitalWrite(rightEncPinA, HIGH);       // turn on pull-up resistor
	pinMode(rightEncPinB, INPUT);
	digitalWrite(rightEncPinB, HIGH);       // turn on pull-up resistor
	pinMode(leftEncPinA,INPUT);
	digitalWrite(leftEncPinA, HIGH);       // turn on pull-up resistor
	pinMode(leftEncPinB, INPUT);
	digitalWrite(leftEncPinB, HIGH);       // turn on pull-up resistor
  
	// Attach Interrupts
	attachInterrupt (digitalPinToInterrupt(rightEncPinA), transitionRightA, CHANGE);
	attachInterrupt (digitalPinToInterrupt(leftEncPinA), transitionLeftA, CHANGE);
	attachInterrupt (digitalPinToInterrupt(rightEncPinB), transitionRightB, CHANGE);
	attachInterrupt (digitalPinToInterrupt(leftEncPinB), transitionLeftB, CHANGE);

	errorCorrectionRight.clear(); //start clean
	errorCorrectionLeft.clear(); //start clean

	Serial.begin(9600);
}

void loop() {
  Serial.print("Distance is:");
  Serial.println(dist());
  delay(1000);
}

//Additional functions

//Meaasure distance
int dist(){
	//Send HIGH pulse for 10us to trigger the sensor 
	digitalWrite(trig, LOW);
	delayMicroseconds(2);
	digitalWrite(trig, HIGH);
	delayMicroseconds(10); //might need 10
	digitalWrite(trig, LOW);
	duration = pulseIn(echo, HIGH); // Wait for HIGH, timer on, timer ends on LOW (counts HIGH duration)
	distance = (duration/2) / 29.1; //distance in cm (might need 58.2)
	return distance;
}

//Function to control movement
void forward() {
	digitalWrite(RPWM_1, velocity); //Right wheel
	digitalWrite(RPWM_2, velocity); //Left wheel 
}

void backwards() {
	digitalWrite(LPWM_1, velocity); //Right wheel
	digitalWrite(LPWM_2, velocity); //Left wheel
	digitalWrite(pinbackLights, HIGH);
	delay(150);
	digitalWrite(pinbackLights, LOW);
}

void turnLeft() {
	digitalWrite(RPWM_1, velocity);
	digitalWrite(RPWM_2, LOW);
}

void turnRight() {
	digitalWrite(RPWM_2, velocity);
	digitalWrite(RPWM_1, LOW);
}

void stopAll() {
	digitalWrite(LPWM_1, LOW);
	digitalWrite(RPWM_1, LOW);
	digitalWrite(LPWM_2, LOW);
	digitalWrite(RPWM_2, LOW);
}

void right360() {
	digitalWrite(RPWM_2, velocity);
	digitalWrite(LPWM_1, velocity);
}

void left360() {
	digitalWrite(RPWM_1, velocity);
	digitalWrite(LPWM_2, velocity);
}

void transitionRightA() {
	if (digitalRead(rightEncPinA)!=digitalRead(rightEncPinB)){
		rightPos = rightPos + 1; //forward direction
		errorCorrectionRight.addValue(1);
	}
	else if (digitalRead(rightEncPinA)==digitalRead(rightEncPinB)) {
		rightPos = rightPos - 1; //reverse direction
		errorCorrectionRight.addValue(-1);
	}
	
	//Velocity calculation
	if (((rightPos%48)==0) || ((rightPos-1)%48==0)){
		rightTempTime = (millis() - rightSpeedTimer);
		rightWheelSpeed = 360000 / rightTempTime;
		rightSpeedTimer = millis();
	}
}

void transitionRightB() {
	if (digitalRead(rightEncPinB)==digitalRead(rightEncPinA)){
		rightPos = rightPos + 1; //forward direction
		errorCorrectionRight.addValue(1);
	}
	else if(digitalRead(rightEncPinB)!=digitalRead(rightEncPinA)) {
		rightPos = rightPos - 1; //reverse direction
		errorCorrectionRight.addValue(-1);
	}

/* 	//Velocity calculation
	if (((leftPos%48)==0) || ((leftPos-1)%48==0)){
		leftTempTime = (millis() - leftSpeedTimer);
		leftWheelSpeed = 360000 / leftTempTime;
		leftSpeedTimer = millis();
	} */
}
void transitionLeftA() {
	if (digitalRead(leftEncPinA)!=digitalRead(leftEncPinB)){
		leftPos = leftPos + 1; //forward direction
		errorCorrectionLeft.addValue(1);
	}
	else if (digitalRead(leftEncPinA)==digitalRead(leftEncPinB)){
		leftPos = leftPos - 1; //reverse direction
		errorCorrectionLeft.addValue(-1);
	}
  
	//Velocity calculation
	if (((leftPos%48)==0) || ((leftPos-1)%48==0)){
		leftTempTime = (millis() - leftSpeedTimer);
		leftWheelSpeed = 360000 / leftTempTime;
		leftSpeedTimer = millis();
	}
}

void transitionLeftB() {
	if (digitalRead(leftEncPinA)==digitalRead(leftEncPinB)){
		leftPos = leftPos + 1; //forward direction
		errorCorrectionLeft.addValue(1);
	}
	else if (digitalRead(leftEncPinA)!=digitalRead(leftEncPinB)){
		leftPos = leftPos - 1; //reverse direction
		errorCorrectionLeft.addValue(-1);
	}
}
