// ================================================================
// ===           	            LIBRARIES	                        ===
// ================================================================

#include <Servo.h>
#include "I2Cdev.h"
#include "Wire.h"
#include <Adafruit_HMC5883_U.h>                   // mag sensor
#include <Adafruit_Sensor.h>                      // part of mag sensor
#include <math.h>
#include <PID_v1.h>
#include <IRremote.h>
#include "HC_SR04.h"


// ================================================================
// ===           	   	  	      PINS	            	            ===
// ================================================================

//Ultrasonic sensor
#define trig 46 //Trigger 
#define echo 2 //Echo
#define echo_int 0

//MotorΒ (right)
#define enB 3
#define in3 5
#define in4 4

//Motor Α (left)
#define enA	 11
#define in1 9
#define in2 8

//Encoder pins
#define rightEncPin 19 //check
#define leftEncPin 18 //check

//Servo pin
#define leftServoPin 22
//IR sensor pin
#define irRcvPin 16
//Buzzer pin
#define buzzerPin 6

// ================================================================
// ===                 		     VARIABLES     	                  ===
// ================================================================

#define DEC_ANGLE 0.081 
#define Kp 4.25
#define Ki 0.005
#define Kd 2.6
#define PIDSampleTime 4
#define midSpeed 125
//#ifdef debug 1

//Initialize volatile variables (for Interrupt Service Routines - ISRs)
volatile int Rcounter=0, Lcounter=0;
int  buzzerTimer = 0, lastRTick=0, lastLTick=0, curMillis, state, last_state, printMillis = 0, headingError, duration, speedA, speedB, moving = 0, movingManual = 0, distToGoal;
byte distToObs;
unsigned long prevMillis, prevMillisH, prevMicrosL, rising_time, falling_time, pulse_length, startTime;
float prevYpr=999,distance, distC = 0, distL = 0,distR = 0, phiG, phiC;
double Setpoint = 0, Input, Output;

//Initialize starting vehicle position and goal in cm
float x = 0,		y = 0,		xG = 100,		yG = 100;

//Initializations
Servo leftServo;
Adafruit_HMC5883_Unified compass =Adafruit_HMC5883_Unified(12345);
sensors_event_t compass_event;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
HC_SR04 us_sensor(trig, echo, echo_int);
IRrecv irrecv(irRcvPin);
decode_results results;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
	Serial.begin(115200);

	if(!compass.begin()){
    Serial.println(F("COMPASS ERROR"));
	}	
	
	//Ultrasonic sensor
	pinMode(trig, OUTPUT);
	pinMode(echo, INPUT);
 //Motor pins
	pinMode(enA, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(trig, LOW);

	pinMode(rightEncPin,INPUT);
  pinMode(leftEncPin, INPUT);
	digitalWrite(rightEncPin, HIGH);       // turn on pull-up resistor
	digitalWrite(leftEncPin, HIGH);       // turn on pull-up resistor
	
	leftServo.attach(leftServoPin);
  leftServo.write(90);
  delay(1000);
  //leftServo.detach();
  
	// Attach Interrupts
	attachInterrupt(digitalPinToInterrupt(leftEncPin),Lcount,RISING);
	attachInterrupt(digitalPinToInterrupt(rightEncPin),Rcount,RISING);
	
	myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-125,125);
  myPID.SetSampleTime(PIDSampleTime);

  us_sensor.begin();
  us_sensor.start();
  
  irrecv.enableIRIn();
  
	//Temporary
  phiG = readCompass() * 180/M_PI; 
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  prevMillis = millis();
  
  if(us_sensor.isFinished()){
    distToObs = us_sensor.getRange();
    us_sensor.start();
  }
  
  phiC = readCompass();

  leftServo.write(90-(phiG - (phiC * 180/M_PI)));
//  Serial.print("phiC: ");
//  Serial.print(phiC * 180/M_PI);
//  Serial.print(" phiG : ");
//  Serial.println(phiG);
  distR = 1.1 * (Rcounter - lastRTick); // 2 * pi * R / 20
  distL = 1.1 * (Lcounter - lastLTick);
  distC = (distR + distL)/2;

  x = x + distC * sin(phiC);
  y = y + distC * cos(phiC);
  
  lastRTick = Rcounter;
  lastLTick = Lcounter;
  
  //distC = 0.55 * (deltaR + deltaL);  //shortest calcualtion 
  //phiG = (atan2((xG - x), (yG - y)) * 180 / M_PI);
  if (phiG <180) phiG = ((int)phiG + 360) % 360; //typechange might be wrong
  
  distToGoal = sqrt(sq(xG - x) + sq(yG - y));
  
  if(irrecv.decode(&results)){
    //Serial.println(results.value);
    if (results.value == 3772778233) {
      forward();
      //Serial.println("front");
      movingManual = 1;
      startTime = millis();
    }
    if(results.value == 3772794553){
      goRight();
      //Serial.println("right");
      startTime = millis();      
      movingManual = 1;   
    }
    if(results.value == 3772819033){
      goLeft();
      startTime = millis();
      //Serial.println("left");      
      movingManual = 1;   
    }
    if(results.value == 3772810873){
      backwards();
      startTime = millis();
      //Serial.println("back");      
      movingManual = 1;   
    }
    if (results.value == 3772782313) {
      stopAll();
      movingManual = 0;
      moving = 0;
    }
    if (results.value == 3772793023) {
      moving = 1;
    }
    irrecv.resume();   
  }


	if (distToObs < 15){
		stopAll();
    moving = 0;
    //Serial.println("Obstacle in front");
	}
	else if (distToObs < 100){
		//turn 45 right or left to avoid obs
    Input = ((100-distToObs)*(phiG - (phiC * 180/M_PI)) + distToObs * ((phiC * 180/M_PI) - phiG  +- 90) / 100.0
    myPID.Compute();    
	}	
	else{
		//no obs ahead, go to goal
		Input = phiG - (phiC * 180/M_PI);
    myPID.Compute();
	}
 
  if (moving == 1 && distToGoal > 20){
    speedB = midSpeed + (int)Output;
    speedA = midSpeed - (int)Output;
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enA, adjustSpeed(speedA));
    analogWrite(enB, adjustSpeed(speedB));
    printStuff();
  } else{
    moving = 0;    
  }
  
  if (movingManual == 1){
    if (millis() - startTime < 100){
      Serial.println("Moving manually...");
    }else{
      stopAll();
      movingManual = 0;
    }
	}
 //Serial.println(millis() - prevMillis);
  Serial.println(distToObs);
}

// ================================================================
// ===            		        FUNCTIONS        		              ===
// ================================================================

void forward(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, midSpeed);
  analogWrite(enB, midSpeed);
}

void backwards(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, midSpeed);
  analogWrite(enB, midSpeed);
}
void goRight(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, midSpeed);
  analogWrite(enB, 0);
  
}
void goLeft(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, midSpeed);
}

void stopAll(){
	digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void Rcount(){
	Rcounter++;
}
void Lcount(){
	Lcounter++;
}


int adjustSpeed(int speeed){
  if (speeed < 0){
    return 0;
  } else if (speeed > 255){
    return 255;
  } else { 
    return speeed;
  } 
}

//int readCompass(void)
float readCompass(void)
{
  compass.getEvent(&compass_event);    
  float heading = atan2(compass_event.magnetic.y, compass_event.magnetic.x); //+ DEC_ANGLE;
	if(heading < 0) heading += 2*M_PI;
	if(heading > 2*M_PI) heading -= 2*M_PI;
	//float headingDegrees = heading * 180/M_PI;
  float headingDegrees = heading;
	//return ((int)headingDegrees);
  return headingDegrees;
}

void printStuff(){
	printMillis = millis();
	Serial.print("Phi: ");
  Serial.print(phiC* 180/M_PI);
	Serial.print(" -> ");
  Serial.print(phiG);
  Serial.print(" x: ");
  Serial.print(x);
  Serial.print(" -> ");
  Serial.print(xG);
  Serial.print(" y: ");
  Serial.print(y);
  Serial.print(" -> ");
  Serial.print(yG);
  Serial.print(" dist: ");
  Serial.print(distToGoal);
	//Serial.print(" Dist from goal: ");
	//Serial.print(distToGoal);
	//Serial.print("Input: ");
	//Serial.print(Input);
	Serial.print(" Speed A: ");
	Serial.print(adjustSpeed(speedA));
	Serial.print(" B: ");
	Serial.println(adjustSpeed(speedB));
//	Serial.print(" PID out: ");
//	Serial.println(Output);
/*	Serial.print("Right:");
	Serial.print(Rcounter); 
	Serial.print("  ");
	Serial.print("Left:"); 
	Serial.println(Lcounter); */
}

//  if (Rcounter != lastRcounter) {
//    lastRcounter = Rcounter;
//    Rvalue_tmp++;
//  if (Rvalue_tmp == (Rvalue +2)) {
//      Rvalue++;
//      Rvalue_tmp--;
//    }
//  }
//  
//  if (Lcounter != lastLcounter) {
//    lastLcounter = Lcounter;
//    Lvalue_tmp++;
//    if (Lvalue_tmp == (Lvalue +2)) {
//      Lvalue++;
//      Lvalue_tmp--;
//    }
//  }
