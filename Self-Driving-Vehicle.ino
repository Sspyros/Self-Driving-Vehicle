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
#include "RunningMedian.h"

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
#define PIDSampleTime 10
#define midSpeed 100
#define distStop 30
#define distAvoid 120
#define distToGoalThresh 25
#define debug 1

//Initialize volatile variables (for Interrupt Service Routines - ISRs)
volatile int Rcounter=0, Lcounter=0;
int  buzzerTimer = 0, lastRTick=0, lastLTick=0, curMillis, state, last_state, printMillis = 0, headingError, duration, speedA, speedB, moving = 0, movingManual = 0, distToGoal;
byte distToObsRaw, distToObs, avoidFlag = 0, turnRight;
unsigned long prevMillis, prevMillisH, prevMicrosL, rising_time, falling_time, pulse_length, startTime, turnMillis;
float prevYpr=999,distance, distC = 0, distL = 0,distR = 0, phiG, phiC, phiCinR;
double Setpoint = 0, Input, Output;

//Initialize starting vehicle position and goal in cm
float x = 0,		y = 0,		xG = -350,		yG = -150;

//Initializations
Servo leftServo;
Adafruit_HMC5883_Unified compass =Adafruit_HMC5883_Unified(12345);
sensors_event_t compass_event;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
HC_SR04 us_sensor(trig, echo, echo_int);
IRrecv irrecv(irRcvPin);
decode_results results;
RunningMedian usReadings = RunningMedian(15);
RunningMedian InputMedian = RunningMedian(15);      

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

 //LED 
 pinMode(13, OUTPUT);
 
 //Motor pins
	pinMode(enA, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);

  digitalWrite(trig, LOW);

	pinMode(rightEncPin,INPUT);
  pinMode(leftEncPin, INPUT);
	digitalWrite(rightEncPin, HIGH);       // turn on pull-up resistor
	digitalWrite(leftEncPin, HIGH);       // turn on pull-up resistor

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
	leftServo.attach(leftServoPin);
  leftServo.write(90);
  delay(1000);
  leftServo.detach();
  
	// Attach Interrupts
	attachInterrupt(digitalPinToInterrupt(leftEncPin),Lcount,RISING);
	attachInterrupt(digitalPinToInterrupt(rightEncPin),Rcount,RISING);
	
	myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-midSpeed, midSpeed);
  myPID.SetSampleTime(PIDSampleTime);

  us_sensor.begin();
  us_sensor.start();
  
  irrecv.enableIRIn();
  
	//Temporary
  //phiG = readCompass() * 180/M_PI; 
  Serial.println("===================== READY =====================");
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  prevMillis = millis();
  
  if(us_sensor.isFinished()){
    distToObsRaw = us_sensor.getRange();
    //Simple running average
//    distToObs = 0.9 * distToObsRaw + 0.1 * distToObs;
//    Serial.print("Raw: ");
//    Serial.print(distToObsRaw);
//    Serial.print(" Avg: ");
//    Serial.println(distToObs);
    usReadings.add(distToObsRaw);
//    Serial.println(usReadings.getMedian());
    distToObs = usReadings.getMedian();
    us_sensor.start();
  }
  
  phiCinR = readCompass();
  phiC = phiCinR * 180/M_PI;

  //Serial.println(subAngles(90, subAngles(phiG,phiC)));
  leftServo.write(subAngles(90, subAngles(phiG,phiC)));
  
  distR = 1.1 * (Rcounter - lastRTick); // 2 * pi * R / 20
  distL = 1.1 * (Lcounter - lastLTick);
  distC = (distR + distL)/2;

  x = x + distC * sin(phiCinR);
  y = y + distC * cos(phiCinR);
  
  lastRTick = Rcounter;
  lastLTick = Lcounter;
  
  //distC = 0.55 * (deltaR + deltaL);  //shortest calcualtion 
  phiG = (atan2((xG - x), (yG - y)) * 180 / M_PI);
//  if (phiG <180) phiG = (phiG + 360) % 360; //typechange might be wrong
  if ( phiG < 0 ) phiG = phiG + 360;
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
      leftServo.detach();
    }
    if (results.value == 3772793023) {
      moving = 1;
      leftServo.attach(leftServoPin);
    }
    irrecv.resume();   
  }

  if ( distToGoal < distToGoalThresh && moving == 1 ){
    moving = 0;
    stopAll();
    Serial.println("===================== ARRIVED =====================");
  }
  
	if (distToObs < distStop){
		stopAll();
    moving = 0;
    leftServo.detach();
    digitalWrite(13, LOW);

    if ( moving == 1){
      Serial.println("===============Obstacle in front===========");
    }
	} else if (distToObs < distAvoid && distToObs > distStop){
    
		//turn 90deg right or left to avoid obs ( phiC - phiG + 90 )
    InputMedian.add((distToObs*subAngles(phiG, phiC) + (distAvoid - distToObs)*subAngles(phiC, phiG + 90) )/distAvoid);
    //Input = InputMedian.getAverage();
    
    if ( avoidFlag == 0 && y - yG < 0 ){
      turnRight = 1;
      avoidFlag = 1;
      turnMillis = millis();
    } else if (avoidFlag == 0 && y - yG > 0){
      turnRight = 0;
      avoidFlag = 1;
      turnMillis = millis();
    }
    
    if ( !turnRight ) Input = (distToObs*subAngles(phiG, phiC) + (100 - distToObs)*subAngles(phiC, phiG + 90) )/100; //left
    if ( turnRight ) Input = (distToObs*subAngles(phiG, phiC) + (100 - distToObs)*subAngles(phiC, subAngles(phiG, 90)) )/100; //right
    
    //Input = phiG - (phiC * 180/M_PI);
    myPID.Compute();
    digitalWrite(13, HIGH);

    if ( moving == 1){
      if ( debug ) Serial.print("--");
      speedB = midSpeed + (int)Output;
      speedA = midSpeed - (int)Output;
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enA, adjustSpeed(speedA));
      analogWrite(enB, adjustSpeed(speedB));
      if ( debug ) printStuff();
   }  
	}	else {
		//no obs ahead, go to goal
		//Input = phiG - (phiC * 180/M_PI);
    InputMedian.add(subAngles (phiG, phiC));
    Input = InputMedian.getAverage();
    digitalWrite(13, LOW);

    myPID.Compute();
    
    if ( moving == 1){
      if ( debug ) Serial.print("++");
      speedB = midSpeed + (int)Output;
      speedA = midSpeed - (int)Output;
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enA, adjustSpeed(speedA));
      analogWrite(enB, adjustSpeed(speedB));
      if ( debug ) printStuff();
   }
   if ( millis() - turnMillis > 1000) avoidFlag = 0;
	}
 
  if (movingManual == 1){
    if (millis() - startTime < 100){
      //Serial.println("Moving manually...");
      //printStuff();
      Serial.print(Rcounter);
      Serial.print(" L: ");
      Serial.println(Lcounter);
    }else{
      stopAll();
      movingManual = 0;
    }
	}
 //Serial.println(millis() - prevMillis);

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
  analogWrite(enA, 0);
  analogWrite(enB, 0);
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

//difference between 2 angles in degrees
float subAngles(float a, float b){
  if ( a > 180 ) a -= 360;
  if ( b > 180 ) b -= 360;
  int ans = a - b;
  if ( ans > 180 ) ans -= 360;
  if ( ans < -180 ) ans += 360;
  return ans;
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
  Serial.print(phiC);
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
  Serial.print(" distG: ");
  Serial.print(distToGoal);
  Serial.print(" distO: ");
  Serial.print(distToObs);
  if ( turnRight ){
    Serial.print(" Go Right ");
  } else {
    Serial.print(" Go Left ");
  }
	Serial.print(" Input: ");
	Serial.print(Input);
  Serial.print(" Out: ");
  Serial.print(Output);
	Serial.print(" Speed A: ");
	Serial.print(adjustSpeed(speedA));
	Serial.print(" B: ");
	Serial.println(adjustSpeed(speedB));
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
