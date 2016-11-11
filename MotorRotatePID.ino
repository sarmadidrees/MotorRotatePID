#include <math.h>

// For encoders
#include <Encoder.h>
Encoder encL(3,5);
Encoder encR(2,4);
long encoderL=0;
long encoderR=0;

// For Motor Control
#include "MotorLib.h"
MotorLib motorL(6, 7, 9);
MotorLib motorR(12, 13, 10);
// Struct to hold motor parameters
struct motorParams {
  double kp;
  double ki;
  double kd;
};
motorParams motorPID;
//motorParams motorPIDR;

// For PID for motors
#include <PID_v1_my.h>
double currentH;
double pwm;
double setpoint;
PID pid(&currentH, &pwm, &setpoint, 1,0,0, DIRECT);
//PID pidR(&spd2, &pwm2, &vel2, 1,0,0, DIRECT);
boolean pidActive= false;

// For incoming Serial String
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

unsigned long previousMillis = 0;
const long interval = 10;

boolean right = false;
boolean left = false;

void setup() {
  Serial.begin(115200);
  inputString.reserve(100);

  encL.write(0);
  encR.write(0);
  motorL.setDir(FORWARD);
  motorR.setDir(FORWARD);

  // initalizing PID
  pid.SetMode(AUTOMATIC);       // PID CONTROL OFF
 // pidR.SetMode(MANUAL);
 // pid.SetTunings(motorPIDL.kp, motorPIDL.ki, motorPIDL.kd);
 
  pid.SetTunings(1.8,0,0);
 
 // pidR.SetTunings(motorPIDR.kp, motorPIDR.ki, motorPIDR.kd);
  pid.SetSampleTime(interval);      // Sample time for PID
 // pidR.SetSampleTime(interval);
  pid.SetOutputLimits(0,255);  // min/max PWM
 // pidR.SetOutputLimits(0,255);

 setpoint = 90;

 //delay(3000);
}

void loop() {

  if (stringComplete) {
    interpretSerialData();
    stringComplete = false;
    inputString = "";
  }

  pid.Compute();
  /*
  Serial.print(currentH);
  Serial.print(",");
  Serial.print(pid.GetError());
  Serial.print(",");
  Serial.println(pwm);
*/

  if(pidActive){
    if(pid.GetError()<0){
        pid.SetControllerDirection(REVERSE);
        if(right){
            motorL.setDir(BACKWARD);
            motorR.setDir(FORWARD);
          }
          else if (left){
            motorL.setDir(FORWARD);
            motorR.setDir(BACKWARD);
          }
    }
    else if (pid.GetError()>=0){
        pid.SetControllerDirection(DIRECT);
        if(right){
            motorL.setDir(FORWARD);
            motorR.setDir(BACKWARD);
          }
          else if (left){
            motorL.setDir(BACKWARD);
            motorR.setDir(FORWARD);
          }
      }
    motorL.setPWM(pwm);
    motorR.setPWM(pwm);
  }
  else {
    motorL.setDir(BRAKE);
    motorR.setDir(BRAKE);
    motorL.setPWM(0);
    motorR.setPWM(0);
  }
}

void interpretSerialData(void){
    int c1=1, c2=1;
    int val1=0, val2=0;

    if(inputString[0]  == 'H'){
      c1 = inputString.indexOf(',')+1;
      c2 = inputString.indexOf(',',c1);
      float headingAngle = inputString.substring(c1,c2).toFloat();
      c1 = c2+1;
      currentH = inputString.substring(c1).toFloat();
    }
    else if (inputString[0]  == 'P'){
      float p,i,d;
        c1 = inputString.indexOf(',')+1;
        c2 = inputString.indexOf(',',c1);
        p = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        c2 = inputString.indexOf(',',c1);
        i = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        c2 = inputString.indexOf(',',c1);
        d = inputString.substring(c1,c2).toFloat();
        c1 = c2+1;
        val1 = inputString.substring(c1).toInt();
        
        if(val1==3) {
          motorPID.kp = p;
          motorPID.ki = i;
          motorPID.kd = d;
          pid.SetTunings(motorPID.kp, motorPID.ki, motorPID.kd);

          pidActive = false;
        }
        
    }
    //For rotation
    else if (inputString[0]  == 'G'){
      if(inputString[2]  == 'R'){
        right = true;
        left = false;
        motorL.setDir(FORWARD);
        motorR.setDir(BACKWARD);
      }else if (inputString[2]  == 'L'){
        right = false;
        left = true;
        motorL.setDir(BACKWARD);
        motorR.setDir(FORWARD);
      }
      pidActive = true;
    }
    
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
