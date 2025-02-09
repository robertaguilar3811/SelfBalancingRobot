#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <SimpleKalmanFilter.h>
#include <PID_v1.h>

BNO080 myIMU;

#define DIRECT 0
#define REVERSE 1

double Setpoint = -4.25;
double delta = 0.25;
double lowSetpoint = Setpoint - delta;
double highSetpoint = Setpoint + delta;

double pitch = 0.0;
double pitchFiltered = 0.0;

double Output = 0.0;

double Kp = 9;
double Ki = 0;
double Kd = 0;

PID myPID(&pitchFiltered, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int LEFT_RPWM = 3;
int LEFT_LPWM = 4;
int RIGHT_RPWM = 7;
int RIGHT_LPWM = 8;

int LEFT_L_EN = 1;
int LEFT_R_EN = 2;
int RIGHT_L_EN = 5;
int RIGHT_R_EN = 6;

SimpleKalmanFilter simpleKalmanFilter(0.1, 0.1, 0.001);

const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

void setup() {

  Serial.begin(115200);
  Wire.begin();

  pinMode(LEFT_RPWM, OUTPUT);
  pinMode(LEFT_LPWM, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(LEFT_L_EN, OUTPUT);
  pinMode(LEFT_R_EN, OUTPUT);
  pinMode(RIGHT_L_EN, OUTPUT);
  pinMode(RIGHT_R_EN, OUTPUT);

  digitalWrite(LEFT_R_EN, HIGH);  
  digitalWrite(LEFT_L_EN, HIGH);
  digitalWrite(RIGHT_R_EN, HIGH);  
  digitalWrite(RIGHT_L_EN, HIGH);

  if (myIMU.begin() == false) {
  Serial.println(F("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing..."));
  while (1);
  }
  Wire.setClock(400000); //Increase I2C data rate to 400kHz
  myIMU.enableRotationVector(50); //Send data update every 50ms

  myPID.SetMode(AUTOMATIC); 
  }

void calculateAngle() {
    if (myIMU.dataAvailable() == true) {
        pitch = (myIMU.getPitch()) * 180.0 / PI;     // Update global pitch variable
    }
}

void filterAngle(){
  //pitchFiltered = simpleKalmanFilter.updateEstimate(pitch);
  pitchFiltered = pitch;
}

void moveForward(int speed) {
  analogWrite(RIGHT_RPWM, speed);  // Right motor forward
  analogWrite(RIGHT_LPWM, 0);      // Right motor off
  
  analogWrite(LEFT_LPWM, speed);   // Left motor forward
  analogWrite(LEFT_RPWM, 0);       // Left motor off

  //Serial.println("\t");
  //Serial.print("Direction: Forward");
  //Serial.print(", ");
  //Serial.print("Output: ");
  //Serial.print(speed);
}

void moveBackward(int speed) {
  analogWrite(RIGHT_RPWM, 0);      // Right motor off
  analogWrite(RIGHT_LPWM, speed);  // Right motor reverse
  
  analogWrite(LEFT_RPWM, speed);       // Left motor off
  analogWrite(LEFT_LPWM, 0);   // Left motor reverse

  //Serial.println("\t");
  //Serial.print("Direction: Reverse");
  //Serial.print(", ");
  //Serial.print("Output: ");
  //Serial.print(speed);
}

void stopMoving() {
  analogWrite(RIGHT_RPWM, 0);
  analogWrite(RIGHT_LPWM, 0);
  analogWrite(LEFT_RPWM, 0);
  analogWrite(LEFT_LPWM, 0);

  //Serial.println("\t");
  //Serial.print("Direction: None");
  //Serial.print(", ");
  //Serial.print("Output: 0");
  
}

void loop() {

  calculateAngle();
  filterAngle();

  Output = constrain(Output, 30, 255);

  if (pitchFiltered < lowSetpoint && pitchFiltered > -45){
    myPID.SetControllerDirection(DIRECT);
    myPID.Compute();
    moveBackward(Output);
  }
  else if (pitchFiltered > highSetpoint && pitchFiltered < 45){
    myPID.SetControllerDirection(REVERSE);
    myPID.Compute();
    moveForward(Output);
  }
  else{
    stopMoving();
  }

  //Serial.print(", ");
  //Serial.print("Angle:");
  //Serial.print(pitch);
  //Serial.print(",");
  //Serial.print("Filtered Angle:");
  //Serial.println(pitchFiltered);

}