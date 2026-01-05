#include <Wire.h>

#define I2C_ADDRESS 0x58

// Enable pins
const int LEFT_L_EN  = 2;
const int LEFT_R_EN  = 3;
const int RIGHT_L_EN = 6;
const int RIGHT_R_EN = 7;

// PWM pins
const int LEFT_R_PWM  = 4;
const int LEFT_L_PWM  = 5;
const int RIGHT_R_PWM = 8;
const int RIGHT_L_PWM = 9;

// Stored values
uint8_t left_r_pwm  = 0;
uint8_t left_l_pwm  = 0;
uint8_t right_r_pwm = 0;
uint8_t right_l_pwm = 0;

bool left_r_en  = false;
bool left_l_en  = false;
bool right_r_en = false;
bool right_l_en = false;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);

  // Set pins
  pinMode(LEFT_R_EN, OUTPUT);
  pinMode(LEFT_L_EN, OUTPUT);
  pinMode(RIGHT_R_EN, OUTPUT);
  pinMode(RIGHT_L_EN, OUTPUT);

  pinMode(LEFT_R_PWM, OUTPUT);
  pinMode(LEFT_L_PWM, OUTPUT);
  pinMode(RIGHT_R_PWM, OUTPUT);
  pinMode(RIGHT_L_PWM, OUTPUT);
}

void loop() {
  // Apply enable signals
  digitalWrite(LEFT_R_EN,  left_r_en  ? HIGH : LOW);
  digitalWrite(LEFT_L_EN,  left_l_en  ? HIGH : LOW);
  digitalWrite(RIGHT_R_EN, right_r_en ? HIGH : LOW);
  digitalWrite(RIGHT_L_EN, right_l_en ? HIGH : LOW);

  // Apply PWM signals
  analogWrite(LEFT_R_PWM,  left_r_pwm);
  analogWrite(LEFT_L_PWM,  left_l_pwm);
  analogWrite(RIGHT_R_PWM, right_r_pwm);
  analogWrite(RIGHT_L_PWM, right_l_pwm);

  delay(10); // 100 Hz update
  Serial.print("LEFT_R_EN: ");
  Serial.println(left_r_en);
  delay(10);
  Serial.print("LEFT_L_EN: ");
  Serial.println(left_l_en);
  //Serial.println(left_r_pwm);
  //Serial.println(left_l_pwm);
}

// Receive I2C data from CODESYS
void receiveEvent(int howMany) {
  while (Wire.available() >= 2) {
    uint8_t reg = Wire.read();
    uint8_t val = Wire.read();

    switch (reg) {
      case 0x00: left_r_en  = val != 0; break;
      case 0x01: left_l_en  = val != 0; break;
      case 0x02: right_r_en = val != 0; break;
      case 0x03: right_l_en = val != 0; break;
      case 0x04: right_r_pwm = val; break;
      case 0x05: right_l_pwm = val; break;
      case 0x06: left_r_pwm  = val; break;
      case 0x07: left_l_pwm  = val; break;
    }
  }
}
