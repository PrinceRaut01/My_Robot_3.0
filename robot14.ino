#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)
#define m1 9
#define m2 8
#define m3 7
#define m4 6
#define m5 5
#define m6 4
#define m7 3
#define m8 2

uint8_t servonum = 0;

#include "DFRobot_DF2301Q.h"

#if (defined(ARDUINO_AVR_UNO) || defined(ESP8266))   // Use software serial
  SoftwareSerial softSerial(/*rx =*/4, /*tx =*/5);
  DFRobot_DF2301Q_UART DF2301Q(/*softSerial =*/&softSerial);
#elif defined(ESP32)   // Use the hardware serial with remappable pin: Serial1
  DFRobot_DF2301Q_UART DF2301Q(/*hardSerial =*/&Serial1, /*rx =*/D3, /*tx =*/D2);
#else   // Use hardware serial: Serial1
  DFRobot_DF2301Q_UART DF2301Q(/*hardSerial =*/&Serial1);
#endif

void setup()
{
  Serial.begin(115200);
  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4, OUTPUT);
  pinMode(m5, OUTPUT);
  pinMode(m6, OUTPUT);
  pinMode(m7, OUTPUT);
  pinMode(m8, OUTPUT);
  pwm.setPWM(4, 0, 100 );//RIGHT SOLDER
  pwm.setPWM(5, 0, 600 );//LEFT SOLDER
  pwm.setPWM(7, 0, 200 );//LEFT ARM
  pwm.setPWM(6, 0, 570 );//RIGHT ARM
  pwm.setPWM(3, 0, 600 );//left eye less
  pwm.setPWM(2, 0, 600 );//RIGHT eye less
  delay(500);
  pwm.setPWM(3, 0, 125 );//left eye less
  pwm.setPWM(2, 0, 125 );//RIGHT eye less
  delay(500);
  pwm.setPWM(3, 0, 600 );//left eye less
  pwm.setPWM(2, 0, 600 );//RIGHT eye less
  pwm.setPWM(1, 0, 400 );//left eye ball 400 is for Straight line
  pwm.setPWM(0, 0, 300 );//RIGHT eye ball 300 is for Straight line
  pwm.setPWM(8, 0, 280 );//mouth for open
  delay(500);
  pwm.setPWM(8, 0, 350 );//mouth for close

  // Init the sensor
  while( !( DF2301Q.begin() ) ) {
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  Serial.println("Begin ok!");

  // DF2301Q.resetModule();
  // DF2301Q.settingCMD(DF2301Q_UART_MSG_CMD_SET_MUTE, 0);
  // DF2301Q.settingCMD(DF2301Q_UART_MSG_CMD_SET_VOLUME, 5);
  // DF2301Q.settingCMD(DF2301Q_UART_MSG_CMD_SET_WAKE_TIME, 20);
  // DF2301Q.settingCMD(DF2301Q_UART_MSG_CMD_SET_ENTERWAKEUP, 0);
}

void loop()
{
  digitalWrite(m1, HIGH);
  digitalWrite(m2, HIGH);
  digitalWrite(m3, HIGH);
  digitalWrite(m4, HIGH);
  digitalWrite(m5, HIGH);
  digitalWrite(m6, HIGH);
  digitalWrite(m7, HIGH);
  digitalWrite(m8, HIGH);
delay(100);
  uint8_t CMDID = 0;
  CMDID = DF2301Q.getCMDID();
  
  if(0 != CMDID) {
    Serial.print("CMDID = ");
    Serial.println(CMDID);
    
    if (CMDID == 5) {
    Serial.println("Command 1 recognized: Performing Action 1");
    digitalWrite(m8, LOW);  
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    pwm.setPWM(6, 0, 240 );//RIGHT ARM
	  delay(1000);
    pwm.setPWM(7, 0, 200 );//LEFT ARM
    pwm.setPWM(6, 0, 570 );//RIGHT ARM
	  delay(1000);
    pwm.setPWM(3, 0, 600 );//left eye less
    pwm.setPWM(2, 0, 600 );//RIGHT eye less
	  delay(500);
    pwm.setPWM(3, 0, 125 );//left eye less
    pwm.setPWM(2, 0, 125 );//RIGHT EYE LESS
	  delay(500);
    pwm.setPWM(3, 0, 600 );//left eye less
    pwm.setPWM(2, 0, 600 );//RIGHT eye less

   for (int i = 0; i < 152; i++) {
    pwm.setPWM(4, 0, 220 );//RIGHT SOLDER
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(6, 0, 350 );//right ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(6, 0, 250 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(6, 0, 350 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(6, 0, 250 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(6, 0, 350 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(6, 0, 250 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(6, 0, 350 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(6, 0, 250 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(6, 0, 350 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(6, 0, 250 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(6, 0, 350 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(6, 0, 250 );//LEFT ARM
    delay(500);
}

      // Add action for CMDID 5
    } else if (CMDID == 6) {
      Serial.println("Command 2 recognized: Performing Action 2");
      digitalWrite(m7, LOW);
      pwm.setPWM(7, 0, 550 );//LEFT ARM
    pwm.setPWM(6, 0, 240 );//RIGHT ARM
	  delay(1000);
    pwm.setPWM(7, 0, 200 );//LEFT ARM
    pwm.setPWM(6, 0, 550 );//RIGHT ARM
	  delay(1000);
    pwm.setPWM(3, 0, 600 );//left eye less
    pwm.setPWM(2, 0, 600 );//RIGHT eye less
	  delay(500);
    pwm.setPWM(3, 0, 125 );//left eye less
    pwm.setPWM(2, 0, 125 );//RIGHT EYE LESS
	  delay(500);
    pwm.setPWM(3, 0, 600 );//left eye less
    pwm.setPWM(2, 0, 600 );//RIGHT eye less
    for (int i = 0; i < 118; i++) {
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
}       
      // Add action for CMDID 6
    } else if (CMDID == 7) {
      Serial.println("Command 2 recognized: Performing Action 2");
      digitalWrite(m6, LOW);
            pwm.setPWM(7, 0, 550 );//LEFT ARM
    pwm.setPWM(6, 0, 240 );//RIGHT ARM
	  delay(1000);
    pwm.setPWM(7, 0, 200 );//LEFT ARM
    pwm.setPWM(6, 0, 550 );//RIGHT ARM
	  delay(1000);
    pwm.setPWM(3, 0, 600 );//left eye less
    pwm.setPWM(2, 0, 600 );//RIGHT eye less
	  delay(500);
    pwm.setPWM(3, 0, 125 );//left eye less
    pwm.setPWM(2, 0, 125 );//RIGHT EYE LESS
	  delay(500);
    pwm.setPWM(3, 0, 600 );//left eye less
    pwm.setPWM(2, 0, 600 );//RIGHT eye less
    for (int i = 0; i < 18; i++) {
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
}       
      // Add action for CMDID 7

    } else if (CMDID == 8) {
      Serial.println("Command 3 recognized: Performing Action 3");
      digitalWrite(m5, LOW);
            pwm.setPWM(7, 0, 550 );//LEFT ARM
    pwm.setPWM(6, 0, 240 );//RIGHT ARM
	  delay(1000);
    pwm.setPWM(7, 0, 200 );//LEFT ARM
    pwm.setPWM(6, 0, 550 );//RIGHT ARM
	  delay(1000);
    pwm.setPWM(3, 0, 600 );//left eye less
    pwm.setPWM(2, 0, 600 );//RIGHT eye less
	  delay(500);
    pwm.setPWM(3, 0, 125 );//left eye less
    pwm.setPWM(2, 0, 125 );//RIGHT EYE LESS
	  delay(500);
    pwm.setPWM(3, 0, 600 );//left eye less
    pwm.setPWM(2, 0, 600 );//RIGHT eye less
    for (int i = 0; i < 20; i++) {
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
}       
      // Add action for CMDID 8
    } else if (CMDID == 9) {
      Serial.println("Command 2 recognized: Performing Action 2");
      digitalWrite(m4, LOW);
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    pwm.setPWM(6, 0, 240 );//RIGHT ARM
	  delay(1000);
    pwm.setPWM(7, 0, 200 );//LEFT ARM
    pwm.setPWM(6, 0, 550 );//RIGHT ARM
	  delay(1000);
    pwm.setPWM(3, 0, 600 );//left eye less
    pwm.setPWM(2, 0, 600 );//RIGHT eye less
	  delay(500);
    pwm.setPWM(3, 0, 125 );//left eye less
    pwm.setPWM(2, 0, 125 );//RIGHT EYE LESS
	  delay(500);
    pwm.setPWM(3, 0, 600 );//left eye less
    pwm.setPWM(2, 0, 600 );//RIGHT eye less
    for (int i = 0; i < 20; i++) {
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
}       
      // Add action for CMDID 9
    } else if (CMDID == 10) {
      Serial.println("Command 2 recognized: Performing Action 2");
      digitalWrite(m3, LOW);
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    pwm.setPWM(6, 0, 240 );//RIGHT ARM
	  delay(1000);
    pwm.setPWM(7, 0, 200 );//LEFT ARM
    pwm.setPWM(6, 0, 550 );//RIGHT ARM
	  delay(1000);
    pwm.setPWM(3, 0, 600 );//left eye less
    pwm.setPWM(2, 0, 600 );//RIGHT eye less
	  delay(500);
    pwm.setPWM(3, 0, 125 );//left eye less
    pwm.setPWM(2, 0, 125 );//RIGHT EYE LESS
	  delay(500);
    pwm.setPWM(3, 0, 600 );//left eye less
    pwm.setPWM(2, 0, 600 );//RIGHT eye less
    for (int i = 0; i < 16; i++) {
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
}       
      // Add action for CMDID 10
    } else if (CMDID == 11) {
      Serial.println("Command 2 recognized: Performing Action 2");
      digitalWrite(m2, LOW);
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    pwm.setPWM(6, 0, 240 );//RIGHT ARM
	  delay(1000);
    pwm.setPWM(7, 0, 200 );//LEFT ARM
    pwm.setPWM(6, 0, 550 );//RIGHT ARM
	  delay(1000);
    pwm.setPWM(3, 0, 600 );//left eye less
    pwm.setPWM(2, 0, 600 );//RIGHT eye less
	  delay(500);
    pwm.setPWM(3, 0, 125 );//left eye less
    pwm.setPWM(2, 0, 125 );//RIGHT EYE LESS
	  delay(500);
    pwm.setPWM(3, 0, 600 );//left eye less
    pwm.setPWM(2, 0, 600 );//RIGHT eye less
    for (int i = 0; i < 16; i++) {
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
}       
      // Add action for CMDID 11
    } else if (CMDID == 12) {
      Serial.println("Command 2 recognized: Performing Action 2");
      digitalWrite(m1, LOW);
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    pwm.setPWM(6, 0, 240 );//RIGHT ARM
	  delay(1000);
    pwm.setPWM(7, 0, 200 );//LEFT ARM
    pwm.setPWM(6, 0, 550 );//RIGHT ARM
	  delay(1000);
    pwm.setPWM(3, 0, 600 );//left eye less
    pwm.setPWM(2, 0, 600 );//RIGHT eye less
	  delay(500);
    pwm.setPWM(3, 0, 125 );//left eye less
    pwm.setPWM(2, 0, 125 );//RIGHT EYE LESS
	  delay(500);
    pwm.setPWM(3, 0, 600 );//left eye less
    pwm.setPWM(2, 0, 600 );//RIGHT eye less
    for (int i = 0; i < 20; i++) {
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth for open
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 320 );//mouth
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 280 );//mouth 
    pwm.setPWM(7, 0, 550 );//LEFT ARM
    delay(500);
    pwm.setPWM(8, 0, 350 );//mouth for close
    pwm.setPWM(7, 0, 450 );//LEFT ARM
    delay(500);
}       
      // Add action for CMDID 12

    } else {
      Serial.println("Unknown command received");
    }
  }
  delay(2000);
}
