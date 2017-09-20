#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include "DFR_Key.h"

DFR_Key keypad;         // define keypad object
Servo myservo;                                // define a Servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);    // define a BNO055 object


LiquidCrystal lcd( 8, 9, 4, 5, 6, 7);         // config the lcd using these pins (don't change the numbers)

//////////// Global variables that may change across the functions /////////////
int STEERANGLE = 90;                          // servo initial angle (range is 0:180 and 90 is middle)
float HEADING = 0;                            // actual heading varaiable
int carSpeedPin = 2;                          // define a pin for DC motor (it should support PWM)
int carSpeed = 20;                            // define a variable for DC motor speed
float errorHeadingRef = 0;                    // define a variable for the error between actual heading and reference
int localkey = 0;                             // variable for reading the keypad value
int ref = 0;                                   // reference angle

void setup() {
  myservo.attach(44);                         // set which pin the servo is connected to (here pin 44)
  lcd.begin( 16, 2 );                         // LCD type (col , row) (it's 16 x 2)
  Serial.begin(9600);                         // setup the serial for monitoring (baudrate = 9600)
  Serial.println("Orientation Sensor Calibration"); Serial.println("");

  ///reading the reference///
  localkey = 0;
  while (localkey != 1) {
    lcd.clear();
    localkey = keypad.getKey();
    if (localkey == 3)
      ref = ref + 30;
    if (localkey == 4)
      ref = ref - 30;
    if (ref < 0)
      ref = 0;
    if (ref > 360)
      ref = 360;
    lcd.print(ref);
    delay(100);
  }

  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");   // check if the sensor is connected and deteced
    while (1);
  }

  byte c_data[22] = {236, 255, 220, 255, 3, 0, 0, 6, 162, 4, 16, 6, 255, 255, 255, 255, 0, 0, 232, 3, 125, 2};    ///////////////// PASTE YOUR CALIBRATION DATA HERE /////////////
  bno.setCalibData(c_data);                                                                                       // Save calibration data
  delay(1000);
  bno.setExtCrystalUse(true);

  analogWrite(carSpeedPin, carSpeed);         // set the pwm duty cycle

  FlexiTimer2::set(100, navigate);            // define a timer interrupt with a period of "100*0.001 = 0.1" s or 10 Hz
  FlexiTimer2::start();                       // start the timer interrupt
}



void ReadHeading() { // Input: Nothing - // Output: HEADING
  // Read the heading using VECTOR_EULER from the IMU
  // that is more stable than any VECTOR_MAGNETOMETER.
}



void CalculateSteering() { // Input: HEADING & Reference Heading - // Output: Steering Angle
  // Calculate the steering angle according to the referece heading and actual heading
}

void Actuate() { // Input: Steering angle - Output: nothing
  if (millis() > 30000) { // after 30 seconds (from the startup)
    // Stop the vehicle (set the speed = 0)
  } else {
    // set the steering angle
    // set the speed
  }
}

void navigate() {         // This function will be called every 0.1 seconds (10Hz)
  sei();                  // set the interrupt flag ** THIS IS VERY IMPORTANT **. You need to set the interrupt flag or the programm will get stuck here!!!
  ReadHeading();          // read the heading angle
  CalculateSteering();    // calculate the desired steering angle
  Actuate();          // actuate (set the steering angle)
}

void printHeadingOnLCD() {
  // print the heading data on serial monitor to verify the actual heading
}

void printSteerAngleOnLCD() {
  // print the steering angle to verify your control command
}

void loop()
{
  lcd.clear();            // clear the LCD
  printHeadingOnLCD();
  printSteerAngleOnLCD();
  delay(100);
}
