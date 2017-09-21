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
float errorHeadingRef = 10.37;                    // define a variable for the error between actual heading and reference
int localkey = 0;                             // variable for reading the keypad value
int ref = 0;                                   // reference angle
int steerPrecision = 10;

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

  if (ref > 180) {
    ref = (360 - ref) * -1;
  }

  Serial.print("Ref: "); Serial.println(ref);

  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");   // check if the sensor is connected and deteced
    while (1);
  }

  byte c_data[22] = {253, 255, 254, 255, 17, 0, 232, 254, 97, 2, 30, 1, 255, 255, 0, 0, 0, 0, 232, 3, 5, 3};    ///////////////// PASTE YOUR CALIBRATION DATA HERE /////////////
  bno.setCalibData(c_data);                                                                                       // Save calibration data
  delay(1000);
  bno.setExtCrystalUse(true);

  //analogWrite(carSpeedPin, carSpeed);         // set the pwm duty cycle

  FlexiTimer2::set(100, navigate);            // define a timer interrupt with a period of "100*0.001 = 0.1" s or 10 Hz
  FlexiTimer2::start();                       // start the timer interrupt
}



void ReadHeading() {
  // Read the heading using VECTOR_EULER from the IMU
  // that is more stable than any VECTOR_MAGNETOMETER.
  //imu::Vector<3> rawAcc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> rawMag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  //imu::Vector<3> rawGyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

//  Serial.print("Acc: <");
//  Serial.print(rawAcc.x()); Serial.print(", ");
//  Serial.print(rawAcc.y()); Serial.print(", ");
//  Serial.print(rawAcc.z()); Serial.println(">");

//  Serial.print("Mag: <");
//  Serial.print(rawMag.x()); Serial.print(", ");
//  Serial.print(rawMag.y()); Serial.print(", ");
//  Serial.print(rawMag.z()); Serial.print("> -- ");

//  Serial.print("Gyr: <");
//  Serial.print(rawGyr.x()); Serial.print(", ");
//  Serial.print(rawGyr.y()); Serial.print(", ");
//  Serial.print(rawGyr.z()); Serial.println(">");

  HEADING = (atan2(rawMag.y(), rawMag.x()) * 180 / PI); // This output makes North = 90 degrees

  // Shifts output to make 0° north, 90° east, +/-180° south, -90° west
  if (HEADING <= 180 && HEADING >= -90) {
    HEADING -= 90;
  } else {
    HEADING += 270;
  }

  HEADING += errorHeadingRef; // Adds error in HEADING for Tempe

  // Corrects any values which exceed 180
  if (HEADING > 180) HEADING -= 360;

  // FINAL OUTPUT: 0° is north, 90° is east, +/-180° is south, -90° is west

  Serial.print("Angle: "); Serial.print(HEADING); Serial.print(" | ");
}



void CalculateSteering() {
  // Calculate the steering angle according to the referece heading and actual heading

  // ref can be between -150 and 180

  int leftBound, rightBound, backBound;

  if (ref < -90) {
    // Between -180 and -90 (does not include -180 or -90)
    // (-150, -120)
    leftBound = ref + 270;
    rightBound = ref + 90;
    backBound = ref + 180;

    if (HEADING <= rightBound) {
      STEERANGLE = (int)((rightBound - HEADING)/steerPrecision) * steerPrecision;
    } else if (HEADING >= leftBound) {
      STEERANGLE = (int)((rightBound - HEADING + 360)/steerPrecision) * steerPrecision;
    } else if (HEADING <= backBound) {
      STEERANGLE = 0;
    } else {
      STEERANGLE = 180;
    }
  } else if (ref < 0) {
    // Between -90 and 0 (includes -90, but not 0)
    // (-90, -60, -30);
    leftBound = ref - 90;
    rightBound = ref + 90;
    backBound = ref + 180;

    if (HEADING >= leftBound && HEADING <= rightBound) {
      STEERANGLE = (int)((rightBound - HEADING)/steerPrecision) * steerPrecision;
    } else if (HEADING <= backBound && HEADING >= rightBound) {
      STEERANGLE = 0;
    } else {
      STEERANGLE = 180;
    }
  } else if (ref <= 90) {
    // Between 0 and 90 (includes 0 and 90)
    // (120, 150, 180)
    leftBound = ref - 90;
    rightBound = ref + 90;
    backBound = ref - 180;

    if (HEADING >= leftBound && HEADING <= rightBound) {
      STEERANGLE = (int)((rightBound - HEADING)/steerPrecision) * steerPrecision; 
    } else if (HEADING >= backBound && HEADING <= leftBound) {
      STEERANGLE = 180;
    } else {
      STEERANGLE = 0;
    }
  } else if (ref <= 180) {
    // Between 90 and 180 (includes 180, but not 90)
    // (0, 30, 60, 90)
    leftBound = ref - 90;
    rightBound = ref - 270;
    backBound = ref - 180;

    if (HEADING >= leftBound) {
      STEERANGLE = (int)((rightBound - HEADING + 360)/steerPrecision) * steerPrecision;
    } else if (HEADING <= rightBound) {
      STEERANGLE = (int)((rightBound - HEADING)/steerPrecision) * steerPrecision;
    } else if (HEADING > rightBound && HEADING <= backBound) {
      STEERANGLE = 0;
    } else {
      STEERANGLE = 180;
    }
  }

  if (STEERANGLE > 135) STEERANGLE = 135;
  if (STEERANGLE < 45) STEERANGLE = 45;

  Serial.print("Steer Angle: "); Serial.println(STEERANGLE);// Serial.print(" | ");
}

void Actuate() { // Input: Steering angle - Output: nothing
  if (millis() > 30000) { // after 30 seconds (from the startup)
    // Stop the vehicle (set the speed = 0)
    carSpeed = 0;
  } else {
    // set the steering angle
    // set the speed
    if (HEADING < (ref+270) && HEADING > (ref+90)) {
      //carSpeed = 255*0.08;
    } else {
      //carSpeed = 255*0.16;
    }
    //myservo.write(STEERANGLE);
  }
  myservo.write(STEERANGLE);
  //Serial.print("Car Speed: "); Serial.println(carSpeed);
  //analogWrite(carSpeedPin, carSpeed);
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
