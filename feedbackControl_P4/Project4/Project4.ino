#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include "DFR_Key.h"

DFR_Key keypad;                               // Define keypad object. This will be used be a TA to input the desired direction.
Servo myservo;                                // Define a Servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);    // Define a BNO055 object


LiquidCrystal lcd( 8, 9, 4, 5, 6, 7);         // Config the lcd using these pins (don't change the numbers).

//////////// Global variables that may change across the functions /////////////
int STEERANGLE = 90;                          // Servo initial angle (range is 0:180 and 90 is middle)
float HEADING = 0;                            // Actual heading varaiable
int carSpeedPin = 2;                          // Define a pin for DC motor (it should support PWM)
int carSpeed = 20;                            // Define a variable for DC motor speed
float errorHeadingRef = 10.37;                // Define a variable for the error between actual heading and reference. For Tempe = 10.37 degrees.
int localkey = 0;                             // Variable for reading the keypad value
int ref = 0;                                  // Reference angle, set via keypad.
int steerPrecision = 10;                      // Variable indicates the step size (in degrees) between steering angles.

void setup() {
  myservo.attach(44);                         // set which pin the servo is connected to (here pin 44).
  lcd.begin( 16, 2 );                         // LCD type (col , row) (it's 16 x 2).
  Serial.begin(9600);                         // Setup the serial for monitoring (baudrate = 9600)
  Serial.println("Orientation Sensor Calibration"); Serial.println("");
  
  while (localkey != 1) {                     // While loop will read the reference angle inputted via the keypad.
    lcd.clear();
    localkey = keypad.getKey();               // Read in the value from the keypad. Expect integer 3 or 4.
    
    if (localkey == 3)                        // When <localkey> = 3, the 'up' arrow was pressed, adding 30 to the angle.                 
      ref = ref + 30;                         // Reflect this change in the reference angle.
    if (localkey == 4)                        // When <localkey> = 4, the 'down' arrow was pressed, subtracting 30 from the angle.
      ref = ref - 30;                         // Reflect this change in the reference angle.

  // Keep the range between 0-2pi
    if (ref < 0)                              // If negative degree angle attempted, remain at zero degrees.
      ref = 0;
    if (ref > 360)                            // If over 360 degree angle attempted, remain at 360 degrees.
      ref = 360;
      
    lcd.print(ref);                           // Print the reference angle to the lcd screen; this is the desired direction in which the car should drive.
    delay(100);
  }

  if (ref > 180) {                            // Now that the input has been printed to the screen, modify <ref> for calculations.
    ref = (360 - ref) * -1;                   // Keep range from (-180) - 180 degrees.
  }

  Serial.print("Ref: "); Serial.println(ref);                                       // Print this modified reference angle to the serial monitor.

  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");   // Check if the sensor is connected and detected
    while (1);
  }

  byte c_data[22] = {253, 255, 254, 255, 17, 0, 232, 254, 97, 2, 30, 1, 255, 255, 0, 0, 0, 0, 232, 3, 5, 3};    ///////////////// PASTE YOUR CALIBRATION DATA HERE /////////////
  bno.setCalibData(c_data);                                                                                     // Save calibration data
  delay(1000);
  bno.setExtCrystalUse(true);
  
  FlexiTimer2::set(1000, navigate);           // Define a timer interrupt with a period of "1000*0.001 = 1" s or 100 Hz
  FlexiTimer2::start();                       // Start the timer interrupt.
}



void ReadHeading() {
 
  imu::Vector<3> rawMag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);     // Read the current data of the car via the Magnetometer.

  // Keep these lines of code for our next project, even though it is not necessary for Project4.
  
  //imu::Vector<3> rawGyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);      // Read the current data of the car via the Magnetometer.
  //imu::Vector<3> rawAcc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);  // Read the current data of the car via the Magnetometer.


  // Display the data collected to the serial monitor. Kept for debugging (might be useful for changing to Euler).
  
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

  HEADING = (atan2(rawMag.y(), rawMag.x()) * 180 / PI); // This output makes North = 90 degrees, it will be corrected to 0 degrees.

  // Shifts output to make 0° NORTH, 90° EAST, +/-180° SOUTH, -90° WEST
  if (HEADING <= 180 && HEADING >= -90) {
    HEADING -= 90;
  } else {
    HEADING += 270;
  }

  HEADING += errorHeadingRef;                          // Adds error in HEADING for Tempe (10.37 degrees).

  // Corrects any values which exceed 180 degrees.
  if (HEADING > 180) HEADING -= 360;                   // FINAL OUTPUT: 0° is NORTH, 90° is EAST, +/-180° is SOUTH, -90° is WEST.

  Serial.print("Angle: "); Serial.print(HEADING); Serial.print(" | ");              // Print HEADING to the Serial Monitor.
  lcd.print(HEADING);                                                               // Print HEADING to the LCD Display.
}

void CalculateSteering() {                    // Calculate the steering angle according to the reference heading and actual heading.
  
  int leftBound, rightBound, backBound;       // Create variables to determine the bounds of <ref>. Range is (-150)-180.

  if (ref < -90) {                            // Between -180 and -90 (does not include -180 or -90). Given that ref will be in increments
                                              // of 30 degrees, ref will specifically either be (-150) degrees or (-120) degrees.
    
    // Alter the bounds to accomodate for these angles.
    leftBound = ref + 270;
    rightBound = ref + 90;
    backBound = ref + 180;

    if (HEADING <= rightBound) {                                                          //     Change the angle of steering by an increment of 10
      STEERANGLE = (int)((rightBound - HEADING)/steerPrecision) * steerPrecision;         // degrees, so as to accurately drive in the "desired
    } else if (HEADING >= leftBound) {                                                    // direction."
      STEERANGLE = (int)((rightBound - HEADING + 360)/steerPrecision) * steerPrecision;
      
    } else if (HEADING <= backBound) {                                                    // If the car is facing nearly opposite of the "desired
      STEERANGLE = 0;                                                                     // direction," but still on the left, start turning left
                                                                                          // until the next reading.
      
    } else {                                                                              // Else, start turning right until the next magnetometer
      STEERANGLE = 180;                                                                   // reading.
    }
    
  } else if (ref < 0) {                       //     Between -90 and 0 (includes -90 but not 0). Given that ref will be in increments of 30 degrees,
                                              //  ref will specifically either be (-90, -60, -30) degrees.

    // Alter the bounds to accomodate for these angles.
    leftBound = ref - 90;
    rightBound = ref + 90;
    backBound = ref + 180;

    if (HEADING >= leftBound && HEADING <= rightBound) {                                  //     If the car is between the left and right bounds,
      STEERANGLE = (int)((rightBound - HEADING)/steerPrecision) * steerPrecision;         // change the angle of steering by an increment of 10 degrees,
                                                                                          // so as to accurately drive in the "desired direction".
                                                                                          
    } else if (HEADING <= backBound && HEADING >= rightBound) {                           //     If the car is outside the back and right bounds, start
      STEERANGLE = 0;                                                                     // turning completely left until the next magnetometer reading.
      
    } else {                                                                              //     Else, the car starts turning completely right until the
      STEERANGLE = 180;                                                                   // next magnetometer reading.
    }
    
  } else if (ref <= 90) {                     //     Between 0 and 90 (includes 0 and 90). Given that ref will be in increments of 30 degrees, ref will
                                              // specifically either be (120, 150, 180) degrees.

    // Alter the bounds to accomodate for these angles.
    leftBound = ref - 90;
    rightBound = ref + 90;
    backBound = ref - 180;

    if (HEADING >= leftBound && HEADING <= rightBound) {                                  //     If the car is between the left and right bounds, change
      STEERANGLE = (int)((rightBound - HEADING)/steerPrecision) * steerPrecision;         // the angle of steering by an increment of 10 degrees, so as to
                                                                                          // accurately drive in the "desired direction."
                                                                                          
    } else if (HEADING >= backBound && HEADING <= leftBound) {                            //     If the car is outside the back and left bounds, start
      STEERANGLE = 180;                                                                   // turning completely right until the next magnetometer reading.
      
    } else {                                                                              //     Else, the car starts turning completely left until the
      STEERANGLE = 0;                                                                     // next magnetometer reading.
    }
    
  } else if (ref <= 180) {                    //     Between 90 and 180 (includes 180 but not 90). Given that ref will be in increments of 30 degrees, ref
                                              // will specifically either be (0, 30, 60, 90) degrees.

    // Alter the bounds to accomodate for these angles.
    leftBound = ref - 90;
    rightBound = ref - 270;
    backBound = ref - 180;

    if (HEADING >= leftBound) {                                                           //     Change the angle of steering by an increment of 10 degrees,
      STEERANGLE = (int)((rightBound - HEADING + 360)/steerPrecision) * steerPrecision;   //  so as to accurately drive in the "desired direction".
    } else if (HEADING <= rightBound) {                                             
      STEERANGLE = (int)((rightBound - HEADING)/steerPrecision) * steerPrecision;
      
    } else if (HEADING > rightBound && HEADING <= backBound) {                            //     If the car is outside the right bound and the back bound,
      STEERANGLE = 0;                                                                     // start turning completely left until the next magnetometer
                                                                                          // reading.
                                                                                          
    } else {                                                                              //     Else, the car starts turning completely right until the next
      STEERANGLE = 180;                                                                   // magnetometer reading.
    }
  }

  if (STEERANGLE > 135) STEERANGLE = 135;     // The car cannot turn its wheels more than 135 degrees, so cap it here.
  if (STEERANGLE < 45) STEERANGLE = 45;       // The car canno turn its wheels less than 45 degrees, so cap it here.

  Serial.print("Steer Angle: "); Serial.println(STEERANGLE);// Serial.print(" | ");       // Print the steering angle to the Serial Monitor.
  lcd.print(STEERANGLE);                                                                  // Print the steering angle to the LCD Display.
}

void Actuate() {                              // Input: Steering angle - Output: nothing
  if (millis() > 30000) {                     // After 30 seconds (from the startup)
    carSpeed = 0;                             // Make the car stop after 30 seconds.
    
  } else {                                    // If 30 seconds hasn't passed yet, set the steering angle and the speed.
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

void navigate() {                              // This function will be called every 0.1 seconds (10Hz)
  sei();                                       // Set the interrupt flag ** THIS IS VERY IMPORTANT **. You need to set the interrupt flag or the program will get stuck here!!!
  ReadHeading();                               // Read the heading angle.
  CalculateSteering();                         // Calculate the desired steering angle.
  Actuate();                                   // Actuate (set the steering angle).
}

void loop()
{
  lcd.clear();                                 // clear the LCD Display.
  delay(100);
}
