/* File: MegaI2C.ino
 *  
 *  Description: Program runs the car...
 *  
 *  Authors:  Erin Hintze, Robert Alimov
 *  Group:    Group 6
 */

#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "DFR_Key.h"

#define GPSECHO false                             // echo GPS Sentence 
#define THRESHOLD 3                              // threshold for Obstacle avoidance (number of obstacles)
#define NEARBY_THRESHOLD 2

Adafruit_GPS GPS(&Serial2);                       // define GPS object DO NOT USE Serial0
DFR_Key keypad;                                   // define keypad object
Servo myservo;                                    // define servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);        // define BNO sensor object

LiquidCrystal lcd( 8, 9, 4, 5, 6, 7);             // define lcd pins use these default values for OUR LCD
SoftwareSerial esp8266(50,51);                    // Pin 50 = ESP_TX->MEGA_RX | Pin 51 = ESP_RX->MEGA_TX

// Global variables that change across functions
int localkey = 0;                                 // variable for reading lcd button values
boolean usingInterrupt = false;                   // interrupt variable
uint8_t inc = 0;                                  

// Heading variables
float errorHeadingRef = -10.3;                     // heading error for Tempe
float headingAngle = 0;                           // angle where the car is currently facing relative to North
float bearingAngle = 0;                           // angle where the car must go relative to car's current position

// Actuation variables
int steerAngle = 90;                              // servo angle controlling steering (0 to 180, 0 is left, 90 is straight, 180 is right)
int carSpeedPin = 2;                              // pin for DC motor (PWM for motor driver).
int steerPrecision = 5;                           // indicates the step size between steering angles (in degrees)
int carSpeed = 0;                                 // pwm value for speed of the car (0 - 255)
float distance;                                   // distance from target
int stopDistance = 5;                             // distance from destination at which car will stop
boolean shouldActuate = false;                    // determines if car should keep actuating or not

// Lidar variables
uint8_t lidarLeft0;                               // variable for detected points on left hand side within 2.5 meters (331-359)
uint8_t lidarLeft1;                               // variable for detected points on left hand side within 2.5 meters (301-330)
uint8_t lidarLeft2;                               // variable for detected points on left hand side within 2.5 meters (271-300)

uint8_t lidarRight0;                              // variable for detected points on right hand side within 2.5 meters (0-29 degrees)
uint8_t lidarRight1;                              // variable for detected points on right hand side within 2.5 meters (30-59 degrees)
uint8_t lidarRight2;                              // variable for detected points on right hand side within 2.5 meters (60-89 degrees)

uint8_t lidarNearby;                              // variable for detected points within 6 meters

// GPS variables
float lat;                                        // GPS latitude in degree decimal * 100000 | we multiply decimal degree by 100000 to convert it to meter  https://en.wikipedia.org/wiki/Decimal_degrees
float lon;                                        // GPS latitude in degree decimal * 100000 | 0.00001 decimal degree is equal to 1.0247 m at 23 degree N/S
float latDestination;                             // define an initial reference Latitude of destination
float lonDestination;                             // define an initial reference Longitude of destination

void setup() {
  myservo.attach(44);     // servo is connected to pin 44     (All pins are used by LCD except 2. Pin 2 is used for DC motor)
  lcd.begin( 16, 2 );     // LCD type is 16x2 (col & row)
  Serial.begin(9600);     // serial for monitoring
  setup_wifi();

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);   // ask GPS to send RMC & GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate, don't use higher rates
  GPS.sendCommand(PGCMD_ANTENNA);               // notify if antenna is detected
  useInterrupt(true);                           // use interrupt for reading chars of GPS sentences (From Serial Port)

  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
    //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  byte c_data[22] = {0, 0, 0, 0, 0, 0, 34, 0, 89, 0, 133, 0, 255, 255, 0, 0, 1, 0, 232, 3, 29, 3};                        // Use your CALIBRATION DATA
  bno.setCalibData(c_data);                                                                                               // SET CALIBRATION DATA
  bno.setExtCrystalUse(true);

  // set timer interrupts
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;               // initialize timer1
  TCCR1B = 0;               // initialize timer1
  TCNT1  = 59016;           // interrupt is generated every 0.1 second ( ISR(TIMER1_OVF_vect) is called)
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer compare interrupt

  TCCR4A = 0;               // initialize timer4
  TCCR4B = 0;               // initialize timer4
  TCNT4  = 336;             // interrupt is generated every 1 second  ( ISR(TIMER4_OVF_vect) is called)
  TCCR4B |= (1 << CS42);    // 256 prescaler
  TIMSK4 |= (1 << TOIE4);   // enable timer compare interrupt
  interrupts();             // enable intrrupt flag again

  while (lat == 0 || lon == 0) { // No GPS signal yet
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("No GPS Signal");                   // Alert user that GPS signal has not yet been obtained.
    delay(1000);
  }

  // Setting the reference (Lat and Lon) //
  localkey = 0;
  while (localkey != 1) {
    lcd.clear();
    localkey = keypad.getKey();
    lcd.print("Press Select");
    lcd.setCursor(0, 1);
    lcd.print("to save dest.");
    delay(100);
  }

  GPSRead();
  latDestination = lat;     // saving the destination point
  lonDestination = lon;     // saving the destination point
  //Serial.print("Lat Destination - "); Serial.println(latDestination, 7);
  //Serial.print("Lon Destination - "); Serial.println(lonDestination, 7);

  // Waiting to activate //
  localkey = 0;
  while (localkey != 1) {
    lcd.clear();
    localkey = keypad.getKey();
    lcd.print("Press Select");
    lcd.setCursor(0, 1);
    lcd.print("to drive!");
    delay(100);
  }

  shouldActuate = true;
}

void setup_wifi() {
  esp8266.begin(9600);
  
  // Set AP mode
  Serial.write("Sent: AT+CWMODE_CUR=2\n");
  esp8266.write("AT+CWMODE_CUR=2\r\n");
  if (get_response()) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
    return;
  }

  // Create AP
  Serial.write("Sent: AT+CWSAP_CUR=\"esp_01\",\"cse325demo\",6,3\n");
  esp8266.write("AT+CWSAP_CUR=\"esp_01\",\"cse325demo\",6,3\r\n");
  if (get_response()) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
    return;
  }

  // Set up router IP addr
  Serial.write("Sent: AT+CIPAP_CUR=\"192.168.4.1\",\"192.168.4.1\",\"255.255.255.0\"\n");
  esp8266.write("AT+CIPAP_CUR=\"192.168.4.1\",\"192.168.4.1\",\"255.255.255.0\"\r\n");
  if (get_response()) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
    return;
  }

  // Enable multiple TCP connections
  Serial.write("Sent: AT+CIPMUX=1\n");
  esp8266.write("AT+CIPMUX=1\r\n");
  if (get_response()) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
    return;
  }

  // Start TCP server on Port 325
  Serial.write("Sent: AT+CIPSERVER=1,325\n");
  esp8266.write("AT+CIPSERVER=1,325\r\n");
  if (get_response()) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
    return;
  }
}

boolean get_response() {
  String response = "";

  boolean isResponseReady = false;
  boolean isOK = false;
  
  delay(10);
  char prev_c = '\0';
  while (!isResponseReady) {
    char c = esp8266.read();
    if (c == '\r' || c == '\n') {
      prev_c = '\n';
    } else {
      if (prev_c == '\n') {
        prev_c = '\0';
        response += ';';
      }
      response += c;
    }
    if (response.indexOf("OK") != -1) {
      isResponseReady = true;
      isOK = true;
    } else if (response.indexOf("ERROR") != -1) {
      isResponseReady = true;
    }
  }
  return isOK;
}

SIGNAL(TIMER0_COMPA_vect) {       // don't change this !!
  char c = GPS.read();            // interrupt for reading GPS sentences char by char....
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
#endif
}

void useInterrupt(boolean v) {  // enable interrupt for GPS, don't change this!!
  if (v) {
    OCR0A = 0xAF;               // Timer0 is already used for millis() - we'll just interrupt somewhere
    TIMSK0 |= _BV(OCIE0A);      // in the middle by Output Compare Register A (OCR0A) and "TIMER0_COMPA_vect" function is called
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);     // do not call the interrupt function COMPA anymore
    usingInterrupt = false;
  }
}

ISR(TIMER4_OVF_vect) {  // Timer interrupt for reading GPS and Lidar data (called every 1 second)
  sei();                // reset interrupt flag
  TCNT4  = 336;         // re-initialize timer value
  GPSRead();            // read GPS data
}

void GPSRead() {
  // read GPS data
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
  }
  if (GPS.fix) {
    lat = GPS.latitudeDegrees;
    lon = GPS.longitudeDegrees;
  }
}

void ReadHeading() {
  // Function computes the direction that the car is facing
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  headingAngle = euler.x();                                  // Grab euler direction output

  headingAngle += errorHeadingRef;                           // Factors in error in headingAnglefor Tempe.

  // Corrects any values which exceed 180 degrees.
  if (headingAngle > 180) headingAngle -= 360;

  //Serial.print("Heading - "); Serial.println(headingAngle);

  // FINAL OUTPUT: 0° is NORTH, 90° is EAST, +/-180° is SOUTH, -90° is WEST.
}

void CalculateBearing() {
  // Function calculates the direction the car needs to go based on GPS data.
  // bearingAngle will be calculated as the angle from the car's location to the reference point.
  // Angle will be the degrees from North.
  //
  //             |     |
  //             |   ° * Car
  //             |   /
  //             | /
  //   ----------*----------
  //             |Ref
  //             |
  //             |
  //
  // Y = Latitude, X = Longitude
  // Output of atan2: 90° is NORTH, 0° is EAST, -90° is SOUTH, +/-180° is WEST.

  float deltaLat = latDestination - lat;
  float deltaLon = lonDestination - lon;

  bearingAngle = (atan2(deltaLat, deltaLon) * 180 / PI) - 90; // OUTPUT: 0° is NORTH, -90° is EAST, -180° is SOUTH, 90/-270° is WEST.

  if (bearingAngle < -180) {
    bearingAngle += 360;                           // OUTPUT: 0° is NORTH, -90° is EAST, +/-180° is SOUTH, 90° is WEST.
  }

  bearingAngle = bearingAngle * -1;                     // FINAL OUTPUT: 0° is NORTH, 90° is EAST, +/-180° is SOUTH, -90° is WEST.

  //Serial.print("Bearing - "); Serial.println(bearingAngle);
}

void CalculateSteer() {
  // Calculate steering angle (steerAngle) based on GPS data (lat, lon, latDestination, lonDestination) and IMU (heading)
  int leftBound, rightBound, backBound;       // Create variables to determine the bounds of <Bearing>. Range is (-179) - 180.

  if (bearingAngle < -90) { // Between -180 and -90 (does not include -180 or -90).
    // Alter the bounds to accomodate for these angles.
    leftBound = bearingAngle + 270;
    rightBound = bearingAngle + 90;
    backBound = bearingAngle + 180;

    if (headingAngle <= rightBound) {                                                          // Change the angle of steering by an increment of 10
      steerAngle = (int)((rightBound - headingAngle) / steerPrecision) * steerPrecision;       // degrees, so as to accurately drive in the "desired
    } else if (headingAngle >= leftBound) {                                                    // direction."
      steerAngle = (int)((rightBound - headingAngle + 360) / steerPrecision) * steerPrecision;
    } else if (headingAngle <= backBound) {                                                    // If the car is facing nearly opposite of the "desired
      steerAngle = 0;                                                                     // direction," but still on the left, start turning left
      // until the next reading.

    } else {                                                                              // Else, start turning right until the next magnetometer
      steerAngle = 180;                                                                   // reading.
    }

  } else if (bearingAngle < 0) { // Between -90 and 0 (includes -90 but not 0).
    // Alter the bounds to accomodate for these angles.
    leftBound = bearingAngle - 90;
    rightBound = bearingAngle + 90;
    backBound = bearingAngle + 180;

    if (headingAngle >= leftBound && headingAngle <= rightBound) {                                  // If the car is between the left and right bounds,
      steerAngle = (int)((rightBound - headingAngle) / steerPrecision) * steerPrecision;       // change the angle of steering by an increment of 10 degrees,
      // so as to accurately drive in the "desired direction".

    } else if (headingAngle <= backBound && headingAngle >= rightBound) {                           // If the car is outside the back and right bounds, start
      steerAngle = 0;                                                                     // turning completely left until the next magnetometer reading.

    } else {                                                                              // Else, the car starts turning completely right until the
      steerAngle = 180;                                                                   // next magnetometer reading.
    }
  } else if (bearingAngle <= 90) { // Between 0 and 90 (includes 0 and 90).
    // Alter the bounds to accomodate for these angles.
    leftBound = bearingAngle - 90;
    rightBound = bearingAngle + 90;
    backBound = bearingAngle - 180;

    if (headingAngle >= leftBound && headingAngle <= rightBound) {                                  //     If the car is between the left and right bounds, change
      steerAngle = (int)((rightBound - headingAngle) / steerPrecision) * steerPrecision;       // the angle of steering by an increment of 10 degrees, so as to
      // accurately drive in the "desired direction."

    } else if (headingAngle >= backBound && headingAngle <= leftBound) {                            //     If the car is outside the back and left bounds, start
      steerAngle = 180;                                                                   // turning completely right until the next magnetometer reading.

    } else {                                                                              //     Else, the car starts turning completely left until the
      steerAngle = 0;                                                                     // next magnetometer reading.
    }

  } else if (bearingAngle <= 180) { // Between 90 and 180 (includes 180 but not 90).
    // Alter the bounds to accomodate for these angles.
    leftBound = bearingAngle - 90;
    rightBound = bearingAngle - 270;
    backBound = bearingAngle - 180;

    if (headingAngle >= leftBound) {                                                           //     Change the angle of steering by an increment of 10 degrees,
      steerAngle = (int)((rightBound - headingAngle + 360) / steerPrecision) * steerPrecision; //  so as to accurately drive in the "desired direction".
    } else if (headingAngle <= rightBound) {
      steerAngle = (int)((rightBound - headingAngle) / steerPrecision) * steerPrecision;
    } else if (headingAngle > rightBound && headingAngle <= backBound) {                            //     If the car is outside the right bound and the back bound,
      steerAngle = 0;                                                                     // start turning completely left until the next magnetometer reading.

    } else {                                                                              //     Else, the car starts turning completely right until the next
      steerAngle = 180;                                                                   // magnetometer reading.
    }
  }

  if (steerAngle > 135) steerAngle = 135;     // The car cannot turn its wheels more than 135 degrees, so cap it here.
  if (steerAngle < 45) steerAngle = 45;       // The car canno turn its wheels less than 45 degrees, so cap it here.
}

void SetCarDirection() {
  // Set steering angle (steerAngle) based on lidar data (lidarRight and lidarLeft). If any obstacle is detected by lidar,
  // ignore steering angle and turn left or right based on lidar data.

  if (lidarLeft2 > THRESHOLD || lidarLeft1 > THRESHOLD || lidarLeft0 > THRESHOLD || lidarRight0 > THRESHOLD || lidarRight1 > THRESHOLD || lidarRight2 > THRESHOLD) {
    if (steerAngle < 60) { // 45, 50, 55 -----------------------------
      
      if (lidarLeft2 > THRESHOLD) {
        if (lidarLeft1 > THRESHOLD) {
          if (lidarLeft0 > THRESHOLD) {
            steerAngle = 105;
          } else {
            steerAngle = 75;
          }
        } else {
          steerAngle = 60;
        }
      } else if (lidarLeft1 > THRESHOLD) {
        steerAngle = 75;
      }
      
    } else if (steerAngle < 80) { // 60, 65, 70, 75 ------------------
      
      if (lidarLeft1 > THRESHOLD) {
        if (lidarLeft0 > THRESHOLD) {
          if (lidarRight0 > THRESHOLD) {
            steerAngle = 110;
          } else {
            steerAngle = 100;
          }
        } else {
          steerAngle = 90;
        }
      } else if (lidarLeft2 > THRESHOLD) {
        // unchanged
      } else if (lidarLeft0 > THRESHOLD) {
        steerAngle = 65;
      }

    } else if (steerAngle < 101) { // 80, 85, 90, 95, 100 ------------

      if (lidarLeft0 > THRESHOLD) {
        if (lidarRight0 > THRESHOLD) {
          if (steerAngle <= 90) {
            if (lidarLeft1 > THRESHOLD) {
              if (lidarRight1 > THRESHOLD) {
                steerAngle = 45;
              } else {
                steerAngle = 120;
              }
            } else {
              steerAngle = 65;
            }
          } else {
            if (lidarRight1 > THRESHOLD) {
              if (lidarRight1 > THRESHOLD) {
                steerAngle = 135;
              } else {
                steerAngle = 60;
              }
            } else {
              steerAngle = 115;
            }
          }
        } else {
          steerAngle = 100;
        }
      } else if (lidarRight0 > THRESHOLD) {
        if (lidarLeft0 > THRESHOLD) {
          if (steerAngle >= 90) {
            if (lidarRight1 > THRESHOLD) {
              if (lidarLeft1 > THRESHOLD) {
                steerAngle = 135;
              } else {
                steerAngle = 60;
              }
            } else {
              steerAngle = 115;
            }
          } else {
            if (lidarLeft1 > THRESHOLD) {
              if (lidarRight1 > THRESHOLD) {
                steerAngle = 45;
              } else {
                steerAngle = 120;
              }
            } else {
              steerAngle = 65;
            }
          }
        } else {
          steerAngle = 80;
        }
      }
      
    } else if (steerAngle < 121) { // 105, 110, 115, 120 -------------

      if (lidarRight1 > THRESHOLD) {
        if (lidarRight0 > THRESHOLD) {
          if (lidarLeft0 > THRESHOLD) {
            steerAngle = 70;
          } else {
            steerAngle = 80;
          }
        } else {
          steerAngle = 90;
        }
      } else if (lidarLeft2 > THRESHOLD) {
        // unchanged
      } else if (lidarLeft0 > THRESHOLD) {
        steerAngle = 115;
      }

    } else { // 125, 130, 135 ----------------------------------------

      if (lidarRight2 > THRESHOLD) {
        if (lidarRight1 > THRESHOLD) {
          if (lidarRight0 > THRESHOLD) {
            steerAngle = 75;
          } else {
            steerAngle = 105;
          }
        } else {
          steerAngle = 120;
        }
      } else if (lidarRight1 > THRESHOLD) {
        steerAngle = 105;
      }
      
    }
  }
}

void CalculateDistance() {      // Input: GPS data
  // set speed,
  // if destination is within 5 meters of current position, set speed to zero.
  float r = 6371000;                          // Earth's radius in meters
  float lat_rad = lat * PI / 180;
  float lon_rad = lon * PI / 180;
  float latDst_rad = latDestination * PI / 180;
  float lonDst_rad = lonDestination * PI / 180;

  float deltaLat = latDst_rad - lat_rad;      // Change in latitude
  float deltaLon = lonDst_rad - lon_rad;      // Change in longitude

  float a = sin(deltaLat/2) * sin(deltaLat/2) + cos(lat_rad) * cos(latDst_rad) * sin(deltaLon/2) * sin(deltaLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  distance = r * c;                           // in meters
}

void ReadLidar() {    // Output: Lidar Data
  // read Lidar Data from Nano Board (I2C)

  LidarRequest(0, &lidarLeft2);
  LidarRequest(1, &lidarLeft1);
  LidarRequest(2, &lidarLeft0);
  LidarRequest(3, &lidarRight0);
  LidarRequest(4, &lidarRight1);
  LidarRequest(5, &lidarRight2);
  LidarRequest(6, &lidarNearby);
}

void LidarRequest(uint8_t requestType, uint8_t* variable) {
  // Request template for lidar data
  byte request = requestType & 0xFF;
  Wire.beginTransmission(8);
  Wire.write(request);
  Wire.endTransmission();

  Wire.requestFrom(8, 1); // Request 1 byte from nano
  while (Wire.available()) {
    *variable = Wire.read();
  }
}

void Actuate() {
  if (shouldActuate) {
    if (distance < stopDistance) {
      carSpeed = 0;
      shouldActuate = false;
    } else {
      if (lidarNearby > NEARBY_THRESHOLD) {
        carSpeed = 25;
      } else {
        carSpeed = 65;
      }
    }
    analogWrite(carSpeedPin, carSpeed);
    myservo.write(steerAngle);
  }
}

ISR(TIMER1_OVF_vect) {        // function will be call every 0.1 seconds
  sei();                      // reset interrupt flag
  TCNT1  = 59016;
  ReadHeading();
  CalculateBearing();
  CalculateSteer();
  SetCarDirection();
  CalculateDistance();
  Actuate();

  if (inc == 3) {
    inc = 0;
    ReadLidar();
  } else {
    inc++;
  }
}

void printToLCD() {
  lcd.print("H:"); lcd.print(headingAngle, 7);
  lcd.setCursor(0, 1);
  lcd.print("D:"); lcd.print(distance, 7);
}

void loop() {
  lcd.clear();
  printToLCD();
  delay(1000);
}
