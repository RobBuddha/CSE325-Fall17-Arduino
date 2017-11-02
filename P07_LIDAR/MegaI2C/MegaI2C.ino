#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
#include "DFR_Key.h"

Adafruit_GPS GPS(&Serial3);                   // define GPS object DO NOT USE Serial0
DFR_Key keypad;                               // define keypad object
Servo myservo;                                // define servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);    // define BNO sensor object

#define GPSECHO  false                        // echo GPS Sentence 
#define Threshold 5                           // Threshold for Obstacle avoidance (number of obstacles)

LiquidCrystal lcd( 8, 9, 4, 5, 6, 7); // define lcd pins use these default values for OUR LCD

// Global variables that change across functions
int STEERANGLE = 90;        // servo initial angle (range is 0:180)
float HEADING = 0;          // Heading in degree
uint8_t LidarRight;             // LIDAR left
uint8_t LidarLeft;              // LIDAR right
boolean usingInterrupt = false;
int carSpeedPin = 2;              // pin for DC motor (PWM for motor driver). don't use other pins....
float errorHeadingRef = 0;        // error
long int lat;                     // GPS latitude in degree decimal * 100000   |     we multiply decimal degree by 100000 to convert it to meter  https://en.wikipedia.org/wiki/Decimal_degrees
long int lon;                     // GPS latitude in degree decimal * 100000   |     0.00001 decimal degree is equal to 1.0247 m at 23 degree N/S
long int latDestination = 33.425891 * 100000;       // define an initial reference Latitude of destination
long int lonDestination =  -111.940458 * 100000;    // define an initial reference Longitude of destination
float Bearing = 0;                                  // initialize bearing
int localkey = 0;                                   // var

void setup() {
  myservo.attach(44);     // servo is connected to pin 44     (All pins are used by LCD except 2. Pin 2 is used for DC motor)
  lcd.begin( 16, 2 );     // LCD type is 16x2 (col & row)

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


  Serial.begin(9600);     // serial for monitoring
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  byte c_data[22] = {0, 0, 0, 0, 0, 0, 209, 4, 9, 5, 9, 6, 0, 0, 255, 255, 255, 255, 232, 3, 1, 3};               // Use your CALIBRATION DATA
  bno.setCalibData(c_data);                                                                                       // SET CALIBRATION DATA
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

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);   // ask GPS to send RMC & GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate, don't use higher rates
  GPS.sendCommand(PGCMD_ANTENNA);               // notify if antenna is detected
  useInterrupt(true);                           // use interrupt for reading chars of GPS sentences (From Serial Port)

  Wire.begin(); // Join I2C Bus as master
}

SIGNAL(TIMER0_COMPA_vect) {       // don't change this !!
  char c = GPS.read();            // interrupt for reading GPS sentences char by char....
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
#endif
}

void useInterrupt(boolean v) {    // enable interrupt for GPS, don't change this!!
  if (v) {
    OCR0A = 0xAF;               // Timer0 is already used for millis() - we'll just interrupt somewhere
    TIMSK0 |= _BV(OCIE0A);      // in the middle by Output Compare Register A (OCR0A) and "TIMER0_COMPA_vect" function is called
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A); // do not call the interrupt function COMPA anymore
    usingInterrupt = false;
  }
}

ISR(TIMER4_OVF_vect) {  // Timer interrupt for reading GPS DATA
  sei();        //   reset interrupt flag
  TCNT4  = 336; //   re-initialize timer value
  GPSRead();    //   read GPS data
}

void GPSRead() {
  // read GPS data
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }
  if (GPS.fix) {
    lat = GPS.latitudeDegrees;
    lon = GPS.longitudeDegrees;
  }
}


void ReadHeading()
{
  // calculate HEADING
}

void CalculateBearing() {
  // Calculate Bearing
}

void CalculateSteer() {
  // Calculate Steer angle based on GPS data and IMU
}

void SetCarDirection() {  // Input: Lidar data
  // Set Steering angle,
  // If any obstacle is detected by Lidar, Ignore steering angle and turn left or right based on observation
}

void SetCarSpeed() {      // Input: GPS data
  // set speed,
  // if destination is within 5 meters of current position, set speed to zero.
}

void ReadLidar() {    // Output: Lidar Data
  // read Lidar Data from Nano Board (I2C)
  // you should request data from Nano and read the number of obstacle (within the range) on your rightside and leftside
  // Then, you can decide to either do nothing, turn left or turn right based on threshold. For instance, 0 = do nothing, 1= left and 2 = right

  // Request Left Data
  uint8_t requestType = 1;
  byte request = requestType & 0xFF;
  Wire.beginTransmission(8);
  Wire.write(request);
  Wire.endTransmission();

  Wire.requestFrom(8, 1); // Request 1 byte from nano
  while (Wire.available()) {
    LidarLeft = Wire.read();
  }
  
  //Request Right Data
  requestType = 2;
  request = requestType & 0xFF;
  Wire.beginTransmission(8);
  Wire.write(request);
  Wire.endTransmission();

  Wire.requestFrom(8, 1); // Request 1 byte from nano
  while (Wire.available()) {
    LidarRight = Wire.read();
  }

  Serial.print("Left  - "); Serial.println(LidarLeft);
  Serial.print("Right - "); Serial.println(LidarRight);
}

ISR(TIMER1_OVF_vect) {        // function will be call every 0.1 seconds
  sei();                  // reset interrupt flag
  TCNT1  = 59016;
  ReadHeading();
  CalculateBearing();
  CalculateSteer();
  SetCarDirection();
  SetCarSpeed();
}


void printHeadingOnLCD() {
  lcd.print("Sending");
}

void printLocationOnLCD() {

}

void printDistanceOnLCD() {

}

void printObstacleOnLCD() {

}

void loop() {
  lcd.clear();      // clear lcd
  // Pring data to LCD in order to debug your program!
  printHeadingOnLCD();
  printObstacleOnLCD();
  ReadLidar();
  delay(1000);
}
