#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
#include "DFR_Key.h"


Adafruit_GPS GPS(&Serial3);                   // define GPS object connected to Serial 3
DFR_Key keypad;  
Servo myservo;                                // define servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);    // define BNO sensor object
LiquidCrystal lcd( 8, 9, 4, 5, 6, 7); // define lcd pins use these default values for OUR LCD

#define GPSECHO  false

// Global variables that are changed across functions
int STEERANGLE = 90;       // servo initial angle (range is 0:180)
float HEADING = 0;  // heading
boolean usingInterrupt = false;
int carSpeedPin = 2;      // pin for DC motor (PWM for motor driver)
float errorHeadingRef = 0;        // error
long int lat;  // GPS latitude in degree decimal multiplied by 100000
long int lon;  // GPS latitude in degree decimal multiplied by 100000
long int latDestination = 33.423933 * 100000;     // reference destination
long int lonDestination = -111.939585 * 100000;   // reference destination
float Bearing = 0;                // bearing angle to destination
int localkey = 0;                 // variable for keypad

void setup() {
  myservo.attach(44);     // servo is connected to pin 44
  lcd.begin( 16, 2 );     // LCD type is 16x2 (col & row)
  Serial.begin(9600);     // serial for monitoring
  Serial.println("Orientation Sensor Calibration"); Serial.println("");
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) { //if you want to calibrate using another mode, set it here. OPERATION_MODE_COMPASS for a precise tilt compensated compass (Section 3.3.2 / 3.3.3)
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  ///Setting the reference (Lat and Lon)///
  localkey = 0;
  while (localkey != 1) {    // wait for select button
    lcd.clear();
    localkey = keypad.getKey();
    lcd.print("Press Select");
    lcd.setCursor(0, 1);
    lcd.print("to save dest.");
    delay(100);               // delay to make display visible
  }
  ReadGPS();
  latDestination = lat;     // saving the destiantion point
  lonDestination = lon;     // saving the destiantion point
  localkey = 0;
  while (localkey != 1) {   // wait for select button
    lcd.clear();
    localkey = keypad.getKey();
    lcd.print("Press Select");
    lcd.setCursor(0, 1);
    lcd.print("to drive!");
    delay(100);             // delay to make display visible
  }


  byte c_data[22] = {0, 0, 0, 0, 0, 0, 211, 3, 117, 4, 55, 5, 255, 255, 255, 255, 255, 255, 232, 3, 21, 3};
  bno.setCalibData(c_data);                                                                                       // SET CALIBRATION DATA
  bno.setExtCrystalUse(true);
  // set timer interrupts
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 59016;           // every 0.1 second
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);  // enable timer compare interrupt

  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 336;             // every 1 second
  TCCR4B |= (1 << CS42);    // 256 prescaler
  TIMSK4 |= (1 << TOIE4);  // enable timer compare interrupt
  interrupts();


  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate // it's more stable than 10Hz
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
}

SIGNAL(TIMER0_COMPA_vect) { // leave this function unchanged//
  char c = GPS.read();    // this function is for reading chars from GPS module
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

ISR(TIMER4_OVF_vect) { // This function will be called every 1 second
  sei();        //   set interrupt flag // don't change this
  TCNT4  = 336; //   re-initialize timer4's value
  ReadGPS();    //   read GPS data
}

void ReadGPS() {
  // read from GPS module and update the current position
}


void ReadHeading() { // Output: HEADING
  // read Heading angle
}

void CalculateBearing() {
  // calculate bearing angle based on current and destination locations (GPS coordinates)
}

void CalculateSteering() { // Input: HEADING // Output: STEERANGLE
  // calculate steering angle based on heading and bearing
}

void CalculateDistance() {
  // calculate distance to destination based on current and destination coordinates
}

void Actuate() {
  // set car's direction and speed
}

ISR(TIMER1_OVF_vect) {        // This function will be called every 0.1 second
  sei();                  // set interrupt flag // don't change this
  TCNT1  = 59016;         // reinitialize the timer1's value
  ReadHeading();          // read heading
  CalculateBearing();     // calc bearing
  CalculateSteering();    // calc steering
  CalculateDistance();    // calc distance
  Actuate();              // Actuate
}


void printHeadingOnLCD() {

}

void printLocationOnLCD() {

}

void printDistanceOnLCD() {

}

void loop() {
  lcd.clear();    // clear the LCD
  // You can print anything on the LCD to debug your program!!!
  printHeadingOnLCD();
  printLocationOnLCD();
  delay(100);
}
