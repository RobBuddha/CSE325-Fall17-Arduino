#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
#include "DFR_Key.h"


Adafruit_GPS GPS(&Serial1);                   // define GPS object connected to Serial 3
DFR_Key keypad;  
Servo myservo;                                    // define servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);        // define BNO sensor object
LiquidCrystal lcd( 8, 9, 4, 5, 6, 7);             // define lcd pins use these default values for OUR LCD

#define GPSECHO  false

// Global variables that are changed across functions
int STEERANGLE = 90;                           // servo initial angle (range is 0:180)
float HEADING = 0;                             // direction car is facing
boolean usingInterrupt = false;
int carSpeedPin = 2;                           // pin for DC motor (PWM for motor driver)
float errorHeadingBearing = -20;               // error in compass readings
float lat = 0;                                 // GPS latitude in degree decimal multiplied by 100000
float lon = 0;                                 // GPS latitude in degree decimal multiplied by 100000
float latDestination = 0;                      // Bearing reference destination
float lonDestination = 0;                      // Bearing reference destination
float Bearing = 0;                             // bearing angle to destination
int localkey = 0;                              // variable for keypad
int steerPrecision = 10;                       // Variable indicates the step size (in degrees) between steering angles.
int carSpeed = 0;                              // speed of car
int lastServoVal  = 0;
int lastCarSpeed = 0;
float distance = 10;                           // distance from destination (in meters)
bool readyToDrive = false;                     // State of the car
int stopDistance = 2;                          // Distance from reference where car is allowed to stop

void setup() {
  myservo.attach(44); // servo is connected to pin 44
  lcd.begin( 16, 2 ); // LCD type is 16x2 (col & row)
  Serial.begin(9600); // serial for monitoring

  Serial.println("Orientation Sensor Calibration"); Serial.println("");
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {                               // If you want to calibrate using another mode, set it here.
  	Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");       // OPERATION_MODE_COMPASS for a precise tilt compensated compass
  	while (1);
  }

  byte c_data[22] = {0, 0, 0, 0, 0, 0, 74, 255, 113, 2, 255, 0, 255, 255, 0, 0, 1, 0, 232, 3, 54, 3}; // IMU Calibration data
  bno.setCalibData(c_data); // SET CALIBRATION DATA
  bno.setExtCrystalUse(true);
  
  // set timer interrupts
  noInterrupts();                                 // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 59016;                                 // every 0.1 second
  TCCR1B |= (1 << CS12);                          // 256 prescaler
  TIMSK1 |= (1 << TOIE1);                         // enable timer compare interrupt

  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 336;                                   // every 1 second
  TCCR4B |= (1 << CS42);                          // 256 prescaler
  TIMSK4 |= (1 << TOIE4);                         // enable timer compare interrupt
  interrupts();

  GPS.begin(9600); // Begin GPS sampling
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);      // 1 Hz update rate because it's more stable than 10Hz
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);

  while (lat == 0 || lon == 0) { // No GPS signal yet
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("No GPS Signal");
    delay(250);
  }

  // Setting the Bearing reference (Lat and Lon)
  localkey = 0;
  while (localkey != 1) {                         // wait for select button
    lcd.clear();                                  // clear the display
    localkey = keypad.getKey();
    lcd.print("Press Select");                    // first press of the button will save the GPS location
    lcd.setCursor(0, 1);
    lcd.print("to save dest.");
    delay(100);                                   // delay to make display visible
  }

  lcd.clear();
  lcd.print("Reference Saved");
  delay(1000);
  
  latDestination = lat;                           // saving the destination point (latitude)
  lonDestination = lon;                           // saving the destination point (longitude)

  Serial.print("-------------> LATITUDE  DESTINATION: "); Serial.print(latDestination, 5); Serial.println("<-------------");
  Serial.print("-------------> LONGITUDE DESTINATION: "); Serial.print(lonDestination, 5); Serial.println("<-------------");

  localkey = 0;
  while (localkey != 1) {                         // wait for select button
    lcd.clear();
    localkey = keypad.getKey();
    lcd.print("Press Select");                    // after button is pressed once, notify user that next press will start car
    lcd.setCursor(0, 1);
    lcd.print("to drive!");
    delay(100);                                   // delay to make display visible
  }

  readyToDrive = true;
}

// leave this function unchanged//
// Functionality: reads in chars from the GPS module
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
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

ISR(TIMER4_OVF_vect) { // This function will be called every 1 second (1Hz)
  sei();        //   set interrupt flag // don't change this
  TCNT4  = 336; //   re-initialize timer4's value
  ReadGPS();    //   read GPS data
}

void ReadGPS() {
  // read from GPS module and update the current position
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }
  if (GPS.fix) {
    lat = GPS.latitude;
    lon = GPS.longitude;
    if (GPS.lon == 'W') {
      lon = lon * -1;
    }
    if (GPS.lat == 'S') {
      lat = lat * -1;
    }
  }
  Serial.print("Latitude: ");  Serial.print(lat, 8); Serial.print(" | Diff: "); Serial.println(latDestination - lat, 8);
  Serial.print("Longitude: "); Serial.print(lon, 8); Serial.print(" | Diff: "); Serial.println(lonDestination - lon, 8);
}

void ReadHeading() {
  // Function computes the direction that the car is facing
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  HEADING = euler.x(); // Grab euler direction output
  //Serial.println(HEADING);
  
  HEADING -= errorHeadingBearing; // Adds error in HEADING for Tempe. 
  
  // Corrects any values which exceed 180 degrees.
  if (HEADING > 180) HEADING -= 360;
  
  // FINAL OUTPUT: 0° is NORTH, 90° is EAST, +/-180° is SOUTH, -90° is WEST.
  Serial.print("Heading: "); Serial.print(HEADING); Serial.print(" | ");              // Print HEADING to the Serial Monitor.
}

void CalculateBearing() {
  // Function calculates the direction the car needs to go based on GPS data. Bearing will be calculated as the angle from the car's location to the reference point.
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
  
  Bearing = (atan2(deltaLat, deltaLon) * 180/ PI) - 90; // 0° is NORTH, -90° is EAST, -180° is SOUTH, 90/-270° is WEST.

  if (Bearing < -180) {
    Bearing += 360;
  } // 0° is NORTH, -90° is EAST, +/-180° is SOUTH, 90° is WEST.

  Bearing = Bearing * -1;

  Serial.print("Bearing: "); Serial.print(Bearing); Serial.print(" | ");

  // FINAL OUTPUT: 0° is NORTH, 90° is EAST, +/-180° is SOUTH, -90° is WEST.
}

void CalculateSteering() {                    // Calculate the steering angle according to the Bearing reference heading and actual heading.
  int leftBound, rightBound, backBound;       // Create variables to determine the bounds of <Bearing>. Range is (-150)-180.
  
  if (Bearing < -90) {
    // Between -180 and -90 (does not include -180 or -90). Given that Bearing will be in increments
    // of 30 degrees, Bearing will specifically either be (-150) degrees or (-120) degrees.
    
    // Alter the bounds to accomodate for these angles.
    leftBound = Bearing + 270;
    rightBound = Bearing + 90;
    backBound = Bearing + 180;
    
    if (HEADING <= rightBound) {                                                          // Change the angle of steering by an increment of 10
      STEERANGLE = (int)((rightBound - HEADING)/steerPrecision) * steerPrecision;         // degrees, so as to accurately drive in the "desired
    } else if (HEADING >= leftBound) {                                                    // direction."
      STEERANGLE = (int)((rightBound - HEADING + 360)/steerPrecision) * steerPrecision;
    } else if (HEADING <= backBound) {                                                    // If the car is facing nearly opposite of the "desired
      STEERANGLE = 0;                                                                     // direction," but still on the left, start turning left
                                                                                          // until the next reading.
    } else {                                                                              // Else, start turning right until the next magnetometer
      STEERANGLE = 180;                                                                   // reading.
    }
  } else if (Bearing < 0) {
    // Between -90 and 0 (includes -90 but not 0). Given that Bearing will be in increments of 30 degrees,
    // Bearing will specifically either be (-90, -60, -30) degrees.
  
    // Alter the bounds to accomodate for these angles.
    leftBound = Bearing - 90;
    rightBound = Bearing + 90;
    backBound = Bearing + 180;
    
    if (HEADING >= leftBound && HEADING <= rightBound) {                                  // If the car is between the left and right bounds,
      STEERANGLE = (int)((rightBound - HEADING)/steerPrecision) * steerPrecision;         // change the angle of steering by an increment of 10 degrees,
                                                                                          // so as to accurately drive in the "desired direction".
    } else if (HEADING <= backBound && HEADING >= rightBound) {                           // If the car is outside the back and right bounds, start
      STEERANGLE = 0;                                                                     // turning completely left until the next magnetometer reading.
    } else {                                                                              // Else, the car starts turning completely right until the
      STEERANGLE = 180;                                                                   // next magnetometer reading.
    }
  } else if (Bearing <= 90) {
    // Between 0 and 90 (includes 0 and 90). Given that Bearing will be in increments of 30 degrees, Bearing will
    // specifically either be (120, 150, 180) degrees.
  
    // Alter the bounds to accomodate for these angles.
    leftBound = Bearing - 90;
    rightBound = Bearing + 90;
    backBound = Bearing - 180;
    
    if (HEADING >= leftBound && HEADING <= rightBound) {                                  //     If the car is between the left and right bounds, change
      STEERANGLE = (int)((rightBound - HEADING)/steerPrecision) * steerPrecision;         // the angle of steering by an increment of 10 degrees, so as to
    // accurately drive in the "desired direction."
    
    } else if (HEADING >= backBound && HEADING <= leftBound) {                            //     If the car is outside the back and left bounds, start
      STEERANGLE = 180;                                                                   // turning completely right until the next magnetometer reading.
    
    } else {                                                                              //     Else, the car starts turning completely left until the
      STEERANGLE = 0;                                                                     // next magnetometer reading.
    }
  
  } else if (Bearing <= 180) {
    // Between 90 and 180 (includes 180 but not 90). Given that Bearing will be in increments of 30 degrees, Bearing
    // will specifically either be (0, 30, 60, 90) degrees.
    
    // Alter the bounds to accomodate for these angles.
    leftBound = Bearing - 90;
    rightBound = Bearing - 270;
    backBound = Bearing - 180;
    
    if (HEADING >= leftBound) {                                                           //     Change the angle of steering by an increment of 10 degrees,
      STEERANGLE = (int)((rightBound - HEADING + 360)/steerPrecision) * steerPrecision;   //  so as to accurately drive in the "desired direction".
    } else if (HEADING <= rightBound) {                                             
      STEERANGLE = (int)((rightBound - HEADING)/steerPrecision) * steerPrecision;
    } else if (HEADING > rightBound && HEADING <= backBound) {                            //     If the car is outside the right bound and the back bound,
      STEERANGLE = 0;                                                                     // start turning completely left until the next magnetometer reading.
    } else {                                                                              //     Else, the car starts turning completely right until the next
      STEERANGLE = 180;                                                                   // magnetometer reading.
    }
  }
  
  if (STEERANGLE > 135) STEERANGLE = 135;     // The car cannot turn its wheels more than 135 degrees, so cap it here.
  if (STEERANGLE < 45) STEERANGLE = 45;       // The car canno turn its wheels less than 45 degrees, so cap it here.
  
  Serial.print("Steer Angle: "); Serial.println(STEERANGLE);// Serial.print(" | ");       // Print the steering angle to the Serial Monitor.
}

// Haversine formula
void CalculateDistance() {
  // Function calculates distance to destination based on current coordinates and destination coordinates (units = meters)
  float r = 6371000; // Earth's radius in meters

  float lat_rad = lat/100 * PI/180;
  float lon_rad = lon/100 * PI/180;
  float latDst_rad = latDestination/100 * PI/180;
  float lonDst_rad = lonDestination/100 * PI/180;

  float deltaLat = latDst_rad - lat_rad;
  float deltaLon = lonDst_rad - lon_rad;

  float a = sin(deltaLat/2) * sin(deltaLat/2) + cos(lat_rad) * cos(latDst_rad) * sin(deltaLon/2) * sin(deltaLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  distance = r * c; // in meters
}

void Actuate() { // Input: Steering angle - Output: nothing
  if (readyToDrive) {
    if (distance < stopDistance) {
      // Stop car if distance from destination is less than stopDistance (meters)
      carSpeed = 0;
      readyToDrive = false;
    } else {
      carSpeed = 255*0.1;
    }
    if(lastServoVal != STEERANGLE){
      myservo.write(STEERANGLE);
      lastServoVal = STEERANGLE;
    }
    if(lastCarSpeed != carSpeed) {
      analogWrite(carSpeedPin, carSpeed);
      lastCarSpeed = carSpeed;
    }
  }
}

ISR(TIMER1_OVF_vect) {                            // This function will be called every 0.1 second (10Hz)
  sei();                                          // set interrupt flag // don't change this
  TCNT1  = 59016;                                 // reinitialize the timer1's value
  ReadHeading();                                  // read heading
  CalculateBearing();                             // calc bearing
  CalculateSteering();                            // calc steering
  CalculateDistance();                            // calc distance
  Actuate();                                      // Actuate
}

void printLocationOnLCD() {
  lcd.print("Lat: ");
  lcd.print(lat, 5);
  lcd.setCursor(0,1);
  lcd.print("Lon: ");
  lcd.print(lon, 5);
}

void loop() {
  lcd.clear();    // clear the LCD
  // You can print anything on the LCD to debug your program!!!
  printLocationOnLCD();
  delay(100);
}

/*
<script>
var lat = 3314.71799;
var lon = 11151.46893;
var latDestination = 3314.71777;
var lonDestination = 11151.46875;

var deltaLat = latDestination - lat;
var deltaLon = lonDestination - lon;

document.write("DeltaLat: " + deltaLat + " | ");
document.write("DeltaLon: " + deltaLon + " | ");

var hypotenuse = Math.sqrt(Math.pow(deltaLat,2) + Math.pow(deltaLon,2));

var distance = 111000 * hypotenuse;

distance = distance / 100;

document.writeln(distance + " meters");

</script>
*/
