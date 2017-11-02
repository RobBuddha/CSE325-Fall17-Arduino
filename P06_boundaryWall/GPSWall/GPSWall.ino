#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
#include "DFR_Key.h"

Adafruit_GPS GPS(&Serial1);                   // define GPS object
Servo myservo;                                // define servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);    // define BNO sensor object
LiquidCrystal lcd( 8, 9, 4, 5, 6, 7);         // define lcd pins use these default values for OUR LCD
DFR_Key keypad;                               // Define keypad object. This will be used be a TA to input the desired direction.

#define GPSECHO  false
#define lThreshold 5                          // Lidar Threshold
#define dThreshold 10                         // GPS Wall Threshol

typedef struct coord {
  double x;
  double y;
} Coordinate;

// Global variables that change across functions
int STEERANGLE = 90;                            // servo initial angle (range is 0:180)
float HEADING = 0;                              // heading
boolean usingInterrupt = false;                 // Using interrupt for reading GPS chars
int carSpeedPin = 2;                            // pin for DC motor (PWM for motor driver)
float errorHeadingRef = 10.37;                  // Heading error
double lat = 0;                                 // GPS latitude in degree decimal
double lon = 0;                                 // GPS latitude in degree decimal
double threshold = 5;				                    // Only correct if within 10m of wall
int localkey = 0;                               // Variable for reading the keypad value
int ref = 0;                                    // Reference angle, car will drive towards this
int steerPrecision = 5;                         // Variable indicates the step size (in degrees) between steering angles.
int carSpeed = 255 * 0;                         // Speed of the car

///////////////////////////////////////// Boundary points  //////////////////////////////////////////
//double latPoint1 = 33.421846;      // reference destination (Point1)
//double lonPoint1 =  -111.934683;   // reference destination (Point1)
//
//double latPoint2 = 33.421846;      // reference destination (Point2)
//double lonPoint2 =  -111.933382;   // reference destination (Point2)
//
//double latPoint3 = 33.421147;      // reference destination (Point3)
//double lonPoint3 =  -111.9337721;  // reference destination (Point3)
//
//double latPoint4 = 33.420825;      // reference destination (Point4)
//double lonPoint4 =  -111.933824;   // reference destination (Point4)
//
//double latPoint5 = 33.4208231;     // reference destination (Point5)
//double lonPoint5 =  -111.9341681;  // reference destination (Point5)
//
//double latPoint6 = 33.421151;      // reference destination (Point6)
//double lonPoint6 =  -111.934220;   // reference destination (Point6)
// ---------------------------------------------------------------- //
double latPoint1 = 33.2511027;       // reference destination (Point1)
double lonPoint1 =  -111.8419250;    // reference destination (Point1)

double latPoint2 = 33.2510547;       // reference destination (Point2)
double lonPoint2 =  -111.8417282;    // reference destination (Point2)

double latPoint3 = 33.2509350;       // reference destination (Point3)
double lonPoint3 =  -111.8417282;    // reference destination (Point3)

double latPoint4 = 33.2508747;       // reference destination (Point4)
double lonPoint4 =  -111.8419250;    // reference destination (Point4)

double latPoint5 = 33.2509350;       // reference destination (Point5)
double lonPoint5 =  -111.8421245;    // reference destination (Point5)

double latPoint6 = 33.2510547;       // reference destination (Point6)
double lonPoint6 =  -111.8421245;    // reference destination (Point6)
/////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  myservo.attach(44);                         // servo is connected to pin 44
  lcd.begin( 16, 2 );                         // LCD type is 16x2 (col & row)
  Serial.begin(9600);                         // serial for monitoring

  while (localkey != 1) {
    lcd.clear();
    lcd.print("Press to begin...");
    localkey = keypad.getKey();               // Read in the value from the keypad.
    delay(100);
  }
  
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  byte c_data[22] = {0, 0, 0, 0, 0, 0, 209, 4, 9, 5, 9, 6, 0, 0, 255, 255, 255, 255, 232, 3, 1, 3};               // YOUR Calibration DATA
  bno.setCalibData(c_data);                                                                                       // SET CALIBRATION DATA
  bno.setExtCrystalUse(true);

  // set timer interrupts
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 59016;           // every 0.1 second
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer compare interrupt

  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 336;             // every 1 second
  TCCR4B |= (1 << CS42);    // 256 prescaler
  TIMSK4 |= (1 << TOIE4);   // enable timer compare interrupt
  interrupts();

  // initialize GPS module
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // set RMC & GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);               // notify if antenna detected
  useInterrupt(true);                           // use interrupt for reading gps data
}

SIGNAL(TIMER0_COMPA_vect) { // Interrupt for reading GPS data. Don't change this...
  char c = GPS.read();
  #ifdef UDR0
    if (GPSECHO)
      if (c) UDR0 = c;
  #endif
}

void useInterrupt(boolean v) { // Interrupt for reading GPS data. Don't change this...
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

ISR(TIMER4_OVF_vect) { // This function is called every 1 second ....
  sei();                    // set interrupt flag ********VERY IMPORTANT******* you need to set the interrupt flag or programm will stuck here!!!
  TCNT4  = 336;             //   re-initialize timer value
  GPSRead();                //   read GPS data
}

void GPSRead() {
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }
  if (GPS.fix) {
    lat = GPS.latitudeDegrees;
    lon = GPS.longitudeDegrees;

    double carX = lon;
    double carY = lat;

    // Find the coordinates of the perpendicular points on each boundary wall that relate to the location of the car
    //  _____0_____
    //  \         /
    // 5 \       / 1
    //    \     /
    //     |   |
    //   4 |   | 2
    //     |___|
    //       3
    Coordinate nearestPointToEdge[6];
    nearestPointToEdge[0] = CalculateNearestPointToEdge(carX, carY, lonPoint1, latPoint1, lonPoint2, latPoint2);
    nearestPointToEdge[1] = CalculateNearestPointToEdge(carX, carY, lonPoint2, latPoint2, lonPoint3, latPoint3);
    nearestPointToEdge[2] = CalculateNearestPointToEdge(carX, carY, lonPoint3, latPoint3, lonPoint4, latPoint4);
    nearestPointToEdge[3] = CalculateNearestPointToEdge(carX, carY, lonPoint4, latPoint4, lonPoint5, latPoint5);
    nearestPointToEdge[4] = CalculateNearestPointToEdge(carX, carY, lonPoint5, latPoint5, lonPoint6, latPoint6);
    nearestPointToEdge[5] = CalculateNearestPointToEdge(carX, carY, lonPoint6, latPoint6, lonPoint1, latPoint1);

    // Get distance to each valid coordinate calculated above
    double distanceToEdge[6];
    int shortestDistanceIndex = 0;  // Keep track of shortest distance index
    for (int i = 0; i < 6; i++) {
      if (nearestPointToEdge[i].x != -1) {
        distanceToEdge[i] = CalculateDistanceFromPoint(carX, carY, nearestPointToEdge[i].x, nearestPointToEdge[i].y);
        Serial.print(i); Serial.print(" - "); Serial.print(distanceToEdge[i]); Serial.print(" - lon "); Serial.print((float)lon, 10); Serial.print(" - lat "); Serial.println(lat, 10);
        
        if (distanceToEdge[i] < distanceToEdge[shortestDistanceIndex]) {
          shortestDistanceIndex = i;
        }
      } else {
        distanceToEdge[i] = -1;
      }
      //Serial.print(i); Serial.print(" - "); Serial.print(distanceToEdge[i]); Serial.print(" - lon "); Serial.print((float)lon, 5); Serial.print(" - lat "); Serial.println(lat, 5);
    }

    // If car is within threshold distance, change ref
    if (distanceToEdge[shortestDistanceIndex] < threshold) {
      int shortestDistanceIndex2 = -1;
      for (int i = 0; i < 6; i++) {
        if (distanceToEdge[i] != -1 && i != shortestDistanceIndex) {
          if (distanceToEdge[i] < (distanceToEdge[shortestDistanceIndex] + threshold)) {
            shortestDistanceIndex2 = i;
          }
        }
      }
      
      if (shortestDistanceIndex2 == -1) {
        // Appraoching only a wall
        ref = calculateBearing(nearestPointToEdge[shortestDistanceIndex].x, nearestPointToEdge[shortestDistanceIndex].y, lon, lat);
      } else {
        // Appraoching a corner
        Coordinate newRefCoords;
        newRefCoords.x = (nearestPointToEdge[shortestDistanceIndex].x + nearestPointToEdge[shortestDistanceIndex2].x) / 2;
        newRefCoords.y = (nearestPointToEdge[shortestDistanceIndex].y + nearestPointToEdge[shortestDistanceIndex2].y) / 2;

        ref = calculateBearing(newRefCoords.x, newRefCoords.y, nearestPointToEdge[shortestDistanceIndex].x, nearestPointToEdge[shortestDistanceIndex].y);
      }
    }
  }
}

Coordinate CalculateNearestPointToEdge(double carX, double  carY, double  x1, double  y1, double  x2, double y2) {
  /*  Description: Function calculates the closest point on the boundary edge that the car is adjacent to.
   *   
   *   Parameters:
   *   carX: Current longitude coordinate of the car.
   *   carY: Current latitude coordinate of the car.
   *   (x1, y1): Coordinates of one boundary point.
   *   (x2, y2): Coordinates of another boundary point.
   *   
   *  Return: Function returns the coordinate of the closest point to the car from the specified boundary. If the
   *  coordinate is not valid (i.e. boundary does not extend that far), then returns -1 for x and y coordinates.
  */
  double intersection_X;
  double intersection_Y;

  if (abs(y2 - y1) <= 0.00001) {
    // Vertical slope
    intersection_X = carX;
    intersection_Y = y2;
  } else if (abs(x2 - x1) <= 0.00001) {
    // Horizontal slope
    intersection_X = x2;
    intersection_Y = carY;
  } else {
    // Other slope
    double slope_1 = (y2 - y1) / (x2 - x1);
    double yIntercept_1 = y2 - (slope_1 * x2);
  
    double slope_2 = -(1 / slope_1);
    double yIntercept_2 = carY - (slope_2 * carX);
  
    intersection_X = (yIntercept_1 - yIntercept_2) / (slope_2 - slope_1);
    intersection_Y = slope_2*intersection_X + yIntercept_2;
  }
  
  double bigX, littleX, bigY, littleY;
  
  if (x1 >= x2) {
    bigX = x1;
    littleX = x2;
  } else {
    bigX = x2;
    littleX = x1;
  }

  if (y1 >= y2) {
    bigY = y1;
    littleY = y2;
  } else {
    bigY = y2;
    littleY = y1;
  }

  Coordinate coord;
  coord.x = -1;
  coord.y = -1;

  if (intersection_X <= bigX && intersection_X >= littleX) {
    if (intersection_Y <= bigY && intersection_Y >= littleY) {
      coord.x = intersection_X;
      coord.y = intersection_Y;
    }
  }

  return coord;
}

double CalculateDistanceFromPoint(double carX, double carY, double x1, double y1) {
  /*  Description: Function will calculate the distance between car and a specified point.
   *   
   *  Return: Function returns the distance from specified point.
  */
  double distance;
  double r = 6371000;                          // Earth's radius in meters
  double lat_rad = carY * PI / 180;
  double lon_rad = carX * PI / 180;
  double latDst_rad = y1 * PI / 180;
  double lonDst_rad = x1 * PI / 180;

  double deltaLat = latDst_rad - lat_rad;      // Change in latitude
  double deltaLon = lonDst_rad - lon_rad;      // Change in longitude

  double a = sin(deltaLat / 2) * sin(deltaLat / 2) + cos(lat_rad) * cos(latDst_rad) * sin(deltaLon / 2) * sin(deltaLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return r * c;                           // in meters
}

double calculateBearing(double x1, double y1, double x2, double y2) {
  double deltaLat = y2 - y1;
  double deltaLon = x2 - x1;
  
  double Bearing = (atan2(deltaLat, deltaLon) * 180/ PI) - 90; // OUTPUT: 0° is NORTH, -90° is EAST, -180° is SOUTH, 90/-270° is WEST.

  if (Bearing < -180) {
    Bearing += 360;                           // OUTPUT: 0° is NORTH, -90° is EAST, +/-180° is SOUTH, 90° is WEST.
  }

  Bearing = Bearing * -1;                     // FINAL OUTPUT: 0° is NORTH, 90° is EAST, +/-180° is SOUTH, -90° is WEST.

  return Bearing;
}

void ReadHeading() {
  // Read Heading from BNO055 sensor
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  HEADING = euler.x();                              // Grab euler direction output

  HEADING -= errorHeadingRef;                       // Factors in error in HEADING for Tempe.

  // Corrects any values which exceed 180 degrees.
  if (HEADING > 180) HEADING -= 360;

  // FINAL OUTPUT: 0° is NORTH, 90° is EAST, +/-180° is SOUTH, -90° is WEST.
}

void CalculateSteering() { // Calculate the steering angle according to the reference heading and actual heading.
  int leftBound, rightBound, backBound;       // Create variables to determine the bounds of <ref>. Range is (-150)-180.

  if (ref < -90) {                            // Between -180 and -90 (does not include -180 or -90). Given that ref will be in increments
    // of 30 degrees, ref will specifically either be (-150) degrees or (-120) degrees.

    // Alter the bounds to accomodate for these angles.
    leftBound = ref + 270;
    rightBound = ref + 90;
    backBound = ref + 180;

    if (HEADING <= rightBound) {                                                          //     Change the angle of steering by an increment of 10
      STEERANGLE = (int)((rightBound - HEADING) / steerPrecision) * steerPrecision;       // degrees, so as to accurately drive in the "desired
    } else if (HEADING >= leftBound) {                                                    // direction."
      STEERANGLE = (int)((rightBound - HEADING + 360) / steerPrecision) * steerPrecision;

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
      STEERANGLE = (int)((rightBound - HEADING) / steerPrecision) * steerPrecision;       // change the angle of steering by an increment of 10 degrees,
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
      STEERANGLE = (int)((rightBound - HEADING) / steerPrecision) * steerPrecision;       // the angle of steering by an increment of 10 degrees, so as to
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
      STEERANGLE = (int)((rightBound - HEADING + 360) / steerPrecision) * steerPrecision; //  so as to accurately drive in the "desired direction".
    } else if (HEADING <= rightBound) {
      STEERANGLE = (int)((rightBound - HEADING) / steerPrecision) * steerPrecision;

    } else if (HEADING > rightBound && HEADING <= backBound) {                            //     If the car is outside the right bound and the back bound,
      STEERANGLE = 0;                                                                     // start turning completely left until the next magnetometer
      // reading.

    } else {                                                                              //     Else, the car starts turning completely right until the next
      STEERANGLE = 180;                                                                   // magnetometer reading.
    }
  }

  if (STEERANGLE > 135) STEERANGLE = 135;     // The car cannot turn its wheels more than 135 degrees, so cap it here.
  if (STEERANGLE < 45) STEERANGLE = 45;       // The car canno turn its wheels less than 45 degrees, so cap it here.

  //Serial.print("Steer Angle: "); Serial.println(STEERANGLE);// Serial.print(" | ");       // Print the steering angle to the Serial Monitor.
}

void Actuate() { // Input: Steering angle - Output: nothing
  myservo.write(STEERANGLE);
  analogWrite(carSpeedPin, carSpeed);
}

ISR(TIMER1_OVF_vect) {        // This function is called every 0.1 seconds
  sei();                      // set interrupt flag ********VERY IMPORTANT******* you need to set the interrupt flag or programm will stuck here!!!
  TCNT1  = 59016;
  ReadHeading();              // Read Heading
  CalculateSteering();        // Calculate Steer angle
  Actuate();                  // Actuate
}

void printLocationOnLCD() {
  lcd.print("Lat: ");
  lcd.print(lat, 5);
  lcd.setCursor(0, 1);
  lcd.print("Lon: ");
  lcd.print(lon, 5);
}

void loop() {
  lcd.clear();
  printLocationOnLCD();
  delay(100);
}
