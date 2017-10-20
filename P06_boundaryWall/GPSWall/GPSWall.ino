#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
#include "DFR_Key.h"

Adafruit_GPS GPS(&Serial3);                   // define GPS object
Servo myservo;                                // define servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);    // define BNO sensor object
LiquidCrystal lcd( 8, 9, 4, 5, 6, 7);         // define lcd pins use these default values for OUR LCD
DFR_Key keypad;                               // Define keypad object. This will be used be a TA to input the desired direction.

#define GPSECHO  false
#define lThreshold 5                          // Lidar Threshold
#define dThreshold 10                         // GPS Wall Threshold

// Global variables that change across functions

float Bearing = 0;
int STEERANGLE = 90;                            // servo initial angle (range is 0:180)
float HEADING = 0;                              // heading
boolean usingInterrupt = false;                 // Using interrupt for reading GPS chars
int carSpeedPin = 2;                            // pin for DC motor (PWM for motor driver)
float errorHeadingRef = 10.37;                  // Heading error
long int lat = 33.420887 * 100000;              // GPS latitude in degree decimal * 100000 (CURRENT POSITION)
long int lon = -111.934089 * 100000;            // GPS latitude in degree decimal * 100000 (CURRENT POSITION)
long int latDestination = 33.421620 * 100000;   // reference destination (INITIAL DESTINATION)
long int lonDestination = -111.930118 * 100000; // reference destination (INITIAL DESTINATION)
double threshhold = 3;				                  // Only correct if within 3m of wall
int localkey = 0;                               // Variable for reading the keypad value
int ref = 0;                                    // Reference angle, set via keypad.
int steerPrecision = 10;                        // Variable indicates the step size (in degrees) between steering angles.
int carSpeed = 0;



///////////////////////////////////////// Boundary points  //////////////////////////////////////////
long int latPoint1 = 33.4218461 * 100000;     // reference destination (Point1)
long int lonPoint1 =  -111.934683 * 100000;   // reference destination (Point1)

long int latPoint2 = 33.421846 * 100000;     // reference destination (Point2)
long int lonPoint2 =  -111.933382 * 100000;   // reference destination (Point2)

long int latPoint3 = 33.421147 * 100000;     // reference destination (Point3)
long int lonPoint3 =  -111.9337721 * 100000;   // reference destination (Point3)

long int latPoint4 = 33.420825 * 100000;     // reference destination (Point4)
long int lonPoint4 =  -111.933824 * 100000;   // reference destination (Point4)

long int latPoint5 = 33.4208231 * 100000;     // reference destination (Point5)
long int lonPoint5 =  -111.9341681 * 100000;   // reference destination (Point5)

long int latPoint6 = 33.421151 * 100000;     // reference destination (Point6)
long int lonPoint6 =  -111.934220 * 100000;   // reference destination (Point6)
/////////////////////////////////////////////////////////////////////////////////////////////////////
struct point {
  long int lon;
  long int lat;
};

typedef struct point Point;
Point endpoints[] = {{lonPoint1, latPoint1}, {lonPoint2, latPoint2}, {lonPoint3, latPoint3}, {lonPoint4, latPoint4}, {lonPoint5, latPoint5}, {lonPoint6, latPoint6}, {lonPoint1, latPoint1}};

void setup() {
  myservo.attach(44);                         // servo is connected to pin 44
  lcd.begin( 16, 2 );                         // LCD type is 16x2 (col & row)
  Serial.begin(9600);                         // serial for monitoring

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

SIGNAL(TIMER0_COMPA_vect) {                   // Interrupt for reading GPS data. Don't change this...
  char c = GPS.read();
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
#endif
}

void useInterrupt(boolean v) {                // Interrupt for reading GPS data. Don't change this...
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

ISR(TIMER4_OVF_vect) {      // This function is called every 1 second ....
  sei();                    // set interrupt flag ********VERY IMPORTANT******* you need to set the interrupt flag or programm will stuck here!!!
  TCNT4  = 336;             //   re-initialize timer value
  GPSRead();                //   read GPS data
}

void GPSRead() {
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  if (GPS.fix) {
    lat = GPS.latitude;
    lon = GPS.longitude;

    // Create a grid by denoting negative Quadrants.
    if (GPS.lon == 'W') {
      lon = lon * -1;
    }
    if (GPS.lat == 'S') {
      lat = lat * -1;
    }

    //At this point, we have new values for latitude and longitude
    // Find which walls

    // Calculate distance from each Wall (call CalculateDistancePerpendicular(); )
    // if distance is less than the threshold
    // compute the direction vector X (call CalculateDirectionPerpendicularX(); )
    // compute the direction vector Y (call CalculateDirectionPerpendicularY(); )
    // Set a new Destination according to direction vectors X & Y

    //6 should be #defined how many lines but im lazy
    double* distanceToLines = new double[6];
    distanceToLines[0] = CalculateDistanceFromPerpendicular(lon, lat, lonPoint1, latPoint1, lonPoint2, latPoint2);
    distanceToLines[1] = CalculateDistanceFromPerpendicular(lon, lat, lonPoint2, latPoint2, lonPoint3, latPoint3);
    distanceToLines[2] = CalculateDistanceFromPerpendicular(lon, lat, lonPoint3, latPoint3, lonPoint4, latPoint4);
    distanceToLines[3] = CalculateDistanceFromPerpendicular(lon, lat, lonPoint4, latPoint4, lonPoint5, latPoint5);
    distanceToLines[4] = CalculateDistanceFromPerpendicular(lon, lat, lonPoint5, latPoint5, lonPoint6, latPoint6);
    distanceToLines[5] = CalculateDistanceFromPerpendicular(lon, lat, lonPoint6, latPoint6, lonPoint1, latPoint1);
    int indexes[] = { -1, -1};
    int size = 0;
    for (int i = 0; i < 6; i++) {
      if (distanceToLines[i] < threshhold) {
        indexes[size] = i;
        size++;
      }
    }
    int x = 0;
    int y = 0;
    for (int i = 0; i < size; i++) {
      x += CalculateDirectionPerpendicularX(lon, lat, endpoints[indexes[i]].lon, endpoints[indexes[i]].lat, endpoints[indexes[i] + 1].lon, endpoints[indexes[i] + 1].lat);
      y += CalculateDirectionPerpendicularY(lon, lat, endpoints[indexes[i]].lon, endpoints[indexes[i]].lat, endpoints[indexes[i] + 1].lon, endpoints[indexes[i] + 1].lat);
    }
    //Sanitize
    //We update ref to drive to
    ref = atan2(y, x)*180/PI;
  }
}
double CalculateDirectionPerpendicularX(double x1, double  y1, double  x2, double  y2, double  x3, double y3) {     // Function to Calculate Horizental vector ---INPUTs:( Current x, Current y, Point i (x). Point i (y), Point j (x), Point j (y) )
  double Dx;
  double m = (y3 - y2) / (x3 - x2); //Slope of line
  double k = y2 - m * x2;      //yint
  Dx = (x1 + m * y1 - m * k) / (m * m + 1);
  return Dx - x1;                        // The output of this function is the x component of the force vector from wall defined by (x2,y2)--(x3,y3)
}

double CalculateDirectionPerpendicularY(double x1, double  y1, double  x2, double  y2, double  x3, double y3) {     // Function to Calculate Vertical vector   ---INPUTs:( Current x, Current y, Point i (x). Point i (y), Point j (x), Point j (y) )
  double Dy;
  double m = (y3 - y2) / (x3 - x2); //Slope of line
  double k = y2 - m * x2;      //yint
  Dy = (x1 + m * y1 - m * k) / (m * m + 1) + k;
  return Dy - y1;                       // The output of this function is the x component of the force vector from wall defined by (x2,y2)--(x3,y3)
}

double CalculateDistanceFromPerpendicular(double x1, double  y1, double  x2, double  y2, double  x3, double y3) {
  /*  Description: Function calculates
      Return: Function returns distance from a wall, or -1 if invalid
  */
  double distance;
  double m = (y3 - y2) / (x3 - x2);
  double k = y2 - m * x2;
  distance = (abs(k + m * x1 - y1)) / (sqrt(1 + m * m));
  return distance;
}

double CalculateDistanceFromPoint(double lon, double lat, double lonDestination, double latDestination) {
  /*  Description: Function will calculate the distance between car and a specified point.
      Returns the distance from specified point.
  */
  double distance;
  float r = 6371000;                          // Earth's radius in meters
  float lat_rad = lat / 100 * PI / 180;
  float lon_rad = lon / 100 * PI / 180;
  float latDst_rad = latDestination / 100 * PI / 180;
  float lonDst_rad = lonDestination / 100 * PI / 180;

  float deltaLat = latDst_rad - lat_rad;      // Change in latitude
  float deltaLon = lonDst_rad - lon_rad;      // Change in longitude

  float a = sin(deltaLat / 2) * sin(deltaLat / 2) + cos(lat_rad) * cos(latDst_rad) * sin(deltaLon / 2) * sin(deltaLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  distance = r * c;                           // in meters
  return r * c;
}

void ReadHeading() {
  // Read Heading from BNO055 sensor

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  HEADING = euler.x();                                  // Grab euler direction output

  HEADING -= errorHeadingRef;                       // Factors in error in HEADING for Tempe.

  // Corrects any values which exceed 180 degrees.
  if (HEADING > 180) HEADING -= 360;

  // FINAL OUTPUT: 0° is NORTH, 90° is EAST, +/-180° is SOUTH, -90° is WEST.
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

  Serial.print("Steer Angle: "); Serial.println(STEERANGLE);// Serial.print(" | ");       // Print the steering angle to the Serial Monitor.
}

void Actuate() {                              // Input: Steering angle - Output: nothing
      myservo.write(STEERANGLE);
      analogWrite(carSpeedPin, carSpeed);
}

ISR(TIMER1_OVF_vect) {        // This function is called every 0.1 seconds
  sei();                      // set interrupt flag ********VERY IMPORTANT******* you need to set the interrupt flag or programm will stuck here!!!
  TCNT1  = 59016;
  ReadHeading();                                                                // Read Heading
  CalculateSteering();                                                         // Calculate Steer angle
  Actuate();                                                                   //Actuate
}

void printLocationOnLCD() {
  lcd.print("Lat: ");
  lcd.print(lat, 5);
  lcd.setCursor(0, 1);
  lcd.print("Lon: ");
  lcd.print(lon, 5);

}

//Print distance from line i clockwise(facing north) starting at University
void printDistanceFrom(int i){
  lcd.print(CalculateDistanceFromPerpendicular(lon, lat, endpoints[i].lon, endpoints[i].lat, endpoints[i + 1].lon, endpoints[i + 1].lat));
}

//Same indexing as PrintDistanceFrom
void printForceVectorsFrom(int i){
  lcd.print("X: ");
  lcd.print(CalculateDirectionPerpendicularX(lon, lat, endpoints[i].lon, endpoints[i].lat, endpoints[i + 1].lon, endpoints[i + 1].lat), 3);
  lcd.setCursor(0,1);
  lcd.print("Y: ");
  lcd.print(CalculateDirectionPerpendicularY(lon, lat, endpoints[i].lon, endpoints[i].lat, endpoints[i + 1].lon, endpoints[i + 1].lat), 3);
}

void loop() {
  lcd.clear();      // clear LCD
  // you can pring anything on the LCD to debug your program while you're in the field!
  //printLocationOnLCD();
  printDistanceFrom(5);
  delay(100);
}




