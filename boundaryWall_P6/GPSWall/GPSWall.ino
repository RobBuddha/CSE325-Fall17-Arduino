#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_GPS.h>

Adafruit_GPS GPS(&Serial3);                   // define GPS object
Servo myservo;                                // define servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);    // define BNO sensor object
LiquidCrystal lcd( 8, 9, 4, 5, 6, 7);         // define lcd pins use these default values for OUR LCD

#define GPSECHO  false
#define lThreshold 5                          // Lidar Threshold
#define dThreshold 10                         // GPS Wall Threshold

// Global variables that change across functions

float Bearing = 0;
int STEERANGLE = 90;                            // servo initial angle (range is 0:180)
float HEADING = 0;                              // heading
boolean usingInterrupt = false;                 // Using interrupt for reading GPS chars
int carSpeedPin = 2;                            // pin for DC motor (PWM for motor driver)
float errorHeadingRef = 0;                      // Heading error
long int lat = 33.420887 * 100000;              // GPS latitude in degree decimal * 100000 (CURRENT POSITION)
long int lon = -111.934089 * 100000;            // GPS latitude in degree decimal * 100000 (CURRENT POSITION)
long int latDestination = 33.421620 * 100000;   // reference destination (INITIAL DESTINATION)
long int lonDestination = -111.930118 * 100000; // reference destination (INITIAL DESTINATION)
double threshhold = 3;				// Only correct if within 3m of wall

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
    for(int i = 0;i< size;i++){
      x += CalculateDirectionPerpendicularX(lon, lat, endpoints[indexes[i]].lon, endpoints[indexes[i]].lat, endpoints[indexes[i] + 1].lon, endpoints[indexes[i] + 1].lat);
      y += CalculateDirectionPerpendicularY(lon, lat, endpoints[indexes[i]].lon, endpoints[indexes[i]].lat, endpoints[indexes[i] + 1].lon, endpoints[indexes[i] + 1].lat);
    }
    //Sanitize
    Bearing = atan2(y, x);
  }
}
double CalculateDirectionPerpendicularX(double x1, double  y1, double  x2, double  y2, double  x3, double y3) {     // Function to Calculate Horizental vector ---INPUTs:( Current x, Current y, Point i (x). Point i (y), Point j (x), Point j (y) )
  double Dx;
  double m = (y3 - y2) / (x3 - x2); //Slope of line
  double k = y2 - m * x2;      //yint
  Dx = (x1 + m * y1 - m * k) / (m * m + 1);
  return Dx - x1;                        // The output of this function is direction along x-axis
}

double CalculateDirectionPerpendicularY(double x1, double  y1, double  x2, double  y2, double  x3, double y3) {     // Function to Calculate Vertical vector   ---INPUTs:( Current x, Current y, Point i (x). Point i (y), Point j (x), Point j (y) )
  double Dy;
  double m = (y3 - y2) / (x3 - x2); //Slope of line
  double k = y2 - m * x2;      //yint
  Dy = (x1 + m * y1 - m * k) / (m * m + 1) + k;
  return Dy - y1;                       // The output of this function is direction along y-axis
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
}

void CalculateBearing() {
  // Calculate Bearing based on current and destination coordinates
}

void CalculateSteering() {
  // calculate steering angle based on heading and bearing
}

void SetCarDirection() {
  // Set direction (actuate)
}


ISR(TIMER1_OVF_vect) {        // This function is called every 0.1 seconds
  sei();                      // set interrupt flag ********VERY IMPORTANT******* you need to set the interrupt flag or programm will stuck here!!!
  TCNT1  = 59016;
  ReadHeading();                                                                // Read Heading
  CalculateBearing();                                                       // Calculate Bearing
  CalculateSteering();                                                         // Calculate Steer angle
  SetCarDirection();                                                        // Set steer angle
}


void printHeadingOnLCD() {

}

void printLocationOnLCD() {

}



void printObstacleOnLCD() {

}

void loop() {
  lcd.clear();      // clear LCD
  // you can pring anything on the LCD to debug your program while you're in the field!
  printHeadingOnLCD();
  printLocationOnLCD();
  //  printObstacleOnLCD();
  delay(100);
}




