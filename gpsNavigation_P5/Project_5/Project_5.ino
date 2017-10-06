#include <Adafruit_BNO055.h>
#include <FlexiTimer2.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_GPS.h>
#include "DFR_Key.h"
//#include <math.h>


Adafruit_GPS GPS(&Serial1);                   // define GPS object connected to Serial 3
DFR_Key keypad;  
Servo myservo;                                    // define servo object
Adafruit_BNO055 bno = Adafruit_BNO055(55);        // define BNO sensor object
LiquidCrystal lcd( 8, 9, 4, 5, 6, 7);             // define lcd pins use these default values for OUR LCD

#define GPSECHO  false

// Global variables that are changed across functions
int STEERANGLE = 90;                              // servo initial angle (range is 0:180)
float HEADING = 0;                                // heading
boolean usingInterrupt = false;
int carSpeedPin = 2;                              // pin for DC motor (PWM for motor driver)
float errorHeadingBearing = 10.37;                        // error
long int lat;                                     // GPS latitude in degree decimal multiplied by 100000
long int lon;                                     // GPS latitude in degree decimal multiplied by 100000
long int latDestination = 33.423933 * 100000;     // Bearingerence destination
long int lonDestination = -111.939585 * 100000;   // Bearingerence destination
float Bearing = 0;                                // bearing angle to destination
int localkey = 0;                                 // variable for keypad
int steerPrecision = 10;                      // Variable indicates the step size (in degrees) between steering angles.
int carSpeed = 0;
int lastServoVal  = 0;
int lastCarSpeed = 0;
float distance = 10;

void setup() {
    myservo.attach(44);                             // servo is connected to pin 44
    lcd.begin( 16, 2 );                             // LCD type is 16x2 (col & row)
    Serial.begin(9600);                             // serial for monitoring

    Serial.println("Orientation Sensor Calibration"); Serial.println("");
    if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {                               //     If you want to calibrate using another mode, set it here.
  	Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");       // OPERATION_MODE_COMPASS for a precise tilt compensated compass
  	while (1);                                                                          // (Section 3.3.2 / 3.3.3)
    }

    ///Setting the Bearingerence (Lat and Lon)///
    localkey = 0;
    while (localkey != 1) {                         // wait for select button
  	lcd.clear();                                  // clear the display
  	localkey = keypad.getKey();
  	lcd.print("Press Select");                    // first press of the button will save the GPS location
  	lcd.setCursor(0, 1);
  	lcd.print("to save dest.");
  	delay(100);                                   // delay to make display visible
    }
    ReadGPS();
    latDestination = lat;                           // saving the destination point (latitude)
    lonDestination = lon;                           // saving the destination point (longitude)
    localkey = 0;
    while (localkey != 1) {                         // wait for select button
  	lcd.clear();
  	localkey = keypad.getKey();
  	lcd.print("Press Select");                    // after button is pressed once, notify user that next press will start car
  	lcd.setCursor(0, 1);
  	lcd.print("to drive!");
  	delay(100);                                   // delay to make display visible
    }

    byte c_data[22] = {0, 0, 0, 0, 0, 0, 68, 255, 50, 2, 207, 0, 255, 255, 255, 255, 1, 0, 232, 3, 159, 3};
    bno.setCalibData(c_data);                                                                                       // SET CALIBRATION DATA
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


    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);      // 1 Hz update rate because it's more stable than 10Hz
    GPS.sendCommand(PGCMD_ANTENNA);
    useInterrupt(true);
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

ISR(TIMER4_OVF_vect) { // This function will be called every 1 second
    sei();        //   set interrupt flag // don't change this
    TCNT4  = 336; //   re-initialize timer4's value
    ReadGPS();    //   read GPS data
}

void ReadGPS() {
    // read from GPS module and update the current position
    if(GPS.newNMEAreceived()){
      GPS.parse(GPS.lastNMEA());
    }
    if(GPS.fix){
      lat = GPS.latitude;
      lon = GPS.longitude;
    }
}

void ReadHeading() {

    imu::Vector<3> rawMag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);     // Read the current data of the car via the Magnetometer.
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
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

    //  Serial.print("  Gyr: <");
    //  Serial.print(rawGyr.x()); Serial.print(", ");
    //  Serial.print(rawGyr.y()); Serial.print(", ");
    //  Serial.print(rawGyr.z()); Serial.println(">");

    //HEADING = (atan2(rawMag.y(), rawMag.x()) * 180 / PI); // This output makes North = 90 degrees, it will be corrected to 0 degrees.

    HEADING = euler.x();
    Serial.println(HEADING);
    HEADING -= errorHeadingBearing;                          // Adds error in HEADING for Tempe (10.37 degrees). 

    // Corrects any values which exceed 180 degrees.
    if (HEADING > 180) HEADING -= 360;                   // FINAL OUTPUT: 0° is NORTH, 90° is EAST, +/-180° is SOUTH, -90° is WEST.

    Serial.print("Angle: "); Serial.print(HEADING); Serial.print(" | ");              // Print HEADING to the Serial Monitor.
}

void CalculateBearing() {
    // calculate bearing angle based on current and destination locations (GPS coordinates)
    Bearing = atan2(latDestination - lat, lonDestination - lon);
}

void CalculateSteering() {                    // Calculate the steering angle according to the Bearingerence heading and actual heading.
    int leftBound, rightBound, backBound;       // Create variables to determine the bounds of <Bearing>. Range is (-150)-180.

    if (Bearing < -90) {                            // Between -180 and -90 (does not include -180 or -90). Given that Bearing will be in increments
	// of 30 degrees, Bearing will specifically either be (-150) degrees or (-120) degrees.

	// Alter the bounds to accomodate for these angles.
	leftBound = Bearing + 270;
	rightBound = Bearing + 90;
	backBound = Bearing + 180;

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

    } else if (Bearing < 0) {                       //     Between -90 and 0 (includes -90 but not 0). Given that Bearing will be in increments of 30 degrees,
	//  Bearing will specifically either be (-90, -60, -30) degrees.

	// Alter the bounds to accomodate for these angles.
	leftBound = Bearing - 90;
	rightBound = Bearing + 90;
	backBound = Bearing + 180;

	if (HEADING >= leftBound && HEADING <= rightBound) {                                  //     If the car is between the left and right bounds,
	    STEERANGLE = (int)((rightBound - HEADING)/steerPrecision) * steerPrecision;         // change the angle of steering by an increment of 10 degrees,
	    // so as to accurately drive in the "desired direction".

	} else if (HEADING <= backBound && HEADING >= rightBound) {                           //     If the car is outside the back and right bounds, start
	    STEERANGLE = 0;                                                                     // turning completely left until the next magnetometer reading.

	} else {                                                                              //     Else, the car starts turning completely right until the
	    STEERANGLE = 180;                                                                   // next magnetometer reading.
	}

    } else if (Bearing <= 90) {                     //     Between 0 and 90 (includes 0 and 90). Given that Bearing will be in increments of 30 degrees, Bearing will
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

    } else if (Bearing <= 180) {                    //     Between 90 and 180 (includes 180 but not 90). Given that Bearing will be in increments of 30 degrees, Bearing
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

//Haversine formula
void CalculateDistance() {
    // calculate distance to destination based on current and destination coordinates
    auto r = 6371000;
    auto x1 = lat * 180 / M_PI;
    auto x2 = latDestination * 180 / M_PI;
    auto deltax = (latDestination - lat) * 180 / M_PI;
    auto deltay = (lonDestination - lon) * 180 / M_PI;

    auto a = sin(deltax/2) * sin(deltax/2) + cos(x1) * cos(x2) + sin(deltay/2) * sin(deltay/2);
    auto c = 2 * atan2(sqrt(a), sqrt(1-a));
    distance = r * c;
}

void Actuate() {                              // Input: Steering angle - Output: nothing
    carSpeed = 255*0.1;
    if(lastServoVal != STEERANGLE){
	    myservo.write(STEERANGLE);
	    lastServoVal = STEERANGLE;
    }
    if(lastCarSpeed != carSpeed) {
	    analogWrite(carSpeedPin, carSpeed);
	    lastCarSpeed = carSpeed;
    }
}

ISR(TIMER1_OVF_vect) {                            // This function will be called every 0.1 second
    sei();                                          // set interrupt flag // don't change this
    TCNT1  = 59016;                                 // reinitialize the timer1's value
    ReadHeading();                                  // read heading
    CalculateBearing();                             // calc bearing
    CalculateSteering();                            // calc steering
    CalculateDistance();                            // calc distance
    Actuate();                                      // Actuate
}

void printLocationOnLCD() {
    lcd.print("Latitude: ");
    lcd.print(lat);
    lcd.setCursor(0,1);
    lcd.print("Longitude: ");
    lcd.print(lon);
}

void loop() {
    lcd.clear();    // clear the LCD
    // You can print anything on the LCD to debug your program!!!
    printLocationOnLCD();
    delay(100);
}
