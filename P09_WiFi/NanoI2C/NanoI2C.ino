/* File: NanoI2C.ino
 *  
 *  Description: Program uses UART to communicate with LIDAR and
 *  I2C to send LIDAR data to Arduino Mega.
 *  
 *  Authors:  Alex Shearer, Erin Hintze, Robert Alimov
 *  Group:    Group 6
 */

#include <Wire.h>                 // Arduino library used to communicate via I2C
#include <RPLidar.h>              // Library for communicating with LIDAR

#define RPLIDAR_MOTOR 3           // motor pin for lidar speed control (D3 on Nano Board)
#define LIDAR_DISTANCE 2500       // Distance threshold to keep value (millimeters)
#define LIDAR_MAX_DISTANCE 6000   // Maximum distance Lidar can detect (millimeters)
#define LIDAR_PERIOD 400

#define ANGLE_LEFT0 330
#define ANGLE_LEFT1 300
#define ANGLE_LEFT2 270

#define ANGLE_RIGHT0 30
#define ANGLE_RIGHT1 60
#define ANGLE_RIGHT2 90

RPLidar lidar;                    // define lidar as RPLIDAR Object

uint8_t left0  = 0;               // variable for detected points on left hand side within 2.5 meters (331-359)
uint8_t left1  = 0;               // variable for detected points on left hand side within 2.5 meters (301-330)
uint8_t left2  = 0;               // variable for detected points on left hand side within 2.5 meters (271-300)

uint8_t right0 = 0;               // variable for detected points on right hand side within 2.5 meters (0-29 degrees)
uint8_t right1 = 0;               // variable for detected points on right hand side within 2.5 meters (30-59 degrees)
uint8_t right2 = 0;               // variable for detected points on right hand side within 2.5 meters (60-89 degrees)

uint8_t nearby = 0;               // variable for detected points within 6 meters

unsigned long time = millis();    // time variable for resetting variables
uint8_t c1;                       // Variable for received byte from the I2C Bus

/*
 *         LIDAR_DISTANCE
 *           ---------
 * 360-FOV/2 *   0°  * FOV/2
 *            \     /
 *             \FOV/
 *              \ /
 *   270° ------ * Lidar -- 90°
 *               |
 *               |
 *              180°
 */

void setup() {
  pinMode(RPLIDAR_MOTOR, OUTPUT); // set lidar speed control pin as output
  lidar.begin(Serial);            // begin communication with lidar
  Wire.begin(8);                  // join `i2c bus with address 8 as SLAVE
  Wire.onRequest(requestEvent);   // call "requestEvent" function when receives a request
  Wire.onReceive(receiveEvent);   // call "receiveEvent" function when receives a byte
}

void receiveEvent(int bytes) {
  // Read the received byte as integer. This indicates what data to send back when master is requesting data
  while (Wire.available()) {
    // Read something into c1
    c1 = Wire.read();
  }
}

void requestEvent() {
  // receive message byte as a character
  // 0 = left2
  // 1 = left1
  // 2 = left0
  // 3 = right0
  // 4 = right1
  // 5 = right2
  // 6 = nearby
  byte sendArr;
  if (c1 == 0) {
    sendArr = left2 & 0xFF;
  } else if (c1 == 1) {
    sendArr = left1 & 0xFF;
  } else if (c1 == 2) {
    sendArr = left0 & 0xFF;
  } else if (c1 == 3) {
    sendArr = right0 & 0xFF;
  } else if (c1 == 4) {
    sendArr = right1 & 0xFF;
  } else if (c1 == 5) {
    sendArr = right2 & 0xFF;
  } else if (c1 == 6) {
    sendArr = nearby & 0xFF;
  }
  Wire.write(sendArr);
}

void loop() {
  if (IS_OK(lidar.waitPoint())) { // if lidar is working properly (waiting time less than timeout)
    // read angle and distance of the obstacle
    // filter data (keep only the data in desired range and with desired angle)
    // COUNT the number of obstacles on LEFT and RIGHT side
    // reset obstacle variables every 1 second
    float distance = lidar.getCurrentPoint().distance; // Millimeters
    float angle = lidar.getCurrentPoint().angle; // Degrees - 0 is straight, 90 is right, 180 is rear, 270 is left

    if (lidar.getCurrentPoint().startBit) {
      // A new scan is happening, wait for scan to finish
      // If distance or angle were read now, it would be the previous values since they haven't updated yet.
    } else {
      if (distance > 0) {
        if (distance < LIDAR_MAX_DISTANCE) {
          if (distance < LIDAR_DISTANCE) {

            if (angle > ANGLE_LEFT0) {
              left0++;
            } else if (angle > ANGLE_LEFT1) {
              left1++;
            } else if (angle > ANGLE_LEFT2) {
              left2++;
            }

            if (angle < ANGLE_RIGHT0) {
              right0++;
            } else if (angle < ANGLE_RIGHT1) {
              right1++;
            } else if (angle < ANGLE_RIGHT2) {
              right2++;
            }
            
          } // distance < LIDAR_DISTANCE

          if (angle > ANGLE_LEFT2 || angle < ANGLE_RIGHT2) {
            nearby++;
          }
        } // distance < LIDAR_MAX_DISTANCE
      } // distance > 0
    } // else

    unsigned long newTime = millis();
    if ((newTime-time) >= LIDAR_PERIOD) {
      // Reset data values every 200ms
      left0 = 0;
      left1 = 0;
      left2 = 0;
      
      right0 = 0;
      right1 = 0;
      right2 = 0;

      nearby = 0;
      
      time = newTime;
    }
  } else {
    // If LIDAR is not responding, stop the lidar spinning motor, try to detect it, and start start the LIDAR again
    analogWrite(RPLIDAR_MOTOR, 0);                          //stop the rplidar motor                // Dont change this......
    rplidar_response_device_info_t info;                    // try to detect RPLIDAR...             // Dont change this......
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {            // if detected,                         // Dont change this......
      lidar.startScan();                                    // start scan                           // Dont change this......
      analogWrite(RPLIDAR_MOTOR, 255);                      // start motor rotating at max speed    // Dont change this......
      delay(1000);                                                                                  // Dont change this......
    }
  }
}
