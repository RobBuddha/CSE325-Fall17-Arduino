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
#define LIDAR_DISTANCE 1500       // Distance threshold to keep value (millimeters)

RPLidar lidar;                    // define lidar as RPLIDAR Object
uint8_t left  = 0;                // variable for detected points on left hand side
uint8_t right = 0;                // variable for detected points on right hand side
unsigned long time = millis();    // time variable for resetting variables
uint8_t c1;                       // Variable for received byte from the I2C Bus
int lidarFOV = 36;                // Lidar Field of View Angle (degrees)

/* 360-FOV/2 *   0   * FOV/2
 *            \     /
 *             \FOV/
 *              \ /
 *               * Lidar
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
  // if master's request is right side data, ("1"), send back the left side data
  // if master's request is left  side data, ("2"), send back the right side data
  byte sendArr;
  if (c1 == 1) {
    sendArr = left & 0xFF;
  } else if (c1 == 2) {
    sendArr = right & 0xFF;
  }
  Wire.write(sendArr);
}

void loop()
{
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
      if (distance <= LIDAR_DISTANCE) {
        if (angle > (360-(lidarFOV/2))) {
          left++;
        } else if (angle < (lidarFOV/2)) {
          right++;
        }
      }
    }

    unsigned long newTime = millis();
    if ((newTime-time) >= 1000) {
      left = 0;
      right = 0;
      time = newTime;
    }
  } else {
    // If LIDAR is not responding, stop the motor, try to detect it, and start start the LIDAR again
    analogWrite(RPLIDAR_MOTOR, 0);                          //stop the rplidar motor                // Dont change this......
    rplidar_response_device_info_t info;                    // try to detect RPLIDAR...             // Dont change this......
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {            // if detected,                         // Dont change this......
      lidar.startScan();                                    // start scan                           // Dont change this......
      analogWrite(RPLIDAR_MOTOR, 255);                      // start motor rotating at max speed    // Dont change this......
      delay(1000);                                                                                  // Dont change this......
    }
  }
}
