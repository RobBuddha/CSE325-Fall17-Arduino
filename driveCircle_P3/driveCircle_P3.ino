
#include <Servo.h>

Servo carServo; // Create a Servo Object

#define pwmToMotor 4               // Defining the PWM pin to run the motor.
#define servoPin 48                // Defining the servo control pin.

void      driveCircle();           // Drive circle function initial definition to drive a circle.
void      carStop();               // Stop function initial definition.



int carSpeed = 255*0.1;            // <carSpeed> determines speed of the car by duty cycles. "255*.1" = 10%, "255*.2" = 20%.
double servoDeg = 0;               // <servoDeg> determines servo motor's degree of turning radius.

void setup()                       // Initial configurations below:
{
  carServo.attach(servoPin);       // Set the Servo motor control (pin 48).
  carServo.write(90);              // Reset to straighten the wheels.
  pinMode(pwmToMotor, OUTPUT);     // Set the output pin (pin 4).
}

void loop()                        // Loop function which will be run forever.
{
  delay(1000);                     // Arbitrarily chosen number of seconds to wait before circle begins.
  driveCircle();                   // Drive one complete circle.
  carStop();                       // Stop the car.
  delay(10000);                    // Arbitrarily chosen number of seconds to wait once the circle is completed.
                                   //     (make longer for debugging purposes).

  
  /*                               
  carSpeed = 255*.2;               // Set new value for speed (change the duty cycle of the PWM).
  driveCircle();                   // Drive another complete circle at the modified speed.
  carStop();                       // Stop the car.
  delay(10000);                    // Arbitrarily chosen number of seconds to wait once circle is complete (10 seconds).
  */
}





void driveCircle()                                  // Function to drive the car in a complete circle.
{  
  for(servoDeg = 90;servoDeg <= 180;servoDeg++){    // Angles between 90-180 degrees will turn the car to the right.
    carServo.write(servoDeg);                       // Set the wheels to the current degree.
    analogWrite(pwmToMotor, carSpeed);              // Relay the speed to the motor.


    // In an attempt to combat the inconsistent radius of the circle turn:
    // Still needing to be refined (play with the delays to perfect a constant radius).
    if(servoDeg < 45){
        delay(40);                                  // Take 40 ms to turn each of the initial 45 degrees (from 90-135).
    } else if(servoDeg < 80){
      delay(30);                                    // Take 30 ms to turn each of the next 35 degrees (from 135-170).
    } else {
      delay(50);                                    // Take 50 ms to turn the remaining 10 degrees (from 170-180).
    }
    
  }
  delay(100);                                       // Delay an additional 100 ms in attempt to get car back to its initial position.
}

void carStop()                   // Function to stop the car.
{
  delay(1000);                   // Delay chosen to compensate for inconsistent circle radius (1 second). (To Be Refined)
  analogWrite(pwmToMotor, 0);    // Stops the car engine by outputting a zero to the servo motor.
  carServo.write(90);            // Reset to straighten the wheels.
}





