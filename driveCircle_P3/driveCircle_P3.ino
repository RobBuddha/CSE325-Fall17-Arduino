
#include <Servo.h>

Servo carServo; // Create a Servo Object

#define pwmToMotor 4        // Defining the PWM pin to run the motor
#define servoPin 48         // Defining the servo control pin

void      driveCircle();    // Drive circle function initial definition to drive a circle
void      carStop();        // Stop function initial definition



int carSpeed = 0;           // define a variable for speed of the car

void setup()               // All the initail configuration should be placed here in setup 
{
  carServo.attach(servoPin);  // setting the Servo motor control for the pin 
}

void loop()                // Loop function which will be run forever
{
  driveCircle();          // drive a circle
  carStop();              // stop the car
                          // set new value for speed (change the duty cycle of the PWM)
  driveCircle();          // drive a circle again
  carStop();              // stop again
  delay(10000);           // wait for 10 seconds
}





void driveCircle() 
{  
  // The code to drive the car in a circle 
}

void carStop() 
{
  // The code to stop the car
}





