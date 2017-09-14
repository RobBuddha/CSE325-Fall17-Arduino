
#include <Servo.h>

Servo carServo; // Create a Servo Object

#define pwmToMotor 4        // Defining the PWM pin to run the motor
#define servoPin 48         // Defining the servo control pin

void      driveCircle();    // Drive circle function initial definition to drive a circle
void      carStop();        // Stop function initial definition



int carSpeed = 255*0.1;           // define a variable for speed of the car
double servoDeg = 0;           // defines a variable for degree of servo motor

void setup()               // All the initail configuration should be placed here in setup 
{
  carServo.attach(servoPin);  // setting the Servo motor control for the pin
  carServo.write(90);         // Reset to straight
  pinMode(pwmToMotor, OUTPUT); 
}

void loop()                // Loop function which will be run forever
{
  delay(1000);
  driveCircle();          // drive a circle
  carStop();              // stop the car
  delay(1000000000000000000);
  /*                        // set new value for speed (change the duty cycle of the PWM)
  driveCircle();          // drive a circle again
  carStop();              // stop again
  delay(10000);           // wait for 10 seconds
  */
}





void driveCircle() 
{  
  // The code to drive the car in a circle 
  for(servoDeg = 90;servoDeg <= 180;servoDeg++){
    carServo.write(servoDeg);
    analogWrite(pwmToMotor, carSpeed);  
    if(servoDeg < 45){
        delay(40);
    } else if(servoDeg < 80){
      delay(30);
    } else {
      delay(50);
    }
    
  }
  delay(100);
}

void carStop() 
{
  delay(1000);
  analogWrite(pwmToMotor, 0);
  carServo.write(90);
  // The code to stop the car
}





