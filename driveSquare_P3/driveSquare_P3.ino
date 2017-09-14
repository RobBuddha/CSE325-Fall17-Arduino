
#include <Servo.h>

Servo carServo; // Create a Servo Object

#define pwmToMotor 4        // Defining the PWM pin to run the motor
#define servoPin 48         // Defining the servo control pin

void      driveSquare();    // Drive square function initial definition to drive a square
void      carStop();        // Stop function initial definition



int carSpeed = 255*0.1;           // define a variable for speed of the car

void setup()               // All the initail configuration should be placed here in setup 
{
  pinMode(pwmToMotor, OUTPUT);
  carServo.attach(servoPin);  // setting the Servo motor control for the pin 
}

void loop()                // Loop function which will be run forever
{
  driveSquare();          // drive a square
  
  carStop();              // stop the car
  delay(10000);           // wait for 10 seconds
}





void driveSquare() 
{  
  // The code to drive the car in a square
  
  carServo.write(90); // Go straight for 1500 sec
  delay(100);
  analogWrite(pwmToMotor, carSpeed);
  delay(1500);
  
  analogWrite(pwmToMotor, 0); // Stop and turn wheels to the right
  delay(100);
  carServo.write(180);
  delay(100);

  analogWrite(pwmToMotor, carSpeed); // Turn car and stop for 250ms
  delay(1075);
  analogWrite(pwmToMotor, 0);
  delay(100);

  //-----

  carServo.write(90); // Go straight for 1500 sec
  delay(100);
  analogWrite(pwmToMotor, carSpeed);
  delay(1500);
  
  analogWrite(pwmToMotor, 0); // Stop and turn wheels to the right
  delay(100);
  carServo.write(180);
  delay(100);

  analogWrite(pwmToMotor, carSpeed); // Turn car and stop for 250ms
  delay(1025);
  analogWrite(pwmToMotor, 0);
  delay(100);

  //-----

  carServo.write(90); // Go straight for 1500 sec
  delay(100);
  analogWrite(pwmToMotor, carSpeed);
  delay(1500);
  
  analogWrite(pwmToMotor, 0); // Stop and turn wheels to the right
  delay(100);
  carServo.write(180);
  delay(100);

  analogWrite(pwmToMotor, carSpeed); // Turn car and stop for 250ms
  delay(1025);
  analogWrite(pwmToMotor, 0);
  delay(100);

  //-----

  carServo.write(90); // Go straight for 1500 sec
  delay(100);
  analogWrite(pwmToMotor, carSpeed);
  delay(1500);
  
  analogWrite(pwmToMotor, 0); // Stop and turn wheels to the right
  delay(100);
  carServo.write(180);
  delay(100);

  analogWrite(pwmToMotor, carSpeed); // Turn car and stop for 250ms
  delay(1025);
  analogWrite(pwmToMotor, 0);
  delay(100);

  //-----

  carServo.write(90); // Go straight for 250ms
  delay(100);
  analogWrite(pwmToMotor, carSpeed);
  delay(250);

  analogWrite(pwmToMotor, 0);
}

void carStop() 
{
  // The code to stop the car
  carServo.write(90);
  analogWrite(pwmToMotor, 0);
}





