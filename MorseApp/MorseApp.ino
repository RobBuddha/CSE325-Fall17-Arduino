#include "Morse.h"
#define morseString  "AB CD"
#define morsePin 13

Morse morse(morsePin);

void setup() 
{
//  put your setup code here, to run once:
}


void loop() 
{
//  put your main code here, to run repeatedly:
  delay(1000);
  morse.stringGen(morseString);
}

