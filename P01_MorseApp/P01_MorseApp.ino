#include "Morse.h"
#define morseString  "Erin Hintze"
#define morsePin 13

Morse morse(morsePin);

void setup() 
{
  // No setup
}


void loop() 
{
  delay(2000);
  morse.stringGen(morseString);
}

