
#include "MorseGenerator.h"
#define morseString  "abcdefghijklmnopqrstuvwxyz."
#define morsePin 13

MorseGenerator morse(morsePin);

void setup()
{
  // put your setup code here, to run once:
}


void loop()
{
  delay(2000);
  morse.stringGen(morseString);
}

