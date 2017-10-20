
#include "MorseGenerator.h"
#define morseString  "Hello World."
#define morsePin 12

MorseGenerator morse(morsePin);

void setup()
{
  morse.stringGen(morseString);
}

void loop()
{
  // morse.stringGen(morseString);
}

