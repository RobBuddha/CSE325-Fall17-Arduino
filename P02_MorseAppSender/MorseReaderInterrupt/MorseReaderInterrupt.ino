#include "MorseReader.h"
#define morsePin 2


MorseReader morseReader = MorseReader(morsePin);
volatile bool checkVal;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(morsePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(morsePin), morseISR, CHANGE);
  Serial.print(F("Setup Done\n"));
}

void loop() {
}

void morseISR()
{
  morseReader.Read_by_interrupt(digitalRead(morsePin));
}

