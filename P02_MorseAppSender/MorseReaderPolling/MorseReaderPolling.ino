#include "MorseReader.h"
#define morsePin 2

MorseReader reader(morsePin);

void setup() {
  Serial.begin(9600);
}

void loop() {
  reader.readByPolling();
}
