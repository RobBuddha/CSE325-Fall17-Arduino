/*
 * Morse.h for craeting Morse Codes
 */
#ifndef Morse_h
#define Morse_h

#include "Arduino.h"

class Morse
{
  public:
    Morse(int morsePin);
    void dot();
    void dash();
    void lspace();
    void wspace();
    void charGen(char);
    void stringGen(String morseString);
  private:
    int _morsePin;
};

#endif     
