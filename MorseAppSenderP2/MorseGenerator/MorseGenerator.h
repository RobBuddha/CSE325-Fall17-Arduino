/*
 * Morse.h for craeting Morse Codes
 */
 #ifndef Morse_h
 #define Morse_h
 #include "Arduino.h"
 class MorseGenerator
 {
  public:
  MorseGenerator(int pin);
  void dot();
  void dash();
  void lspace();
  void wspace();
  void charGen(char);
  void stringGen(String morse);
  private:
    int _morsePin;
      };


  #endif     

  
