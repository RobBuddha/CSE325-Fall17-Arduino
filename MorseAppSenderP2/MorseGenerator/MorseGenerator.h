/*
 * Morse.h for craeting Morse Codes
 */
 #ifndef Morse_h
 #define Morse_h
 #include "Arduino.h"
 class Morse
 {
  public:
  Morse(int pin);
  void dot();
  void dash();
  void lspace();
  void wspace();
  void lettergen(char);
  void generate(String morse);
  private:
    int _pin;
      };


  #endif     

  
