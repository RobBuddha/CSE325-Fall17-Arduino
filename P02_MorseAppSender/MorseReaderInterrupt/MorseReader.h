#include "Arduino.h"

class MorseReader
{
  public:
            MorseReader(int buttonPin);
            void Read_by_interrupt(int value);   //The procedure to read the morse code by interrupt
            void convert_morse(String); //The procedure to convert the morse code to the letters
            void getChar(unsigned long duration);
  private:
 
            int             button_s = LOW;         // current state of the button
            int             last_b_s = LOW;         // previous state of the button
            unsigned long   x = 0;                  // rising time 
            unsigned long   y = 0;                  // fallin time
            unsigned long   w = 0;                  // signal width in LOW level
            unsigned long   z = 0;                  // signal width in HIGH level
            int             e=0;                    // the array contatins a valid letter
            String          ch;                     // the binary string corresponding to dots and dashes
            int             buttonPin;              // Pin which is connected to the Mega 2560 to read the morse code
};
