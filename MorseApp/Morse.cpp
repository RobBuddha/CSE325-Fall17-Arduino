/*
 * Morse.cpp For using the class
 */

#include "Arduino.h"
#include "Morse.h"

Morse::Morse(int morsePin)
{
  _morsePin = morsePin;
   pinMode(_morsePin, OUTPUT);
}

void Morse::dot()
{
  digitalWrite(_morsePin, HIGH);
  delay(250);
  digitalWrite(_morsePin, LOW);
  delay(250);
}

void Morse::dash()
{
  digitalWrite(_morsePin, HIGH);
  delay(750);
  digitalWrite(_morsePin, LOW);
  delay(250);
}

void Morse::lspace()
{
  delay(500);
}

void Morse::wspace()
{
  delay(750);
}
  
void Morse::stringGen(String str)
{
  // Loop over each character of the string
  for (int i = 0; i < str.length(); i++) {
    char ch = str.charAt(i); // Store the value of the current character
    if (ch == '.'){
	    return; // If current character is a period, end morse code translation
    }
    charGen(ch); // Output morse for the current character
    if (ch != ' ') {
      lspace(); // If the current character is not a space, do a letter space delay
    }
  }
}

void Morse::charGen(char ch)
{
  switch(ch) {
    case 'A' :
    case 'a' :
      dot();
      dash();
      break;
    case 'B' :
    case 'b' :
      dash();
      dot();
      dot();
      dot();
      break;
    case 'C' :
    case 'c' :
      dash();
      dot();
      dash();
      dot();
      break;
    case 'D' :
    case 'd' :
      dash();
      dot();
      dot();
      break;
    case 'E' :
    case 'e' :
      dot();
      break;
    case 'F' :
    case 'f' :
      dot();
      dot();
      dash();
      dot();
      break;
    case 'G' :
    case 'g' :
      dash();
      dash();
      dot();
      break;
    case 'H' :
    case 'h' :
      dot();
      dot();
      dot();
      dot();
      break;
    case 'I' :
    case 'i' :
      dot();
      dot();
      break;
    case 'J' :
    case 'j' :
      dot();
      dash();
      dash();
      dash();
      break;
    case 'K' :
    case 'k' :
      dash();
      dot();
      dash();
      break;
    case 'L' :
    case 'l' :
      dot();
      dash();
      dot();
      dot();
      break;
    case 'M' :
    case 'm' :
      dash();
      dash();
      break;
    case 'N' :
    case 'n' :
      dash();
      dot();
      break;
    case 'O' :
    case 'o' :
      dash();
      dash();
      dash();
      break;
    case 'P' :
    case 'p' :
      dot();
      dash();
      dash();
      dot();
      break;
    case 'Q' :
    case 'q' :
      dash();
      dash();
      dot();
      dash();
      break;
    case 'R' :
    case 'r' :
      dot();
      dash();
      dot();
      break;
    case 'S' :
    case 's' :
      dot();
      dot();
      dot();
      break;
    case 'T' :
    case 't' :
      dash();
      break;
    case 'U' :
    case 'u' :
      dot();
      dot();
      dash();
      break;
    case 'V' :
    case 'v' :
      dot();
      dot();
      dot();
      dash();
      break;
    case 'W' :
    case 'w' :
      dot();
      dash();
      dash();
      break;
    case 'X' :
    case 'x' :
      dash();
      dot();
      dot();
      dash();
      break;
    case 'Y' :
    case 'y' :
      dash();
      dot();
      dash();
      dash();
      break;
    case 'Z' :
    case 'z' :
      dash();
      dash();
      dot();
      dot();
      break;
    case '1' :
      dot();
      dash();
      dash();
      dash();
      dash();
      break;
    case '2' :
      dot();
      dot();
      dash();
      dash();
      dash();
      break;
    case '3' :
      dot();
      dot();
      dot();
      dash();
      dash();
      break;
    case '4' :
      dot();
      dot();
      dot();
      dot();
      dash();
      break;
    case '5' :
      dot();
      dot();
      dot();
      dot();
      dot();
      break;
    case '6' :
      dash();
      dot();
      dot();
      dot();
      dot();
      break;
    case '7' :
      dash();
      dash();
      dot();
      dot();
      dot();
      break;
    case '8' :
      dash();
      dash();
      dash();
      dot();
      dot();
      break;
    case '9' :
      dash();
      dash();
      dash();
      dash();
      dot();
      break;
    case '0' :
      dash();
      dash();
      dash();
      dash();
      dash();
      break;
    case ' ' :
      wspace();
      break;
  }
}
