#include "Arduino.h"
#include "MorseReader.h"

// Declare variables
int buttonPin;
int button_s = 0;
int last_b_s = 0;
String ch;
unsigned long lastChange = 0; // Keeps track of the 250ms spaces between letters, not using digitalRead()
unsigned long curTime, dif;   // Will

MorseReader::MorseReader(int pin)
{
  buttonPin = pin;
  ch = "";
}

void MorseReader::convert_morse(String str)
{
   // Morse to Alphanumeric conversion block.

  if (str == ".-") {
    Serial.print("A");
    }
  else if (str == "-...") {
    Serial.print("B");
    }
  else if (str == "-.-.") {
    Serial.print("C");
    }
  else if (str == "-..") {
    Serial.print("D");
    }
  else if (str == ".") {
    Serial.print("E");
    }
  else if (str == "..-.") {
    Serial.print("F");
    }
  else if (str == "--.") {
    Serial.print("G");
    }
  else if (str == "....") {
    Serial.print("H");
    }
  else if (str == "..") {
    Serial.print("I");
    }
  else if (str == ".---") {
    Serial.print("J");
    }
  else if (str == "-.-") {
    Serial.print("K");
    }
  else if (str == ".-..") {
    Serial.print("L");
    }
  else if (str == "--") {
    Serial.print("M");
    }
  else if (str == "-.") {
    Serial.print("N");
    }
  else if (str == "---") {
    Serial.print("O");
    }
  else if (str == ".--.") {
    Serial.print("P");
    }
  else if (str == "--.-") {
    Serial.print("Q");
    }
  else if (str == ".-.") {
    Serial.print("R");
    }
  else if (str == "...") {
    Serial.print("S");
    }
  else if (str == "-") {
    Serial.print("T");
    }
  else if (str == "..-") {
    Serial.print("U");
    }
  else if (str == "...-") {
    Serial.print("V");
    }
  else if (str == ".--") {
    Serial.print("W");
    }
  else if (str == "-..-") {
    Serial.print("X");
    }
  else if (str == "-.--") {
    Serial.print("Y");
    }
  else if (str == "--..") {
    Serial.print("Z");
    }
  else if (str == ".----") {
    Serial.print("1");
    }
  else if (str == "..---") {
    Serial.print("2");
    }
  else if (str == "...--") {
    Serial.print("3");
    }
  else if (str == "...-") {
    Serial.print("4");
    }
  else if (str == ".....") {
    Serial.print("5");
    }
  else if (str == "-....") {
    Serial.print("6");
    }
  else if (str == "--...") {
    Serial.print("7");
    }
  else if (str == "---..") {
    Serial.print("8");
    }
  else if (str == "----.") {
    Serial.print("9");
    }
  else if (str == "-----") {
    Serial.print("0");
    }

  ch = "";                              // Reset the string to receive the next letter.
}

void MorseReader::Read_by_interrupt(int value)
{

  button_s = value;                     // Set the current state to button_s.
  curTime = millis();                   // Record time in ms.

  dif = curTime - lastChange;           // Duration of the LOW signal.

  // If the previous state was HIGH and is currently LOW, a space occurred.
  if(button_s == HIGH && last_b_s == LOW){

    if(dif > 745 && dif < 755){
      // Letter space happened (t = 750ms). Just convert the Morse string.
      convert_morse(ch);

    } else if(dif > 995 && dif < 1005){
      // Word space happened (t = 1000ms). First convert the character from Morse
      // string, then print a space to denote the end of a word.
      convert_morse(ch);
      Serial.print(" ");
    }

  // Button was just LOW, and now has received input.
  } else if(button_s == LOW && last_b_s == HIGH){

    // Dot or Dash happened. Measure duration of signal to determine HIGH/LOW.
    if(dif > 245 && dif < 255){
      // Dot happened (t=250ms).
      ch += '.';
    } else if(dif > 745 && dif < 755){
      // Dash happened (t=750 ms).
      ch += '-';
    } else {
      convert_morse(ch);
    }
  }
  lastChange = curTime;         // Update the last_time with the current time.
  last_b_s = button_s;          // Record the current state into the last state.
}
