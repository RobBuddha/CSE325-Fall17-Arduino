#include "Arduino.h"
#include "MorseReader.h"

unsigned int last_time = 0;

MorseReader::MorseReader(int pin)
{
  buttonPin = pin;
  pinMode(buttonPin, INPUT);
}

void MorseReader::convert_morse(String str)
{
  // Morse to Alphanumeric conversion block

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

  ch = "";  // Reset the string to receive the next letter
}

void MorseReader::readByPolling() {
  button_s = digitalRead(buttonPin);  // Digital Read the current state to button_s

  if (button_s == HIGH && last_b_s == LOW) {
    
    // Dot/Dash space, Letter space, or Word space happened
    unsigned int time = millis();  // Record time in ms
    int duration = time - last_time;  // Duration of the LOW signal

    if (duration > 700 && duration < 800) {
      // Letter Space happened (duration = 750 ms)
      convert_morse(ch);
    } else if (duration > 950 && duration < 1050) {
      // Word Space happened ( duration = 1000 ms)
      convert_morse(ch);
      Serial.print(" ");
    }
    
    last_time = time;  // Update last_time with the current time
  } else if (button_s == LOW && last_b_s == HIGH) {
    
    // Dot or Dash happened. Measure duration of signal to determine HIGH/LOW
    unsigned int time = millis();  // Time recorded in ms
    int duration = time - last_time;  // Duration of HIGH signal

    if (duration > 150 && duration < 350) {
      // Dot happened. Append string <ch> with a '.'
      ch += ".";
    } else if (duration > 650 && duration < 850) {
      // Dash happened. Append string <ch> with a '-'
      ch += "-";
    }

    last_time = time;  // Update last_time with the current time
  }

  last_b_s = button_s; // Record the current state into the last state
}

