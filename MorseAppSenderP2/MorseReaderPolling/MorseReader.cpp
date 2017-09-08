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

  str = "";
}

void MorseReader::readByPolling()
{
  button_s = digitalRead(buttonPin);

  if (button_s == HIGH && last_b_s == LOW) {
    // Dot/Dash, Letter, or Word space happened
    unsigned int time = millis(); // log the time when the state changed
    int duration = time - last_time;

    if (duration > 650 && duration < 850) {
      // Letter Space
      convert_morse(ch);
    } else if (duration > 900 && duration < 1100) {
      // Word Space
      Serial.print(" ");
    }

    last_time = time;
  } else if (button_s == LOW && last_b_s == HIGH) {
    // Dot or Dash happened
    unsigned int time = millis();
    int duration = time - last_time;

    if (duration > 150 && duration < 350) {
      // Dot
      ch += ".";
    } else if (duration > 650 && duration < 850) {
      // Dash
      ch += "-";
    }
    
    last_time = time; // log the time when the state changed
  }

  last_b_s = button_s;
}

void MorseReader::Read_by_interrupt()
{
  
}
