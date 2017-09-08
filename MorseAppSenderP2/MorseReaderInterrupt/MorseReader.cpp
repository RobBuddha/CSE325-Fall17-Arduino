#include "Arduino.h"
#include "MorseReader.h"

int buttonPin;
int button_s = 0;
int last_b_s = 0;
String ch;
unsigned long lastChange = 0; //Keeps track of the 250ms spaces between letters, not using digitalRead()
unsigned long curTime, dif;
bool lastSpaceChar = false;

MorseReader::MorseReader(int pin)
{
  buttonPin = pin;
  ch = "";
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

void MorseReader::Read_by_interrupt(int value)
{
  button_s = value;
  curTime = millis();
  dif = curTime - lastChange;
  
  if(button_s == HIGH && last_b_s == LOW){
    if(dif > 745 && dif < 755){
      //Letter space
      Serial.print(ch);
      convert_morse(ch);
      Serial.print("\n");
      ch = "";
    } else if(dif > 995 && dif < 1005){
      convert_morse(ch);
      Serial.print(" ");
      Serial.print("space\n");
      ch = "";
    }
  } else if(button_s == LOW && last_b_s == HIGH){
    Serial.print(dif);
    Serial.print('\n');
    if(dif > 245 && dif < 255){
      ch += '.';
    } else if(dif > 745 && dif < 755){
      ch += '-';
    }
  }
  lastChange = curTime;
  last_b_s = button_s;
  /*
   * 
   * 
   * if(value != 0){
    Serial.print(dif);
    Serial.print('\n');
  }
  if(value == 0 && dif > 745 && dif < 755){
    Serial.print("letter space\n");
  } else if(value == 0 && dif > 1495 && dif < 1505){
    Serial.print("word space\n");
  } else if(value == 1 && dif > 245 && dif < 255){
    Serial.print("dot\n");
  } else if(value == 1 && dif > 745 && dif < 755){
    Serial.print("dash\n");
  }
   * 
   * 
   * 
  curTime = millis();
  dif = curTime - lastChange;
  
  if(value == 0 && dif > 745 && dif < 755){
    Serial.print(ch);
    lastChange = curTime;
    convert_morse(ch);
    ch = "";
    //return;
  } else if(dif > 245 && dif < 255 && !lastSpaceChar){
    //Serial.print("dot\n");
    ch += ".";
    //lastSpaceChar = false;
  } else if(dif > 745 && dif < 755 && !lastSpaceChar) {
    //Serial.print("dash\n");
    ch += "-";
    //lastSpaceChar = false;
  } else if(dif > 1495 && dif < 1505 && !lastSpaceChar){
    //Serial.print(ch);
    //convert_morse(ch);
    //ch = "";
    //lastSpaceChar = false;
  }
  lastChange = curTime;
  //lastChange = curTime;
  //Serial.print(millis() - lastChange);
  //Serial.print(F("\n"));
  */
}
