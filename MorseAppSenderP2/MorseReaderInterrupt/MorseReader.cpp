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
  
}
  
void MorseReader::convert_morse(String ch1)
{
	
}

void MorseReader::Read_by_interrupt()
{
  curTime = millis();
  dif = curTime - lastChange;
  //Dot
  if(dif > 245 && dif < 255 && !lastSpaceChar){
    Serial.print("dot\n");
    ch += ".";
  } else if(dif > 745 && dif < 755 && !lastSpaceChar) {
    Serial.print("dash\n");
    ch += "-";
  } else if(dif > 995 && dif < 1005 && !lastSpaceChar){
    Serial.print(ch);
    ch = "";
  }
  lastChange = curTime;
  //Serial.print(millis() - lastChange);
  //Serial.print(F("\n"));
}
