#include <SoftwareSerial.h>

SoftwareSerial esp8266(50,51); // RX, TX

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
  Serial.write("Hardware serial started...\n\n");

  setup_wifi();
}

void loop() {
  
}

boolean get_response() {
  String response = "";

  boolean isResponseReady = false;
  boolean isOK = false;
  
  delay(10);
  char prev_c = '\0';
  while (!isResponseReady) {
    char c = esp8266.read();
    if (c == '\r' || c == '\n') {
      prev_c = '\n';
    } else {
      if (prev_c == '\n') {
        prev_c = '\0';
        response += ';';
      }
      response += c;
    }
    if (response.indexOf("OK") != -1) {
      isResponseReady = true;
      isOK = true;
    } else if (response.indexOf("ERROR") != -1) {
      isResponseReady = true;
    }
  }
  return isOK;
}

void setup_wifi() {
  esp8266.begin(9600);
  
  // Set STA mode
  Serial.write("Sent: AT+CWMODE_CUR=1\n");
  esp8266.write("AT+CWMODE_CUR=1\r\n");
  if (get_response()) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
    return;
  }

  // Connect to AP
  Serial.write("Sent: AT+CWJAP_CUR=\"esp_01\",\"cse325demo\"\n");
  esp8266.write("AT+CWJAP_CUR=\"esp_01\",\"cse325demo\"\r\n");
  if (get_response()) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
    return;
  }
}

