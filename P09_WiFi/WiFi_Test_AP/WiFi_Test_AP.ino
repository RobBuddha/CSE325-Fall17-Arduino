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
  
  // Set AP mode
  Serial.write("Sent: AT+CWMODE_CUR=2\n");
  esp8266.write("AT+CWMODE_CUR=2\r\n");
  if (get_response()) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
    return;
  }

  // Create AP
  Serial.write("Sent: AT+CWSAP_CUR=\"esp_01\",\"cse325demo\",6,3\n");
  esp8266.write("AT+CWSAP_CUR=\"esp_01\",\"cse325demo\",6,3\r\n");
  if (get_response()) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
    return;
  }

  // Set up router IP addr
  Serial.write("Sent: AT+CIPAP_CUR=\"192.168.4.1\",\"192.168.4.1\",\"255.255.255.0\"\n");
  esp8266.write("AT+CIPAP_CUR=\"192.168.4.1\",\"192.168.4.1\",\"255.255.255.0\"\r\n");
  if (get_response()) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
    return;
  }

  // Enable multiple TCP connections
  Serial.write("Sent: AT+CIPMUX=1\n");
  esp8266.write("AT+CIPMUX=1\r\n");
  if (get_response()) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
    return;
  }

  // Start TCP server on Port 325
  Serial.write("Sent: AT+CIPSERVER=1,325\n");
  esp8266.write("AT+CIPSERVER=1,325\r\n");
  if (get_response()) {
    Serial.println("OK");
  } else {
    Serial.println("ERROR");
    return;
  }
}

