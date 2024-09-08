#include <ESP8266WiFi.h>

void setup() {                
  WiFiMode(WIFI_STA);
  WiFi.disconnect(); 
  WiFi.mode(WIFI_OFF);
  delay(100);
}


void loop() {
}