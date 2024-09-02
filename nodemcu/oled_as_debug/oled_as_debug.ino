#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define DHTPIN 4  // D2 pin on ESP8266 corresponds to GPIO4
#define DHTTYPE DHT22

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
U8G2_FOR_ADAFRUIT_GFX u8g2_for_adafruit_gfx;
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Wire.begin(14, 12);
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  u8g2_for_adafruit_gfx.begin(display);  // connect u8g2 procedures to Adafruit GFX
  dht.begin();

  display.clearDisplay();
  display.setTextSize(1);  // Set text size to 2 (twice the original size)
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("DHT22 Sensor");
  display.display();
  
  delay(2000);  // Show the initial message for 2 seconds

  // Read and display temperature/humidity immediately
  updateDisplay();
}

void loop() {
  delay(30000); // Wait for 30 seconds

  // Read and display temperature/humidity
  updateDisplay();
}

void updateDisplay() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  display.clearDisplay();
  display.setCursor(0, 0);

  if (isnan(humidity) || isnan(temperature)) {
    display.println("Failed to read");
    display.println("from DHT sensor!");
  } else {
    display.println("Temp: ");
    display.print(temperature);
    display.println(" *C");

    display.println("Hum: ");
    display.print(humidity);
    display.println(" %");
  }

  display.display();
}