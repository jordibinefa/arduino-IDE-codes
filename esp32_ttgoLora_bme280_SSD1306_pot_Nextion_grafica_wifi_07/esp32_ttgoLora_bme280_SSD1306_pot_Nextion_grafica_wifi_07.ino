#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
#include <BME280I2C.h>
#include "images.h"

#include <WiFi.h>
#include <WiFiMulti.h>
WiFiMulti WiFiMulti;

#define WIFI_SSID "JESUITESFP"
#define WIFI_PWD "educanet01"

#define I2C_SDA 4
#define I2C_SCL 15

#include "Nextion.h"

// https://www.reddit.com/r/arduino/comments/4b6jfi/nexconfigh_for_nextion_display_on_arduino_uno/
#define ESP32
HardwareSerial HMISerial(2); // pin 16=RX, pin 17=TX

SSD1306  display(0x3c, I2C_SDA, I2C_SCL);
BME280I2C bme;

#define ADC_POT 13

#define LEVEL_HIGH      (15)
#define LEVEL_LOW       (0)

#define CH0_OFFSET  (20 - LEVEL_HIGH/2)
#define CH1_OFFSET  (CH0_OFFSET + 20 * 1)
#define CH2_OFFSET  (CH0_OFFSET + 20 * 2)
#define CH3_OFFSET  (CH0_OFFSET + 20 * 3)


NexWaveform s0 = NexWaveform(0, 1, "s0");

static uint8_t ch0_data = LEVEL_LOW;
static uint8_t ch1_data = LEVEL_LOW;
static uint8_t ch2_data = LEVEL_LOW;
static uint8_t ch3_data = LEVEL_LOW;

void vSetupWiFi() {
  delay(10);

  // We start by connecting to a WiFi network
  WiFiMulti.addAP(WIFI_SSID, WIFI_PWD);

  Serial.println();
  Serial.println();
  Serial.print("Wait for WiFi... ");

  while (WiFiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  vPantallaUnaLinia(WiFi.localIP().toString());

  delay(500);
  delay(1500);
}

void vPantallaUnaLinia(String szStr) {
  display.clear();

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 22, szStr);
  display.display();
}

void vInitBme280() {
  Wire.begin(I2C_SDA, I2C_SCL);

  while (!bme.begin())
  {
    Serial.println("No es detecta el sensor BME280!");
    vPantallaUnaLinia("No hi ha BME280");
    delay(1000);
  }
  Serial.println("Trobat el sensor BME280!");
  vPantallaUnaLinia("Detectat BME280");
  delay(1000);
  // bme.chipID(); // Deprecated. See chipModel().
  switch (bme.chipModel())
  {
    case BME280::ChipModel_BME280:
      Serial.println("Found BME280 sensor! Success.");
      break;
    case BME280::ChipModel_BMP280:
      Serial.println("Found BMP280 sensor! No Humidity available.");
      break;
    default:
      Serial.println("Found UNKNOWN sensor! Error!");
  }
}

void vInitOLED() {
  display.init();
  display.flipScreenVertically();

  display.clear();
  display.drawXbm(0, 0, 128, 64, (const char*)logo128x64_bmp);
  display.display();

  delay(1000);
}

void vLecturaBme280(float *pfTemp, float *pfHum, float *pfPres) {
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  bme.read(*pfPres, *pfTemp, *pfHum, tempUnit, presUnit);
}

void vPresentaPotBme280() {
  float fTemp, fHum, fPres;

  vLecturaBme280(&fTemp, &fHum, &fPres);
  display.clear();
  // Font Demo1
  // create more fonts at http://oleddisplay.squix.ch/
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Temp: " + String(fTemp) + " C");
  display.drawString(0, 14, "Hum: " + String(fHum) + " % HR");
  display.drawString(0, 28, "Pres: " + String(fPres) + " Pa");
  display.setFont(ArialMT_Plain_24);
  display.drawString(10, 42, "Pot: " + String(analogRead(13)));

  display.display();
}

void setup(void) {
  // 9600 bauds
  vInitOLED();
  vInitBme280();
  nexInit();
  vSetupWiFi();
  dbSerialPrintln("setup done");
}

void loop(void) {
  static int nCentisegonPrevi = 0, nCmpt = 0;
  int nCentisegonAra = millis() / 10;
  long int lnAD13 = analogRead(13);
  static uint32_t started = 0;

  if (millis() - started >= 2000)
  {
    started = millis();
    if (LEVEL_HIGH == ch0_data)
    {
      ch0_data = LEVEL_LOW;
    }
    else
    {
      ch0_data = LEVEL_HIGH;
    }
  }

  //ch1_data = ch0_data + random(0, 2);
  ch1_data = (uint8_t )((80.f / 4096) * float(analogRead(ADC_POT)));
  ch2_data = ch0_data + random(0, 5);
  ch3_data = ch0_data + random(0, 8);

  s0.addValue(0, CH0_OFFSET + ch0_data);
  //s0.addValue(1, CH1_OFFSET + ch1_data);
  s0.addValue(1, ch1_data);
  s0.addValue(2, CH2_OFFSET + ch2_data);
  s0.addValue(3, CH3_OFFSET + ch3_data);

  if (nCentisegonPrevi != nCentisegonAra) {
    nCmpt++;
    if (nCmpt > 1) {
      vPresentaPotBme280();
      nCmpt = 0;
    }
    nCentisegonPrevi = nCentisegonAra;
  }

}


