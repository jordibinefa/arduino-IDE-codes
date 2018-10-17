/*

  AllWize - Simple Slave Example

  Simple slave that sends an auto-increment number every 5 seconds.

  Copyright (C) 2018 by AllWize <github@allwize.io>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

// -----------------------------------------------------------------------------
// Board definitions
// -----------------------------------------------------------------------------
#include <SoftwareSerial.h>
#if defined(ARDUINO_AVR_UNO)
#define RESET_PIN           7
#define RX_PIN              8
#define TX_PIN              9
#define DEBUG_SERIAL        Serial
#endif // ARDUINO_AVR_UNO

#if defined(ARDUINO_AVR_LEONARDO)
#define RESET_PIN           7
#define HARDWARE_SERIAL     Serial1
#define DEBUG_SERIAL        Serial
#endif // ARDUINO_AVR_LEONARDO

#if defined(ARDUINO_ARCH_SAMD)
#define RESET_PIN           7
#define HARDWARE_SERIAL     Serial1
#define DEBUG_SERIAL        SerialUSB
#endif // ARDUINO_ARCH_SAMD

#if defined(ARDUINO_ARCH_ESP8266)
#define RESET_PIN           14
#define RX_PIN              12
#define TX_PIN              13
#define DEBUG_SERIAL        Serial
#endif // ARDUINO_ARCH_ESP8266

#if defined(ARDUINO_ARCH_ESP32)
#define RESET_PIN           14
#define RX_PIN              12
#define TX_PIN              13
#define DEBUG_SERIAL        Serial
#endif // ARDUINO_ARCH_ESP32

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

#define WIZE_CHANNEL            11 /*CHANNEL_11*/
#define WIZE_POWER              POWER_20dBm
#define WIZE_DATARATE           DATARATE_2400bps
#define WIZE_NODE_ID            4 /*0x04*/


#define TEMPERATURE_PIN     A2
#define TEMPERATURE_SAMPLES 10

#if defined(ARDUINO_AVR_LEONARDO)
#define ANALOG_DEPTH        10
#define ANALOG_REFERENCE    5000
#endif // ARDUINO_AVR_LEONARDO
#define ANALOG_COUNT            (1 << ANALOG_DEPTH)
// -----------------------------------------------------------------------------
// AllWize
// -----------------------------------------------------------------------------

#include "AllWize.h"
AllWize * allwize;

void wizeSetup() {

  // Create and init AllWize object
#if defined(HARDWARE_SERIAL)
  allwize = new AllWize(&HARDWARE_SERIAL, RESET_PIN);
#else
  allwize = new AllWize(RX_PIN, TX_PIN, RESET_PIN);
#endif
  allwize->begin();
  if (!allwize->waitForReady()) {
    DEBUG_SERIAL.println("Error connecting to the module, check your wiring!");
    while (true);
  }

  allwize->slave();
  //allwize->setChannel(WIZE_CHANNEL);
  allwize->setChannel(WIZE_CHANNEL, true);
  allwize->setPower(WIZE_POWER);
  allwize->setDataRate(WIZE_DATARATE);
  allwize->setControlInformation(WIZE_NODE_ID);

}

void wizeSend(const char * payload) {

  DEBUG_SERIAL.print("[AllWize] Payload: ");
  DEBUG_SERIAL.println(payload);

  if (!allwize->send(payload)) {
    DEBUG_SERIAL.println("[AllWize] Error sending message");
  }

}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  pinMode(5, INPUT);
  pinMode(7, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);



  // Init serial DEBUG_SERIAL
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL && millis() < 5000);
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("[AllWize] Basic slave example");

  // Init radio
  wizeSetup();

}

char * snfloat(char * buffer, uint8_t len, uint8_t decimals, float value) {

  bool negative = value < 0;

  uint32_t mul = 1;
  for (uint8_t i = 0; i < decimals; i++) mul *= 10;

  value = abs(value);
  uint32_t value_int = int(value);
  uint32_t value_dec = int((value - value_int) * mul);

  char format[20];
  snprintf(format, sizeof(format), "%s%%lu.%%0%ulu", negative ? "-" : "", decimals);
  snprintf(buffer, len, format, value_int, value_dec);

  return buffer;

}

// Get the temperature from the MCP9701 sensor on board attached to A2
// As per datasheet (page 8):
// The change in  voltage  is  scaled  to  a  temperature  coefficient  of
// 10.0 mV/°C   (typical)   for   the   MCP9700/9700A   and
// 19.5 mV/°C  (typical)  for  the  MCP9701/9701A.  The
// output voltage at 0°C is also scaled to 500 mV (typical)
// and  400 mV  (typical)  for  the  MCP9700/9700A  and
// MCP9701/9701A,  respectively.
double getTemperature() {
  double sum = 0;
  for (uint8_t i = 0; i < TEMPERATURE_SAMPLES; i++) {
    sum += analogRead(TEMPERATURE_PIN);
  }
  double mV = sum * ANALOG_REFERENCE / ANALOG_COUNT / TEMPERATURE_SAMPLES;
  double t = (mV - 400.0) / 19.5;
  return t;
}

void vTramet(int count) {
  static char payloadCmpt[4], payloadTemp[7],payload[12];
  float t = getTemperature();

  // Convert the number to a string
  itoa(count, payloadCmpt, 10);
  snfloat(payloadTemp, sizeof(payloadTemp), 2, t);

  strcpy(payload,payloadCmpt);
  strcat(payload,",");
  strcat(payload,payloadTemp);

  // Send the string as payload
  wizeSend(payload);
}

void loop() {
  boolean bButtonState = digitalRead(7);
  boolean bButtonState2 = digitalRead(6);
  boolean bButtonSend = digitalRead(5);
  static boolean bLastButtonState = bButtonState;
  static boolean bLastButtonState2 = bButtonState2;
  static boolean bLastButtonSend = bButtonSend;
  static int nCmpt = 0;

  if (bButtonState != bLastButtonState) {
    delay(30);
    bLastButtonState = bButtonState;
    if (bButtonState) {
      nCmpt++;
      Serial.println(nCmpt);
    }
  }
  if (bButtonState2 != bLastButtonState2) {
    delay(30);
    bLastButtonState2 = bButtonState2;
    if (bButtonState2) {
      nCmpt--;
      Serial.println(nCmpt);
    }
  }
  if (bButtonSend != bLastButtonSend) {
    delay(30);
    bLastButtonSend = bButtonSend;
    if (bButtonSend) {
      vTramet(nCmpt);
      Serial.println("Tramet");
    }
  }
  /*
    // This static variables will hold the number as int and char string
    static uint8_t count = 0;
    static char payload[4];

    // Convert the number to a string
    itoa(count, payload, 10);

    // Send the string as payload
    wizeSend(payload);

    // Increment the number (it will overflow at 255)
    count++;




    // Wait 5 seconds and redo
    delay(5000);
  */
}
