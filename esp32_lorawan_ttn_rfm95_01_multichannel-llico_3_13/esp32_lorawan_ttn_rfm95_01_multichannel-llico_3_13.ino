/*
  Based on Xose Pérez codes.

  Code modified by Jordi Binefa <jordibinefa at electronics dot cat>

  More info at: https://wiki.binefa.cat/
  Twitter: https://twitter.com/JordiBinefa

  -------------------------------------------------------------------------------

  Xose Pérez Info: 
  
  Copyright (C) 2016-2018
  Xose Pérez <xose dot perez at gmail dot com>
  for The Things Network Catalunya Wiki (http://thethingsnetwork.cat)
  Based on LMIC library example

  This sketch sends an incrementing number every minute

  // -----------------------------------------------------------------------------

  Requirements:

  This sketch requires LMIC library by Matthijs Kooijman
  https://github.com/matthijskooijman/arduino-lmic


  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include "credentials.h"
#include "pinMapping.h"

#define BUTTON_PRG 0

#define ESP32

#ifndef CFG_eu868
#error "This script is meant to connect to TTN EU network at 868MHz"
#endif

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

#define SERIAL_BAUD         115200
#define TX_INTERVAL         60

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// Job
static osjob_t sendjob;

// Value
unsigned long autoincrement = 0;

// -----------------------------------------------------------------------------
// LMIC
// -----------------------------------------------------------------------------

void ttnSend(osjob_t* j) {

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("[RFM95] Pending message"));
    return;
  }

  // Prepare buffer
  unsigned char data[4];
  data[0] = (autoincrement >> 24) & 0xFF;
  data[1] = (autoincrement >> 16) & 0xFF;
  data[2] = (autoincrement >>  8) & 0xFF;
  data[3] = (autoincrement >>  0) & 0xFF;

  // Prepare upstream data transmission at the next possible time.
  // Parameters are port, data, length, confirmed
  LMIC_setTxData2(1, data, 4, 0);

  Serial.println(F("[RFM95] Packet queued"));

  // Next TX is scheduled after TX_COMPLETE event.
  autoincrement++;
}

// LMIC library will call this method when an event is fired
void onEvent(ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");

  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("[RFM95] EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("[RFM95] EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("[RFM95] EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("[RFM95] EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("[RFM95] EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("[RFM95] EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("[RFM95] EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("[RFM95] EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("[RFM95] EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("[RFM95] EV_TXCOMPLETE (includes waiting for RX windows)"));

      if (TXRX_ACK) {
        Serial.println(F("TXRX_ACK confirmed UP frame was acked\n"));
      }
      Serial.println(F("TX complete ....................................."));
      Serial.print(LMIC.dataLen); Serial.print(" ");
      Serial.println(LMIC.frame[LMIC.dataBeg - 1]);
      for (int i = 0;  i < LMIC.dataLen;  i++) {
        Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
      }
      Serial.println(" ");

      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("[RFM95] ACK received"));
      }

      if (LMIC.dataLen) {
        Serial.print(F("[RFM95] Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));

        for (int i = 0; i < LMIC.dataLen; i++) {
          if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
            Serial.print(F("0"));
          }
          Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }

      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), ttnSend);
      break;

    case EV_LOST_TSYNC:
      Serial.println(F("[RFM95] EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("[RFM95] EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("[RFM95] EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("[RFM95] EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("[RFM95] EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("[RFM95] Unknown event"));
      break;

  }

}

void ttnSetup() {
  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100); // A treure

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band

  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job
  ttnSend(&sendjob);
}

// -----------------------------------------------------------------------------
// Main methods
// -----------------------------------------------------------------------------

void setup() {
  // Init serial port for debugging
  Serial.begin(115200);
  Serial.println("[MAIN] Startup");

  // SPI interface
#ifdef ESP32
  SPI.begin(SCK_GPIO, MISO_GPIO, MOSI_GPIO, NSS_GPIO);
#endif

  // Init LMIC library to work with TTN EU
  ttnSetup();
}

void loop() {
  static unsigned long last = 0;
  
  // Keeps track of the scheduled jobs
  os_runloop_once();

  // Manual send using the button
  static bool button_status = true; // not-pressed status (button has pull-up)
  if (digitalRead(BUTTON_PRG) != button_status) {
    delay(50);  // debounce
    if (digitalRead(BUTTON_PRG) != button_status) {
      // If status was true now it's false so button pressed
      if (button_status) {
        last = millis();
        ttnSend(NULL);
      }
      // Update status cache
      button_status = !button_status;
    }
  }
}
