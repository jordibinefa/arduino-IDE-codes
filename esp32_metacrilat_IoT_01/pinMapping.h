/*
  // -----------------------------------------------------------------------------
  // Heltec ESP32 LoRa
  // https://robotzero.one/heltec-wifi-lora-32/
  // -----------------------------------------------------------------------------

  GPIO5  -- SX1278's SCK
  GPIO19 -- SX1278's MISO
  GPIO27 -- SX1278's MOSI
  GPIO18 -- SX1278's CS
  GPIO14 -- SX1278's RESET
  GPIO26 -- SX1278's IRQ(Interrupt Request)

  // -----------------------------------------------------------------------------

  RFM95 module pinout (top view, with the chip facing you) and the corresponding
  ESP32 PIN connected (wire the ones marked with an asterisk *)

        |-------------|
    GND | GND    DIO2 | 32
     19 | MISO   DIO1 | 33
     27 | MOSI   DIO0 | 26
      5 | SCK     3V3 | 3V
     18 | NSS    DIO4 |
     14 | RESET  DIO3 |
        | DIO5    GND | GND
    GND | GND     ANT |
        |-------------|

  // -----------------------------------------------------------------------------
*/
#define PCB_TTGO_LORA     0
#define PCBs_ESP32_LORA   1
#define PCBs_ESP32_RFM95  2

#define PCB_MODE PCB_TTGO_LORA

#if PCB_MODE == PCB_TTGO_LORA
  #define SCK_GPIO     5
  #define MISO_GPIO   19
  #define MOSI_GPIO   27
  #define NSS_GPIO    18
  #define RXTX_GPIO   LMIC_UNUSED_PIN
  #define RESET_GPIO  14
  #define DIO0_GPIO   26
  #define DIO1_GPIO   33
  #define DIO2_GPIO   32
#endif

#if PCB_MODE == PCBs_ESP32_LORA
  #define SCK_GPIO     5
  #define MISO_GPIO   15
  #define MOSI_GPIO   25
  #define NSS_GPIO    16
  #define RXTX_GPIO   36
  #define RESET_GPIO  14
  #define DIO0_GPIO   26
  #define DIO1_GPIO   13
  #define DIO2_GPIO   12
#endif

#if PCB_MODE == PCBs_ESP32_RFM95
  #define SCK_GPIO     5
  #define MISO_GPIO   15
  #define MOSI_GPIO   25
  #define NSS_GPIO    16
  #define RXTX_GPIO   LMIC_UNUSED_PIN
  #define RESET_GPIO  14
  #define DIO0_GPIO   26
  #define DIO1_GPIO   13
  #define DIO2_GPIO   12
#endif

// Requires this fork: https://github.com/jpmeijers/arduino-lmic
const lmic_pinmap lmic_pins = {
    .nss = NSS_GPIO,
    .rxtx = RXTX_GPIO,
    .rst = RESET_GPIO,
    .dio = {DIO0_GPIO, DIO1_GPIO, DIO2_GPIO},
};

