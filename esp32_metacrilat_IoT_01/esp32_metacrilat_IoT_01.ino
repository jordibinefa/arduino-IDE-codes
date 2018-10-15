#include "Nextion.h"

// https://www.reddit.com/r/arduino/comments/4b6jfi/nexconfigh_for_nextion_display_on_arduino_uno/
#define ESP32
HardwareSerial HMISerial(2); // UART2. U2_RXD:GPIO16, U2_TXD:GPIO17

#include "PCF8574.h" // https://github.com/RobTillaart/Arduino/tree/master/libraries/PCF8574
#include <Wire.h>
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "credentials.h"
#include "pinMapping.h"
#define ESP32
#ifndef CFG_eu868
#error "This script is meant to connect to TTN EU network at 868MHz"
#endif
#define TX_INTERVAL         300

#define N_IMAGES 4
#define EXTERNAL_BUTTON 23

#define I2C_SDA 21
#define I2C_SCL 22

#define I2C_SDA_OLED 4
#define I2C_SCL_OLED 15

#define R1 0x01
#define R2 0x02
#define R3 0x04
#define R4 0x08

#define ADC_POT 35

#define ADDR_I2C_BME280 0x76

//PCF8574 PCF_38(0x38);
PCF8574 PCF_38(0x20);
SSD1306  display(0x3c, I2C_SDA, I2C_SCL);

NexPicture p0 = NexPicture(0, 0, "page0");

NexPage page0    = NexPage(0, 0, "page0");
NexPage page1    = NexPage(1, 0, "page1");

NexDSButton btR1 = NexDSButton(0, 1, "bt0");
NexDSButton btR2 = NexDSButton(0, 2, "bt1");
NexDSButton btR3 = NexDSButton(0, 3, "bt2");
NexDSButton btR4 = NexDSButton(0, 4, "bt3");

NexWaveform s0 = NexWaveform(1, 1, "s0");

NexGauge manometre  = NexGauge(1, 2, "z0");

char buffer[100] = {0};

static byte byState;

NexTouch *nex_listen_list[] =
{
  &btR1,
  &btR2,
  &btR3,
  &btR4,
  NULL
};

void vPresentaPantallaDemo(int n) {
  char buf[12]; // "-2147483648\0"

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_24);
  display.drawString(64, 0, "ttn.cat");
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 25, "wiki.binefa.cat");
  display.setFont(ArialMT_Plain_24);
  display.drawString(64, 42, itoa(n, buf, 10));

  display.display();
}

void vPresentaPantallaDemo(byte byS) {
  char buf[12]; // "-2147483648\0"

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, 0, (byS & R1) ? "R1:H" : "R1:L");
  display.drawString(64, 0, (byS & R2) ? "R2:H" : "R2:L");
  display.drawString(0, 30, (byS & R3) ? "R3:H" : "R3:L");
  display.drawString(64, 30, (byS & R4) ? "R4:H" : "R4:L");

  display.display();

}

void vInitOLED() {
  display.init();
  display.flipScreenVertically();

  display.clear();
  vPresentaPantallaDemo(0);

  delay(1000);
}

void btR1PopCallback(void *ptr) {
  uint32_t dual_state;
  NexDSButton *btn = (NexDSButton *)ptr;
  //dbSerialPrintln("btR1PopCallback");
  //dbSerialPrint("ptr=");
  //dbSerialPrintln((uint32_t)ptr);
  memset(buffer, 0, sizeof(buffer));

  /* Get the state value of dual state button component . */
  btR1.getValue(&dual_state);
  if (dual_state) {
    //t0.setText("HI! OPEN STATE");
    Serial.println("R1: HI! OPEN STATE");
    byState |= R1;
  }
  else
  {
    //t0.setText("HI! OFF STATE");
    Serial.println("R1: HI! OFF STATE");
    byState &= ~R1;
  }
  vPresentaPantallaDemo(byState);
}

void btR2PopCallback(void *ptr) {
  uint32_t dual_state;
  NexDSButton *btn = (NexDSButton *)ptr;
  //dbSerialPrintln("btR1PopCallback");
  //dbSerialPrint("ptr=");
  //dbSerialPrintln((uint32_t)ptr);
  memset(buffer, 0, sizeof(buffer));

  /* Get the state value of dual state button component . */
  btR2.getValue(&dual_state);
  if (dual_state) {
    //t0.setText("HI! OPEN STATE");
    Serial.println("R2: HI! OPEN STATE");
    byState |= R2;
  }
  else
  {
    //t0.setText("HI! OFF STATE");
    Serial.println("R2: HI! OFF STATE");
    byState &= ~R2;
  }
  vPresentaPantallaDemo(byState);
}

void btR3PopCallback(void *ptr) {
  uint32_t dual_state;
  NexDSButton *btn = (NexDSButton *)ptr;
  //dbSerialPrintln("btR1PopCallback");
  //dbSerialPrint("ptr=");
  //dbSerialPrintln((uint32_t)ptr);
  memset(buffer, 0, sizeof(buffer));

  /* Get the state value of dual state button component . */
  btR3.getValue(&dual_state);
  if (dual_state) {
    //t0.setText("HI! OPEN STATE");
    Serial.println("R3: HI! OPEN STATE");
    byState |= R3;
  }
  else
  {
    //t0.setText("HI! OFF STATE");
    Serial.println("R3: HI! OFF STATE");
    byState &= ~R3;
  }
  vPresentaPantallaDemo(byState);
}

void btR4PopCallback(void *ptr) {
  uint32_t dual_state;
  NexDSButton *btn = (NexDSButton *)ptr;
  //dbSerialPrintln("btR1PopCallback");
  //dbSerialPrint("ptr=");
  //dbSerialPrintln((uint32_t)ptr);
  memset(buffer, 0, sizeof(buffer));

  /* Get the state value of dual state button component . */
  btR4.getValue(&dual_state);
  if (dual_state) {
    //t0.setText("HI! OPEN STATE");
    Serial.println("R4: HI! OPEN STATE");
    byState |= R4;
  }
  else
  {
    //t0.setText("HI! OFF STATE");
    Serial.println("R4: HI! OFF STATE");
    byState &= ~R4;
  }
  vPresentaPantallaDemo(byState);
}

// Comença la part de LoRaWAN
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
void vReadBME280(float* fTc, float* fTf, float* fP, float* fRH) {
  unsigned int b1[24];
  unsigned int data[8];
  unsigned int dig_H1 = 0;
  for (int i = 0; i < 24; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(ADDR_I2C_BME280);
    // Select data register
    Wire.write((136 + i));
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 1 byte of data
    Wire.requestFrom(ADDR_I2C_BME280, 1);

    // Read 24 bytes of data
    if (Wire.available() == 1)
    {
      b1[i] = Wire.read();
    }
  }

  // Convert the data
  // temp coefficients
  unsigned int dig_T1 = (b1[0] & 0xff) + ((b1[1] & 0xff) * 256);
  int dig_T2 = b1[2] + (b1[3] * 256);
  int dig_T3 = b1[4] + (b1[5] * 256);

  // pressure coefficients
  unsigned int dig_P1 = (b1[6] & 0xff) + ((b1[7] & 0xff ) * 256);
  int dig_P2 = b1[8] + (b1[9] * 256);
  int dig_P3 = b1[10] + (b1[11] * 256);
  int dig_P4 = b1[12] + (b1[13] * 256);
  int dig_P5 = b1[14] + (b1[15] * 256);
  int dig_P6 = b1[16] + (b1[17] * 256);
  int dig_P7 = b1[18] + (b1[19] * 256);
  int dig_P8 = b1[20] + (b1[21] * 256);
  int dig_P9 = b1[22] + (b1[23] * 256);

  // Start I2C Transmission
  Wire.beginTransmission(ADDR_I2C_BME280);
  // Select data register
  Wire.write(161);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(ADDR_I2C_BME280, 1);

  // Read 1 byte of data
  if (Wire.available() == 1)
  {
    dig_H1 = Wire.read();
  }

  for (int i = 0; i < 7; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(ADDR_I2C_BME280);
    // Select data register
    Wire.write((225 + i));
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 1 byte of data
    Wire.requestFrom(ADDR_I2C_BME280, 1);

    // Read 7 bytes of data
    if (Wire.available() == 1)
    {
      b1[i] = Wire.read();
    }
  }

  // Convert the data
  // humidity coefficients
  int dig_H2 = b1[0] + (b1[1] * 256);
  unsigned int dig_H3 = b1[2] & 0xFF ;
  int dig_H4 = (b1[3] * 16) + (b1[4] & 0xF);
  int dig_H5 = (b1[4] / 16) + (b1[5] * 16);
  int dig_H6 = b1[6];

  // Start I2C Transmission
  Wire.beginTransmission(ADDR_I2C_BME280);
  // Select control humidity register
  Wire.write(0xF2);
  // Humidity over sampling rate = 1
  Wire.write(0x01);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(ADDR_I2C_BME280);
  // Select control measurement register
  Wire.write(0xF4);
  // Normal mode, temp and pressure over sampling rate = 1
  Wire.write(0x27);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(ADDR_I2C_BME280);
  // Select config register
  Wire.write(0xF5);
  // Stand_by time = 1000ms
  Wire.write(0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();

  for (int i = 0; i < 8; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(ADDR_I2C_BME280);
    // Select data register
    Wire.write((247 + i));
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 1 byte of data
    Wire.requestFrom(ADDR_I2C_BME280, 1);

    // Read 8 bytes of data
    if (Wire.available() == 1)
    {
      data[i] = Wire.read();
    }
  }

  // Convert pressure and temperature data to 19-bits
  long adc_p = (((long)(data[0] & 0xFF) * 65536) + ((long)(data[1] & 0xFF) * 256) + (long)(data[2] & 0xF0)) / 16;
  long adc_t = (((long)(data[3] & 0xFF) * 65536) + ((long)(data[4] & 0xFF) * 256) + (long)(data[5] & 0xF0)) / 16;
  // Convert the humidity data
  long adc_h = ((long)(data[6] & 0xFF) * 256 + (long)(data[7] & 0xFF));

  // Temperature offset calculations
  double var1 = (((double)adc_t) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
  double var2 = ((((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0) *
                 (((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
  double t_fine = (long)(var1 + var2);
  double cTemp = (var1 + var2) / 5120.0;
  double fTemp = cTemp * 1.8 + 32;

  // Pressure offset calculations
  var1 = ((double)t_fine / 2.0) - 64000.0;
  var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
  var2 = var2 + var1 * ((double)dig_P5) * 2.0;
  var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
  var1 = (((double) dig_P3) * var1 * var1 / 524288.0 + ((double) dig_P2) * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
  double p = 1048576.0 - (double)adc_p;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = ((double) dig_P9) * p * p / 2147483648.0;
  var2 = p * ((double) dig_P8) / 32768.0;
  double pressure = (p + (var1 + var2 + ((double)dig_P7)) / 16.0) / 100;

  // Humidity offset calculations
  double var_H = (((double)t_fine) - 76800.0);
  var_H = (adc_h - (dig_H4 * 64.0 + dig_H5 / 16384.0 * var_H)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * var_H * (1.0 + dig_H3 / 67108864.0 * var_H)));
  double humidity = var_H * (1.0 - dig_H1 * var_H / 524288.0);
  if (humidity > 100.0)
  {
    humidity = 100.0;
  }
  else if (humidity < 0.0)
  {
    humidity = 0.0;
  }

  // Output data to serial monitor
  
  Serial.print("Temperature in Celsius : ");
  Serial.print(cTemp);
  Serial.println(" C");
  Serial.print("Temperature in Fahrenheit : ");
  Serial.print(fTemp);
  Serial.println(" F");
  Serial.print("Pressure : ");
  Serial.print(pressure);
  Serial.println(" hPa");
  Serial.print("Relative Humidity : ");
  Serial.print(humidity);
  Serial.println(" RH");
  
  *fTc = (float)cTemp;
  *fTf = (float)fTemp;
  *fP = (float)pressure;
  *fRH = (float)humidity;
}

void vSwap(byte* a, byte* b) {
  byte byAux = *b;

  *b = *a;
  *a = byAux;
}
void float2Bytes(float val, byte* bytes_array) {
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
  vSwap(&bytes_array[0], &bytes_array[3]);
  vSwap(&bytes_array[1], &bytes_array[2]);
}

void ttnSend(osjob_t* j) {

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    //Serial.println(F("[RFM95] Pending message"));
    return;
  }

  // Prepare buffer
  unsigned char data[5];
  float fTc, fTf, fP, fRH;
  //data[0] = (autoincrement >> 24) & 0xFF;
  //data[1] = (autoincrement >> 16) & 0xFF;
  //data[2] = (autoincrement >>  8) & 0xFF;
  //data[3] = (autoincrement >>  0) & 0xFF;
  //float2Bytes(fT,data);
  vReadBME280(&fTc, &fTf, &fP, &fRH);
  if (fTc > -10.f && fTc < 90.) {
    
    Serial.print("T:");
    Serial.print(fTc);
    Serial.print(" *C <--> ");
    
    float2Bytes(fTc, data);
    data[4] = byState;
    for (int i = 0; i < 5 ; i++) {
      Serial.print("[0x");
      Serial.print(data[i], HEX);
      Serial.print("]");
    }
    Serial.println();
    

    // Prepare upstream data transmission at the next possible time.
    // Parameters are port, data, length, confirmed
    if(j)
      LMIC_setTxData2(1, data, 4, 0);
    else
      LMIC_setTxData2(1, data, 5, 0);

    Serial.println(F("[RFM95] Packet queued"));
  }

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
      /*
      if (LMIC.dataLen == 1) {
        if (LMIC.frame[LMIC.dataBeg] == 0x71){
          Serial.println("Activació de relé");
          PCF_38.write8(0xF7);
        }
        if (LMIC.frame[LMIC.dataBeg] == 0x51){
          Serial.println("Desactivació de relé");
          PCF_38.write8(0xFF);
        }
      }
      *//*
      if (LMIC.dataLen > 0 && LMIC.dataLen <= 2) {
        char szHex[3] = {0}; 
        szHex[0] = char(LMIC.frame[LMIC.dataBeg]);
        szHex[1] = char(LMIC.frame[LMIC.dataBeg+1]);
        //szHex[2] = 0;
        //PlcSerial.print(szHex);
        Serial.print("szHex: ");
        Serial.println(szHex);
      }
      */
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
      //Serial.println(F("[RFM95] EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      //Serial.println(F("[RFM95] EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      //Serial.println(F("[RFM95] EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      //Serial.println(F("[RFM95] EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      //Serial.println(F("[RFM95] EV_LINK_ALIVE"));
      break;
    default:
      //Serial.println(F("[RFM95] Unknown event"));
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

// Acaba la part de LoRaWAN


void setup() {
  Serial.begin(115200);    // U0_RXD:GPIO3, U0_TXD:GPIO1   (UART0)
  HMISerial.begin(9600); // U2_RXD:GPIO16, U2_TXD:GPIO17 (UART2)
  nexInit();
  btR1.attachPop(btR1PopCallback, &btR1);
  btR2.attachPop(btR2PopCallback, &btR2);
  btR3.attachPop(btR3PopCallback, &btR3);
  btR4.attachPop(btR4PopCallback, &btR4);

  pinMode(EXTERNAL_BUTTON, INPUT_PULLUP);
  delay(1000); // give me time to bring up serial monitor
  vInitOLED();
  Serial.println("esp32_metacrilat_IoT.ino");
  Wire.begin(I2C_SDA, I2C_SCL);
  //Wire.begin(I2C_SDA_OLED, I2C_SCL_OLED);

  byState = 0x00;

  // SPI interface
#ifdef ESP32
  SPI.begin(SCK_GPIO, MISO_GPIO, MOSI_GPIO, NSS_GPIO);
#endif

  // Init LMIC library to work with TTN EU
  ttnSetup();

  float fTc, fTf, fP, fRH;
  vReadBME280(&fTc, &fTf, &fP, &fRH);  
}

void loop() {
  static int nCmpt = 0;
  static uint8_t ch0_data, ch0_previ = 0;
  boolean bButtonState = !digitalRead(EXTERNAL_BUTTON);
  static boolean bLastButtonState = bButtonState;

  // Keeps track of the scheduled jobs
  os_runloop_once();

  if (bButtonState != bLastButtonState) {
    bLastButtonState = bButtonState;
    delay(50);
    if (bButtonState)
      nCmpt++;
    if (nCmpt >= N_IMAGES)
      nCmpt = 0;
    if (bButtonState){
      Serial.print("n: "); Serial.println(nCmpt);
      //p0.setPic(nCmpt);
      if (nCmpt % 2){
        ttnSend(NULL);
        page1.show();
        Serial.print("Tramet: ");Serial.println(byState,HEX);
      }else {
        page0.show();
        btR1.setValue(!!(byState & R1));
        btR2.setValue(!!(byState & R2));
        btR3.setValue(!!(byState & R3));
        btR4.setValue(!!(byState & R4));
      }
      vPresentaPantallaDemo(nCmpt);
    }
  }
  nexLoop(nex_listen_list);
  PCF_38.write8(~byState);
  if (nCmpt % 2) {
    ch0_data = (uint8_t )((180.f / 4096) * float(analogRead(ADC_POT)));
    s0.addValue(0, ch0_data);
    //if (ch0_previ != ch0_data) {
      manometre.setValue(ch0_data);
    //  ch0_previ = ch0_data;
    //}
  }
}
