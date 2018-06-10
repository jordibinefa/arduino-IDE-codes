#include "Nextion.h"

// https://www.reddit.com/r/arduino/comments/4b6jfi/nexconfigh_for_nextion_display_on_arduino_uno/
#define ESP32
HardwareSerial HMISerial(2); // UART2. U2_RXD:GPIO16, U2_TXD:GPIO17

#include "PCF8574.h" // https://github.com/RobTillaart/Arduino/tree/master/libraries/PCF8574
#include <Wire.h>
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"

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

PCF8574 PCF_38(0x38);
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
  Serial.println("esp32_Nextion_02.ino");
  Wire.begin(I2C_SDA, I2C_SCL);
  //Wire.begin(I2C_SDA_OLED, I2C_SCL_OLED);

  byState = 0x00;
}

void loop() {
  static int nCmpt = 0;
  static uint8_t ch0_data, ch0_previ = 0;
  boolean bButtonState = !digitalRead(EXTERNAL_BUTTON);
  static boolean bLastButtonState = bButtonState;

  if (bButtonState != bLastButtonState) {
    bLastButtonState = bButtonState;
    delay(50);
    if (bButtonState)
      nCmpt++;
    if (nCmpt >= N_IMAGES)
      nCmpt = 0;
    Serial.print("n: "); Serial.println(nCmpt);
    //p0.setPic(nCmpt);
    if (nCmpt % 2)
      page1.show();
    else {
      page0.show();
      btR1.setValue(!!(byState & R1));
      btR2.setValue(!!(byState & R2));
      btR3.setValue(!!(byState & R3));
      btR4.setValue(!!(byState & R4));
    }
    vPresentaPantallaDemo(nCmpt);
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
