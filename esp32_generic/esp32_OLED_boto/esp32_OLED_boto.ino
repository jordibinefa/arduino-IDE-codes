#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"

#define N_IMAGES 4
#define EXTERNAL_BUTTON 23

#define I2C_SDA 4
#define I2C_SCL 15

SSD1306  display(0x3c, I2C_SDA, I2C_SCL);

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

void vInitOLED() {
  display.init();
  display.flipScreenVertically();

  display.clear();
  vPresentaPantallaDemo(0);

  delay(1000);
}

void setup(){
  Serial.begin(115200);
  pinMode(EXTERNAL_BUTTON, INPUT_PULLUP);
  delay(1000); // give me time to bring up serial monitor
  vInitOLED();
  Serial.println("esp32_OLED_boto.ino");
}

void loop() {
  static int nCmpt = 0;
  boolean bButtonState = !digitalRead(EXTERNAL_BUTTON);
  static boolean bLastButtonState = bButtonState;

  if(bButtonState != bLastButtonState){
    bLastButtonState = bButtonState;
    delay(50);
    if(bButtonState)
      nCmpt++;
    if(nCmpt >= N_IMAGES)
      nCmpt = 0;
    Serial.print("n: "); Serial.println(nCmpt); 
    vPresentaPantallaDemo(nCmpt);
  }
}
