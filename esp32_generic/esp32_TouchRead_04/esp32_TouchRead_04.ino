#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"

#define TOUCH_A T5
#define TOUCH_B T4
#define EXTERNAL_BUTTON 23

#define TOUCH_THRESHOLD 65

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

void vPresentaPantallatresValors(int n1,int n2, int n3) {
  char buf[20] = "TOUCH_A: "; // "-2147483648\0"

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  //strcpy(&buf[9],itoa(n2, buf, 10));
  display.drawString(64, 0, "TOUCH_A: "+String(n2));
  //display.setFont(ArialMT_Plain_16);
  //buf[6] = 'B'; buf[7] = '\0';
  //strcat(buf,itoa(n3, buf, 10));
  //display.drawString(64, 25, buf);
  display.drawString(64, 25, "TOUCH_B: "+String(n3));
  display.setFont(ArialMT_Plain_24);
  display.drawString(64, 42, itoa(n1, buf, 10));

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
  Serial.println("IoT board Touch & button Test");
}

boolean bIsPressedNowEdge(bool *bTouchPressedNow) {
  int nTouch[2] = {touchRead(TOUCH_A) , touchRead(TOUCH_B) };
  boolean bTouchIsPressed[2] = {nTouch[0] < TOUCH_THRESHOLD , nTouch[1] < TOUCH_THRESHOLD};
  static boolean bLastTouchState[2] = {false, false};


  for (int nWhichOne = 0 ; nWhichOne < 2 ; nWhichOne++) {
    //Serial.print("n[");Serial.print(nWhichOne);Serial.print("]= ");Serial.println(nTouch[nWhichOne]);
    if (bLastTouchState[nWhichOne] != bTouchIsPressed[nWhichOne]) {
      if (bTouchIsPressed[nWhichOne]) {
        //Serial.print("n[");Serial.print(nWhichOne);Serial.print("]= ");Serial.println(nTouch[nWhichOne]);
        if(nTouch[nWhichOne] != 0){
          bTouchPressedNow[nWhichOne] = true;
          Serial.print("nTouch[");Serial.print(nWhichOne);Serial.print("] = ");Serial.println(nTouch[nWhichOne]);
        }
      }
      bLastTouchState[nWhichOne] = bTouchIsPressed[nWhichOne];
    }
  }
  return bTouchPressedNow[0] || bTouchPressedNow[1];
}

void loop() {
  boolean bTouchPressedNow[2] = {false, false};
  static int nCmpt = 0;

  if (bIsPressedNowEdge(bTouchPressedNow)) {
    //if (bTouchPressedNow[0])
    //  nCmpt++;
    if (bTouchPressedNow[1])
      nCmpt--;
    Serial.print("n: "); Serial.println(nCmpt);
    //vPresentaPantallaDemo(nCmpt);
    //vPresentaPantallatresValors(nCmpt,touchRead(TOUCH_A),touchRead(TOUCH_B));
  }
  if(!digitalRead(EXTERNAL_BUTTON)){
    nCmpt = 0;
    Serial.print("n: "); Serial.println(nCmpt); 
    //vPresentaPantallaDemo(nCmpt);
    //vPresentaPantallatresValors(nCmpt,touchRead(TOUCH_A),touchRead(TOUCH_B));
  }
  //Serial.print("Button:"); Serial.println(digitalRead(EXTERNAL_BUTTON));
  vPresentaPantallatresValors(nCmpt,touchRead(TOUCH_A),touchRead(TOUCH_B));
  delay(200);
}
