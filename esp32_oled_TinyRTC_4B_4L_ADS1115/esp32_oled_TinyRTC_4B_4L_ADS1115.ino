#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;  /* 16-bit version */

#include "SSD1306.h"
// Include the UI lib
#include "OLEDDisplayUi.h"
// Include custom images
#include "images.h"

#define ESP32
#include "RTClib.h"
/*RTClib.h has been slaightly modified:
  #ifdef ESP32
  #define WIRE Wire
  #else
  #define WIRE Wire1
  #endif
*/


DS1307 rtc;
DateTime now;

#define I2C_SDA 21
#define I2C_SCL 22

#define I2C_SDA_OLED 4
#define I2C_SCL_OLED 15

#define EXTERNAL_BUTTON_B1 27
#define EXTERNAL_BUTTON_B2 19 /* B2 <-> B3) */
#define EXTERNAL_BUTTON_B3 18 /* B2 <-> B3) */
#define EXTERNAL_BUTTON_B4 23

#define R1_3  32
#define R2_3  33
#define R3_3  25
#define R4_3  26

// Initialize the OLED display using Wire library
SSD1306  display(0x3c, I2C_SDA, I2C_SCL);

OLEDDisplayUi ui ( &display );

int screenW = 128;
int screenH = 64;
int clockCenterX = screenW / 2;
int clockCenterY = ((screenH - 16) / 2) + 16; // top yellow part is 16 px height
//int clockRadius = 23;
int clockRadius = 26;

// utility function for digital clock display: prints leading 0
String twoDigits(int digits) {
  if (digits < 10) {
    String i = '0' + String(digits);
    return i;
  }
  else {
    return String(digits);
  }
}

void clockOverlay(OLEDDisplay *display, OLEDDisplayUiState* state) {

}

void analogClockFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  //  ui.disableIndicator();

  // Draw the clock face
  //  display->drawCircle(clockCenterX + x, clockCenterY + y, clockRadius);
  display->drawCircle(clockCenterX + x, clockCenterY + y, 2);
  //
  //hour ticks
  for ( int z = 0; z < 360; z = z + 30 ) {
    //Begin at 0 and stop at 360
    float angle = z ;
    angle = ( angle / 57.29577951 ) ; //Convert degrees to radians
    int x2 = ( clockCenterX + ( sin(angle) * clockRadius ) );
    int y2 = ( clockCenterY - ( cos(angle) * clockRadius ) );
    int x3 = ( clockCenterX + ( sin(angle) * ( clockRadius - ( clockRadius / 8 ) ) ) );
    int y3 = ( clockCenterY - ( cos(angle) * ( clockRadius - ( clockRadius / 8 ) ) ) );
    display->drawLine( x2 + x , y2 + y , x3 + x , y3 + y);
  }

  // display second hand
  float angle = now.second() * 6 ;
  angle = ( angle / 57.29577951 ) ; //Convert degrees to radians
  int x3 = ( clockCenterX + ( sin(angle) * ( clockRadius - ( clockRadius / 5 ) ) ) );
  int y3 = ( clockCenterY - ( cos(angle) * ( clockRadius - ( clockRadius / 5 ) ) ) );
  display->drawLine( clockCenterX + x , clockCenterY + y , x3 + x , y3 + y);
  //
  // display minute hand
  angle = now.minute() * 6 ;
  angle = ( angle / 57.29577951 ) ; //Convert degrees to radians
  x3 = ( clockCenterX + ( sin(angle) * ( clockRadius - ( clockRadius / 4 ) ) ) );
  y3 = ( clockCenterY - ( cos(angle) * ( clockRadius - ( clockRadius / 4 ) ) ) );
  display->drawLine( clockCenterX + x , clockCenterY + y , x3 + x , y3 + y);
  //
  // display hour hand
  angle = now.hour() * 30 + int( ( now.minute() / 12 ) * 6 )   ;
  angle = ( angle / 57.29577951 ) ; //Convert degrees to radians
  x3 = ( clockCenterX + ( sin(angle) * ( clockRadius - ( clockRadius / 2 ) ) ) );
  y3 = ( clockCenterY - ( cos(angle) * ( clockRadius - ( clockRadius / 2 ) ) ) );
  display->drawLine( clockCenterX + x , clockCenterY + y , x3 + x , y3 + y);
}

void digitalClockFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  String timenow = String(now.hour()) + ":" + twoDigits(now.minute()) + ":" + twoDigits(now.second());
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_24);
  display->drawString(clockCenterX + x , clockCenterY + y, timenow );
}

void vShowVoltages(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  int16_t adc0, adc1, adc2, adc3;

  //display.clear();

  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);
  adc0 = ads.readADC_SingleEnded(0);
  display->setTextAlignment(TEXT_ALIGN_LEFT);
  display->setFont(ArialMT_Plain_10);
  display->drawString(0, 5, "V0: " + String(adc0*0.125) + " mV");
  display->drawString(0, 20,"V1: " + String(adc1*0.125) + " mV");
  display->drawString(0, 35,"V2: " + String(adc2*0.125) + " mV");
  display->drawString(0, 50,"V3: " + String(adc3*0.125) + " mV");

  //display.display();
}

// This array keeps function pointers to all frames
// frames are the single views that slide in
FrameCallback frames[] = { analogClockFrame, digitalClockFrame, vShowVoltages };

// how many frames are there?
int frameCount = 3;

// Overlays are statically drawn on top of a frame eg. a clock
OverlayCallback overlays[] = { clockOverlay };
int overlaysCount = 1;

void vSetupOled() {
  // The ESP is capable of rendering 60fps in 80Mhz mode
  // but that won't give you much time for anything else
  // run it in 160Mhz mode or just set it to 30 fps
  ui.setTargetFPS(60);

  // Customize the active and inactive symbol
  ui.setActiveSymbol(activeSymbol);
  ui.setInactiveSymbol(inactiveSymbol);

  // You can change this to
  // TOP, LEFT, BOTTOM, RIGHT
  ui.setIndicatorPosition(TOP);

  // Defines where the first frame is located in the bar.
  ui.setIndicatorDirection(LEFT_RIGHT);

  // You can change the transition that is used
  // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
  ui.setFrameAnimation(SLIDE_LEFT);

  // Add frames
  ui.setFrames(frames, frameCount);

  // Add overlays
  ui.setOverlays(overlays, overlaysCount);

  // Initialising the UI will init the display too.
  ui.init();

  display.flipScreenVertically();
}

void vSetupTinyRTC() {
  Wire.begin(I2C_SDA, I2C_SCL);
  rtc.begin();
  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(__DATE__, __TIME__));
  }
  now = rtc.now();
}

void vSetupADS1115() {
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  ads.begin();
}

void setup() {
  pinMode(EXTERNAL_BUTTON_B1, INPUT_PULLUP);
  pinMode(EXTERNAL_BUTTON_B2, INPUT_PULLUP);
  pinMode(EXTERNAL_BUTTON_B3, INPUT_PULLUP);
  pinMode(EXTERNAL_BUTTON_B4, INPUT_PULLUP);

  pinMode(R1_3, OUTPUT);
  pinMode(R2_3, OUTPUT);
  pinMode(R3_3, OUTPUT);
  pinMode(R4_3, OUTPUT);

  Serial.begin(115200);
  Serial.println();

  vSetupOled();
  vSetupTinyRTC();
}

void loop() {
  int remainingTimeBudget = ui.update();
  int16_t adc0, adc1, adc2, adc3;

  if (remainingTimeBudget > 0) {
    // You can do some work here
    // Don't do stuff if you are below your
    // time budget.
    now = rtc.now();
    digitalWrite(R1_3, !digitalRead(EXTERNAL_BUTTON_B1));
    digitalWrite(R2_3, !digitalRead(EXTERNAL_BUTTON_B2));
    digitalWrite(R3_3, !digitalRead(EXTERNAL_BUTTON_B3));
    digitalWrite(R4_3, !digitalRead(EXTERNAL_BUTTON_B4));
    adc0 = ads.readADC_SingleEnded(0);
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2);
    adc3 = ads.readADC_SingleEnded(3);
    adc0 = ads.readADC_SingleEnded(0);
    Serial.print("AIN0: "); Serial.println(adc0);
    Serial.print("AIN1: "); Serial.println(adc1);
    Serial.print("AIN2: "); Serial.println(adc2);
    Serial.print("AIN3: "); Serial.println(adc3);
    Serial.print("V0: "); Serial.print(adc0 * 0.125); Serial.println("mV");
    Serial.print("V1: "); Serial.print(adc1 * 0.125); Serial.println("mV");
    Serial.print("V2: "); Serial.print(adc2 * 0.125); Serial.println("mV");
    Serial.print("V3: "); Serial.print(adc3 * 0.125); Serial.println("mV");
    delay(remainingTimeBudget);
  }
}
