#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define I2C_SDA A4
#define I2C_SCL A5

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

void vSetupPwm(){
  // Set pin 9's PWM frequency to 31250 Hz (31250/1 = 31250)
  setPwmFrequency(9, 8);
  setPwmFrequency(10, 8);
  setPwmFrequency(11, 8);
  // Note that the base frequency for pins 3, 9, 10, and 11 is 31250 Hz
  // Set pins 5 and 6's PWM frequency to 20833,33 Hz (62500/3 = 20833,33 Hz)
  // Note that the base frequency for pins 5 and 6 is 62500 Hz
  //setPwmFrequency(5, 3); // 3 is not valid number
  //setPwmFrequency(6, 3);
}

void vSetupOled() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
  display.clearDisplay();
}

void vPresentaPantallaDemo(int n0,int n1,int n2) {
  String sz;

  display.clearDisplay();              

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20, 0);
  sz = "SIARQ / ACCIONA";
  display.println(sz);
  display.setCursor(30, 20);
  sz = "n0: " + String(n0) + " %";
  display.println(sz);
  display.setCursor(30, 35);
  sz = "n1: " + String(n1) + " %";
  display.println(sz);
  display.setCursor(30, 50);
  sz = "n2: " + String(n2) + " %";
  display.println(sz);

  display.display();
}

void setup() {
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);

  vSetupPwm();
  vSetupOled();
}

void loop() {
  int nPot = analogRead(A1),n1 = 25, n2 = 75;

  if(!digitalRead(2)){
    n1+=25; n2-=25;
  }
  if(!digitalRead(3)){
    n1+=50; n2-=50;
  }
  
  analogWrite(9,map(n2,0,100,0,255));
  analogWrite(10,map(nPot,0,1023,0,255));
  analogWrite(11,map(n1,0,100,0,255));
  vPresentaPantallaDemo(map(nPot,0,1023,0,100),n1,n2);
  delay(2);
}
