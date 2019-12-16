#include <HardwareSerial.h>

#define RX_1 34
#define TX_1 27
#define RX_2 16
#define TX_2 17

HardwareSerial MySerial1(1);
HardwareSerial MySerial2(2);

void setup() {
  Serial.begin(115200);
  MySerial1.begin(9600, SERIAL_8N1, RX_1, TX_1);
  MySerial2.begin(9600, SERIAL_8N1, RX_2, TX_2);
}

void loop() {
  uint8_t byteFromSerial;

  while (MySerial1.available() > 0) {
    byteFromSerial = MySerial1.read();
    MySerial2.write(byteFromSerial);
    Serial.write(byteFromSerial);
  }
  while (MySerial2.available() > 0) {
    byteFromSerial = MySerial2.read();
    MySerial1.write(byteFromSerial);
    Serial.write(byteFromSerial);
  }
  while (Serial.available() > 0) {
    byteFromSerial = Serial.read();
    MySerial1.write(byteFromSerial);
    MySerial2.write(byteFromSerial);
  }
}
