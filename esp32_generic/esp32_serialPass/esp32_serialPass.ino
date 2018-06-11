HardwareSerial HMISerial(2); // UART2. U2_RXD:GPIO16, U2_TXD:GPIO17

void setup(){
  Serial.begin(115200);    // U0_RXD:GPIO3, U0_TXD:GPIO1   (UART0)
  HMISerial.begin(115200); // U2_RXD:GPIO16, U2_TXD:GPIO17 (UART2)
}

void loop(){
  if (HMISerial.available())
    Serial.write(HMISerial.read());
  if (Serial.available())
    HMISerial.write(Serial.read());
}
