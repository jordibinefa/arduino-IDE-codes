// https://playground.arduino.cc/Main/I2cScanner?action=sourceblock&num=1

#include <Wire.h>

#define I2C_SDA 21
#define I2C_SCL 22

#define I2C_SDA_OLED 4
#define I2C_SCL_OLED 15

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  //Wire.begin(I2C_SDA_OLED, I2C_SCL_OLED);
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");
}

void loop() {
  static int nCmpt = 1;
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
  /* Això no va:
  nCmpt++;
  if(nCmpt%2){
    Wire.begin(I2C_SDA_OLED, I2C_SCL_OLED);
  }else{
    Wire.begin(I2C_SDA, I2C_SCL);
  }
  */
}
