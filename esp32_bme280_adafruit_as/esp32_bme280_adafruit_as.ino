#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define I2C_SDA_0 4
#define I2C_SCL_0 5

#define I2C_SDA 21
#define I2C_SCL 22

#define I2C_SDA_OLED 4
#define I2C_SCL_OLED 15

#define SEALEVELPRESSURE_HPA (1022.25)

Adafruit_BME280 bme; // I2C

unsigned long delayTime;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(2000);

  // initialize BME280 sensor
  bool status;
  status = bme.begin(0x76);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  } else
    Serial.println("OK");

  Serial.println("-- Default Test --");
  Serial.println("normal mode, 16x oversampling for all, filter off,");
  Serial.println("0.5ms standby period");
  delayTime = 5000;

  Serial.println();
}


void loop() {
  // Only needed in forced mode! In normal mode, you can remove the next line.
  bme.takeForcedMeasurement(); // has no effect in normal mode

  printValues();
  delay(delayTime);
}


void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}
