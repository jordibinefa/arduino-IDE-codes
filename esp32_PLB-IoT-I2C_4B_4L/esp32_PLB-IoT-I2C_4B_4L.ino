/*
   wiki.binefa.cat - 20180929
*/

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
}

void loop() {
  digitalWrite(R1_3, !digitalRead(EXTERNAL_BUTTON_B1));
  digitalWrite(R2_3, !digitalRead(EXTERNAL_BUTTON_B2));
  digitalWrite(R3_3, !digitalRead(EXTERNAL_BUTTON_B3));
  digitalWrite(R4_3, !digitalRead(EXTERNAL_BUTTON_B4));
}
