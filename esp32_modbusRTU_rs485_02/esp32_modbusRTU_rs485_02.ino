// Implementacio de Modbus RTU esclau
// a la placa IoT blanca
//
//  ESP32      IoT PLB      Modul RS485
//
//  GND          -              GND
//  5V           5              VCC
//  15           R1 (5)         DE
//  4            R2 (5)         RE
//  3            R  (5)         RO
//  1            T  (5)         DI
//                              A  / D+
//                              B  / D-
//
// 20180707 - wiki.binefa.cat

#include "PCF8574.h" // https://github.com/RobTillaart/Arduino/tree/master/libraries/PCF8574
#include <Wire.h>
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"

#define EXTERNAL_BUTTON 23
#define PRG_BUTTON 0

#define I2C_SDA 21
#define I2C_SCL 22

#define I2C_SDA_OLED 4
#define I2C_SCL_OLED 15

#define R1 0x01
#define R2 0x02
#define R3 0x04
#define R4 0x08

#define GPIO_R1 36
#define GPIO_R2 37

#define ADC_POT 35

PCF8574 PCF_38(0x38);
//PCF8574 PCF_38(0x20);
SSD1306  display(0x3c, I2C_SDA, I2C_SCL);
/////////////////////////////!!!!!!!!

#define N_MAX 12

#define MODULE_ADDRESS 0x02
#define COIL_ADDRESS 0x0000
#define COIL_ADDRESS_B 0x0001
#define COIL_ADDRESS_WR 0x0002

#define REG_ADDRESS_PRES 0x0001
#define REG_ADDRESS_TEMP 0x0002
#define REG_ADDRESS_DIST_INSP 0x0003
#define REG_ADDRESS_DIST_EXPU 0x0004

#define SSerialTxControlA I2C_SDA_OLED   //RS485 Direction control
#define SSerialTxControlB I2C_SCL_OLED   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

unsigned char ucSt[N_MAX];
static byte byState;

HardwareSerial modbusData(2);

void vModeTxRxRS485(boolean bMode) {
  digitalWrite(SSerialTxControlA, bMode);
  digitalWrite(SSerialTxControlB, bMode);
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

void vPresentaPantallaReg(int nS, const char* ccConcept, const char* ccUnit,int nDiv) {
  char buf[30]; // "-2147483648\0"

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_24);
  String sz(ccConcept);
  if(nDiv != 1)
    sz += " " + String(float(nS)/nDiv);
  else
    sz += " " + String(nS);
  Serial.println(sz);
  sz.toCharArray(buf,sz.length()+1);
  display.drawString(0, 0, buf);
  display.drawString(64, 30, ccUnit);

  display.display();

}

void vInitOLED() {
  display.init();
  display.flipScreenVertically();

  display.clear();
  vPresentaPantallaDemo(0);

  delay(1000);
}

void setup() {
  Serial.begin(115200);    // U0_RXD:GPIO3, U0_TXD:GPIO1   (UART0)
  modbusData.begin(115200); // U2_RXD:GPIO16, U2_TXD:GPIO17 (UART2)

  pinMode(EXTERNAL_BUTTON, INPUT_PULLUP);
  pinMode(GPIO_R1, INPUT);
  pinMode(GPIO_R2, INPUT);
  pinMode(SSerialTxControlA, OUTPUT); // hardwired to GPIO_R2
  pinMode(SSerialTxControlB, OUTPUT); // hardwired to GPIO_R1

  vModeTxRxRS485(RS485Receive);

  Wire.begin(I2C_SDA, I2C_SCL);
  //Wire.begin(I2C_SDA_OLED, I2C_SCL_OLED);

  vInitOLED();
  byState = 0x00;

  delay(1000); // give me time to bring up serial monitor
}

// Compute the MODBUS RTU CRC
// Adapted to Arduino from http://www.ccontrolsys.com/w/How_to_Compute_the_Modbus_RTU_Message_CRC
unsigned int uiModRTU_CRC(byte* buf, int len) {
  unsigned int crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
    crc ^= (unsigned int)buf[pos];          // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}

byte byReadButtons() {
  byte byE = 0xF0;

  byE |= (!digitalRead(EXTERNAL_BUTTON)) ? 0x01 : 0x00;
  byE |= (!digitalRead(PRG_BUTTON)) ? 0x02 : 0x00;

  return byE;
}

void vReadInputs(byte* ucSt, int len) {
  int i;
  unsigned int uiCRC, adr = (ucSt[2] << 8) | ucSt[3]; // Adreça d'inici dels coils
  // ucSt[4] i ucSt[5] diuen quants coils escriure
  // ucSt[6] diuen quants bytes de dades s'escriuen
  // ucSt[7] i ucSt[8] diuen quines dades s'escriuen
  // ucSt[9] i ucSt[10] diuen quin es el CRC
  //Serial.print("adr: "); Serial.println(adr, HEX);
  if (adr == COIL_ADDRESS) {
    ucSt[2] = 0x01; // Bytes a trametre
    ucSt[3] = byReadButtons();
    vModeTxRxRS485(RS485Transmit);
    //delay(10);
    for (i = 0 ; i < 4 ; i++) {
      modbusData.write(ucSt[i]);
      //Serial.print(ucSt[i], HEX); Serial.print(" ");
    }
    uiCRC = uiModRTU_CRC(ucSt, 4);
    modbusData.write((byte)(uiCRC & 0xFF));
    modbusData.write((byte)(uiCRC >> 8));
    //Serial.print((byte)(uiCRC & 0xFF), HEX); Serial.print(" ");
    //Serial.print((byte)(uiCRC >> 8)); Serial.print(" ");
    delay(10); // Added at ESP32 version
    vModeTxRxRS485(RS485Receive);
  }
  Serial.println();
}

void vReadInputsB(byte* ucSt, int len) {
  int i;
  unsigned int uiCRC, adr = (ucSt[2] << 8) | ucSt[3]; // Adreça d'inici dels coils
  // ucSt[4] i ucSt[5] diuen quants coils escriure
  // ucSt[6] diuen quants bytes de dades s'escriuen
  // ucSt[7] i ucSt[8] diuen quines dades s'escriuen
  // ucSt[9] i ucSt[10] diuen quin es el CRC
  if (adr == COIL_ADDRESS_B) {
    ucSt[2] = 0x02; // Bytes a trametre
    ucSt[3] = 0x00;
    ucSt[4] = byReadButtons();
    vModeTxRxRS485(RS485Transmit);
    for (i = 0 ; i < 5 ; i++) {
      modbusData.write(ucSt[i]);
    }
    uiCRC = uiModRTU_CRC(ucSt, 5);
    modbusData.write((byte)(uiCRC & 0xFF));
    modbusData.write((byte)(uiCRC >> 8));
    delay(10); // Added at ESP32 version
    vModeTxRxRS485(RS485Receive);
  }
  if (adr == COIL_ADDRESS_WR) {
    ucSt[2] = 0x02; // Bytes a trametre
    ucSt[3] = 0x00;
    //ucSt[4] = byReadButtons();
    ucSt[4] = byState;
    vModeTxRxRS485(RS485Transmit);
    for (i = 0 ; i < 5 ; i++) {
      modbusData.write(ucSt[i]);
    }
    uiCRC = uiModRTU_CRC(ucSt, 5);
    modbusData.write((byte)(uiCRC & 0xFF));
    modbusData.write((byte)(uiCRC >> 8));
    delay(10); // Added at ESP32 version
    vModeTxRxRS485(RS485Receive);
  }
}


void vEscriuCoils(byte* ucSt, int len) {
  int i;
  unsigned int uiCRC, adr = (ucSt[2] << 8) | ucSt[3]; // Adreça d'inici dels coils
  // ucSt[4] i ucSt[5] diuen quants coils escriure
  // ucSt[6] diuen quants bytes de dades s'escriuen
  // ucSt[7] i ucSt[8] diuen quines dades s'escriuen
  // ucSt[9] i ucSt[10] diuen quin es el CRC
  if (adr == COIL_ADDRESS) {
    //Serial.print("Escriu 0x");
    //Serial.println(ucSt[7],HEX);
    (ucSt[7] & 0x01) ? byState |= R1 : byState &= ~R1;
    (ucSt[7] & 0x02) ? byState |= R2 : byState &= ~R2;
    (ucSt[7] & 0x04) ? byState |= R3 : byState &= ~R3;
    (ucSt[7] & 0x08) ? byState |= R4 : byState &= ~R4;
    vModeTxRxRS485(RS485Transmit);
    for (i = 0 ; i < 6 ; i++) {
      modbusData.write(ucSt[i]);
    }
    uiCRC = uiModRTU_CRC(ucSt, 6);
    modbusData.write((byte)(uiCRC & 0xFF));
    modbusData.write((byte)(uiCRC >> 8));
    PCF_38.write8(~byState);
    delay(10); // Added at ESP32 version
    vModeTxRxRS485(RS485Receive);
  }
}

unsigned int uiEscriuRegistre(byte* ucSt, unsigned int *uiA,int len) {
  int i;
  unsigned int uiCRC, adr = (ucSt[2] << 8) | ucSt[3]; // Adreça d'inici dels coils
  unsigned int uiValue = (ucSt[4] << 8) | ucSt[5];
  *uiA = adr;
  
  if ((adr == REG_ADDRESS_PRES)||(adr == REG_ADDRESS_TEMP)||(adr == REG_ADDRESS_DIST_INSP)||(adr == REG_ADDRESS_DIST_EXPU)) {
    Serial.print("Valor registre: "); Serial.println(uiValue);
    //digitalWrite(10, ucSt[5] & 0x08);
    //digitalWrite(11, ucSt[5] & 0x04);
    //digitalWrite(12, ucSt[5] & 0x02);
    //digitalWrite(13, ucSt[5] & 0x01);
    vModeTxRxRS485(RS485Transmit);
    for (i = 0 ; i < 6 ; i++) {
      modbusData.write(ucSt[i]);
    }
    uiCRC = uiModRTU_CRC(ucSt, 6);
    modbusData.write((byte)(uiCRC & 0xFF));
    modbusData.write((byte)(uiCRC >> 8));
    delay(10); // Added at ESP32 version
    vModeTxRxRS485(RS485Receive);
  }
  return uiValue;
}

void vEscriuCoils5(byte* ucSt, int len) {
  int i;
  unsigned int uiCRC, adr = (ucSt[2] << 8) | ucSt[3]; // Adreça d'inici dels coils

  if (adr == 0x0000) {
    (ucSt[4] == 0xFF) ? byState |= R1 : byState &= ~R1;
    //digitalWrite(13, ucSt[4] == 0xFF);
  }
  if (adr == 0x0001) {
    (ucSt[4] == 0xFF) ? byState |= R2 : byState &= ~R2;
    //digitalWrite(12, ucSt[4] == 0xFF);
  }
  if (adr == 0x0002) {
    (ucSt[4] == 0xFF) ? byState |= R3 : byState &= ~R3;
    //digitalWrite(11, ucSt[4] == 0xFF);
  }
  if (adr == 0x0003) {
    (ucSt[4] == 0xFF) ? byState |= R4 : byState &= ~R4;
    //digitalWrite(10, ucSt[4] == 0xFF);
  }
  vModeTxRxRS485(RS485Transmit);
  for (i = 0 ; i < len ; i++) { // echo
    modbusData.write(ucSt[i]);
  }
  PCF_38.write8(~byState);
  delay(10); // Added at ESP32 version
  vModeTxRxRS485(RS485Receive);
}

void vProcessa(byte* ucSt, int len) {
  unsigned int uiR,uiA;
  
  if (ucSt[0] == MODULE_ADDRESS) {
    switch (ucSt[1]) {
      case 0x02: //Serial.println("Lectura a esclau");
        vReadInputs(ucSt, len);
        break;
      case 0x03: //Serial.println("Lectura a esclau");
        vReadInputsB(ucSt, len);
        break;
      case 0x05: //Serial.println("Escriptura de coils");
        vEscriuCoils5(ucSt, len);
        vPresentaPantallaDemo(byState);
        break;
      case 0x06: //Serial.println("Escriptura a esclau");
        uiR = uiEscriuRegistre(ucSt, &uiA, len);
        switch(uiA){
          case REG_ADDRESS_PRES:
            vPresentaPantallaReg(uiR,"Pr:","BAR",100);
            break;
          case REG_ADDRESS_TEMP:
            vPresentaPantallaReg(uiR,"Tª:","ºC",10);
            break;
          case REG_ADDRESS_DIST_INSP:
            vPresentaPantallaReg(uiR,"DI:","mm",1);
            break;
          case REG_ADDRESS_DIST_EXPU:
            vPresentaPantallaReg(uiR,"DE:","mm",1);
            break;
          default: 
            vPresentaPantallaReg(0,"?:","????",1);
        }      
        break;
      case 0x0F: //Serial.println("Escriptura a esclau");
        vEscriuCoils(ucSt, len);
        vPresentaPantallaDemo(byState);
        break;
        //default: //Serial.println("Funcio no implementada");
    }
  }
}

void loop() {
  int i, nCmpt = 0;
  unsigned int uiCRC;
  byte uiCrcL, uiCrcH;

  while (modbusData.available() > 0) {
    ucSt[nCmpt] = (unsigned char)modbusData.read();
    nCmpt++;
    delay(2);
  }
  if (nCmpt) {
    Serial.print("He llegit aquests "); Serial.print(nCmpt); Serial.println(" bytes: ");
    for (i = 0; i < nCmpt; i++) {
      Serial.print(ucSt[i], HEX); Serial.print(" ");
    }
    Serial.println();
    uiCRC = uiModRTU_CRC(ucSt, nCmpt - 2);
    if ((byte)(uiCRC >> 8) == ucSt[nCmpt - 1] && (byte)(uiCRC & 0xFF) == ucSt[nCmpt - 2]) {
      Serial.println("Trama amb CRC correcte");
      vProcessa(ucSt, nCmpt);
    } else {
      Serial.println("Trama amb CRC incorrecte");
    }
    Serial.print("Entrada: "); Serial.println(byReadButtons(), HEX);
    Serial.print("byState: "); Serial.println(byState, HEX);
  }
  nCmpt = 0;
}
