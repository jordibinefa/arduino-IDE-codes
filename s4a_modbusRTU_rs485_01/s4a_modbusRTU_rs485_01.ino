// Implementacio de Modbus RTU esclau
//
//  Arduino     S4A PLB      Modul RS485
//
//  GND          P2-1           GND
//  5V           P2-10          VCC
//  A2 / 16      P1-2           DE
//  A3 / 17      P1-3           RE
//  4            4              RO
//  5            5              DI
//                              A  / D+
//                              B  / D-
//
// 20180702 - www.binefa.cat/blog

#define RS485

#ifdef RS485
  #include <SoftwareSerial.h>
  SoftwareSerial mySerial(4, 5); // RX, TX !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#else
  #define mySerial Serial
#endif
/////////////////////////////!!!!!!!!

#define N_MAX 12

#define MODULE_ADDRESS 0x02
#define COIL_ADDRESS 0x0000
#define COIL_ADDRESS_B 0x0001
#define COIL_ADDRESS_WR 0x0002

#define SSerialTxControlA 16   //RS485 Direction control
#define SSerialTxControlB 17   //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

unsigned char ucSt[N_MAX];

void vModeTxRxRS485(boolean bMode) {
  digitalWrite(SSerialTxControlA, bMode);
  digitalWrite(SSerialTxControlB, bMode);
}

void setup() {
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(19, INPUT); // A5
  pinMode(18, INPUT); // A4
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(SSerialTxControlA, OUTPUT); //A2/16 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  pinMode(SSerialTxControlB, OUTPUT); //A3/17 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  vModeTxRxRS485(RS485Receive);
  Serial.begin(9600);
#ifdef RS485
  mySerial.begin(9600); //!!!!!!!!!!!!!!!!!!!!!!!!!!
#endif
  /*
  Serial.print("Hola");
  delay(50);
  vModeTxRxRS485(RS485Transmit);
  delay(50);
  mySerial.print("A10");
  vModeTxRxRS485(RS485Receive);
  */
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
    digitalWrite(10, ucSt[7] & 0x08);
    digitalWrite(11, ucSt[7] & 0x04);
    digitalWrite(12, ucSt[7] & 0x02);
    digitalWrite(13, ucSt[7] & 0x01);
    vModeTxRxRS485(RS485Transmit);
    for (i = 0 ; i < 6 ; i++) {
      mySerial.write(ucSt[i]);
    }
    uiCRC = uiModRTU_CRC(ucSt, 6);
    mySerial.write((byte)(uiCRC & 0xFF));
    mySerial.write((byte)(uiCRC >> 8));
    vModeTxRxRS485(RS485Receive);
  }
}

byte byLecturaBotons() {
  byte byE = 0xF0;

  byE |= (!digitalRead(2)) ? 0x01 : 0x00;
  byE |= (!digitalRead(3)) ? 0x02 : 0x00;
  byE |= (!digitalRead(19)) ? 0x04 : 0x00;
  byE |= (!digitalRead(18)) ? 0x08 : 0x00;

  return byE;
}

byte byLecturaLeds() {
  byte byE = 0xF0;

  byE |= (!digitalRead(13)) ? 0x01 : 0x00;
  byE |= (!digitalRead(12)) ? 0x02 : 0x00;
  byE |= (!digitalRead(11)) ? 0x04 : 0x00;
  byE |= (!digitalRead(10)) ? 0x08 : 0x00;

  return byE;
}


void vLlegeixCoils(byte* ucSt, int len) {
  int i;
  unsigned int uiCRC, adr = (ucSt[2] << 8) | ucSt[3]; // Adreça d'inici dels coils
  // ucSt[4] i ucSt[5] diuen quants coils escriure
  // ucSt[6] diuen quants bytes de dades s'escriuen
  // ucSt[7] i ucSt[8] diuen quines dades s'escriuen
  // ucSt[9] i ucSt[10] diuen quin es el CRC
  if (adr == COIL_ADDRESS) {
    ucSt[2] = 0x01; // Bytes a trametre
    ucSt[3] = byLecturaBotons();
    vModeTxRxRS485(RS485Transmit);
    for (i = 0 ; i < 4 ; i++) {
      mySerial.write(ucSt[i]);
    }
    uiCRC = uiModRTU_CRC(ucSt, 4);
    mySerial.write((byte)(uiCRC & 0xFF));
    mySerial.write((byte)(uiCRC >> 8));
    vModeTxRxRS485(RS485Receive);
  }
}

void vLlegeixCoilsB(byte* ucSt, int len) {
  int i;
  unsigned int uiCRC, adr = (ucSt[2] << 8) | ucSt[3]; // Adreça d'inici dels coils
  // ucSt[4] i ucSt[5] diuen quants coils escriure
  // ucSt[6] diuen quants bytes de dades s'escriuen
  // ucSt[7] i ucSt[8] diuen quines dades s'escriuen
  // ucSt[9] i ucSt[10] diuen quin es el CRC
  if (adr == COIL_ADDRESS_B) {
    ucSt[2] = 0x02; // Bytes a trametre
    ucSt[3] = 0x00;
    ucSt[4] = byLecturaBotons();
    vModeTxRxRS485(RS485Transmit);
    for (i = 0 ; i < 5 ; i++) {
      mySerial.write(ucSt[i]);
    }
    uiCRC = uiModRTU_CRC(ucSt, 5);
    mySerial.write((byte)(uiCRC & 0xFF));
    mySerial.write((byte)(uiCRC >> 8));
    vModeTxRxRS485(RS485Receive);
  }
  if (adr == COIL_ADDRESS_WR) {
    ucSt[2] = 0x02; // Bytes a trametre
    ucSt[3] = 0x00;
    //ucSt[4] = byLecturaLeds();
    ucSt[4] = byLecturaBotons();
    vModeTxRxRS485(RS485Transmit);
    for (i = 0 ; i < 5 ; i++) {
      mySerial.write(ucSt[i]);
    }
    uiCRC = uiModRTU_CRC(ucSt, 5);
    mySerial.write((byte)(uiCRC & 0xFF));
    mySerial.write((byte)(uiCRC >> 8));
    vModeTxRxRS485(RS485Receive);
  }

}

void vEscriuCoilsB(byte* ucSt, int len) {
  int i;
  unsigned int uiCRC, adr = (ucSt[2] << 8) | ucSt[3]; // Adreça d'inici dels coils
  if (adr == COIL_ADDRESS_B) {
    digitalWrite(10, ucSt[5] & 0x08);
    digitalWrite(11, ucSt[5] & 0x04);
    digitalWrite(12, ucSt[5] & 0x02);
    digitalWrite(13, ucSt[5] & 0x01);
    vModeTxRxRS485(RS485Transmit);
    for (i = 0 ; i < 6 ; i++) {
      mySerial.write(ucSt[i]);
    }
    uiCRC = uiModRTU_CRC(ucSt, 6);
    mySerial.write((byte)(uiCRC & 0xFF));
    mySerial.write((byte)(uiCRC >> 8));
    vModeTxRxRS485(RS485Receive);
  }
}

void vEscriuCoils5(byte* ucSt, int len) {
  int i;
  unsigned int uiCRC, adr = (ucSt[2] << 8) | ucSt[3]; // Adreça d'inici dels coils

  if (adr == 0x0000) {
    digitalWrite(13, ucSt[4] == 0xFF);
  }
  if (adr == 0x0001) {
    digitalWrite(12, ucSt[4] == 0xFF);
  }
  if (adr == 0x0002) {
    digitalWrite(11, ucSt[4] == 0xFF);
  }
  if (adr == 0x0003) {
    digitalWrite(10, ucSt[4] == 0xFF);
  }
  vModeTxRxRS485(RS485Transmit);
  for (i = 0 ; i < len ; i++) { // echo
    mySerial.write(ucSt[i]);
  }
  vModeTxRxRS485(RS485Receive);
}

void vProcessa(byte* ucSt, int len) {
  if (ucSt[0] == MODULE_ADDRESS) {
    switch (ucSt[1]) {
      case 0x02: //Serial.println("Lectura a esclau");
        vLlegeixCoils(ucSt, len);
        break;
      case 0x03: //Serial.println("Lectura a esclau");
        vLlegeixCoilsB(ucSt, len);
        break;
      case 0x05: //Serial.println("Escriptura de coils");
        vEscriuCoils5(ucSt, len);
        break;
      case 0x06: //Serial.println("Escriptura a esclau");
        vEscriuCoilsB(ucSt, len);
        break;
      case 0x0F: //Serial.println("Escriptura a esclau");
        vEscriuCoils(ucSt, len);
        break;
        //default: //Serial.println("Funcio no implementada");
    }
  }
}

void loop() {
  int i, nCmpt = 0;
  unsigned int uiCRC;
  byte uiCrcL, uiCrcH;

  while (mySerial.available() > 0) {
    ucSt[nCmpt] = (unsigned char)mySerial.read();
    nCmpt++;
    delay(2);
  }
  if (nCmpt) {
#ifdef RS485
    Serial.print("He llegit aquests "); Serial.print(nCmpt); Serial.println(" bytes: ");
    for(i=0; i < nCmpt; i++){
      Serial.print(ucSt[i],HEX);Serial.print(" ");
    }
    Serial.println();
#endif
    uiCRC = uiModRTU_CRC(ucSt, nCmpt - 2);
    if ((byte)(uiCRC >> 8) == ucSt[nCmpt - 1] && (byte)(uiCRC & 0xFF) == ucSt[nCmpt - 2]) {
#ifdef RS485
      Serial.println("Trama amb CRC correcte");
#endif
      vProcessa(ucSt, nCmpt);
    } else {
#ifdef RS485
      Serial.println("Trama amb CRC incorrecte");
#endif
    }
  }
  nCmpt = 0;
}
