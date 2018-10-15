/*
   Copyright (c) 2016 Boot&Work Corp., S.L.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <MDUINO-Relay.h>

/* IMPORTANT: SWITCHES configuration
COMMUNICATION SWITCH - A ZONE
  SCL: ON
  SDA: ON
  RX1: ON
  TX1: ON
  Pin3: ON
  Pin2: ON

DIGITAL/ANALOG OUT SWITCH - B ZONE
  Q0.2: ON
  Q0.1: ON
  Q0.0: ON

DIGITAL/ANALOG OUT SWITCH - C ZONE
  Q1.2: ON
  Q1.1: ON
  Q1.0: ON

DIGITAL/ANALOG OUT SWITCH - D ZONE
  Q2.2: ON
  Q2.1: ON
  Q2.0: ON
*/

String szMsg;

////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);

  szMsg = "";
  
  // Configure INPUT pins
  pinMode(I0_0, INPUT);
  pinMode(I0_1, INPUT);
  pinMode(I0_2, INPUT);
  pinMode(I0_3, INPUT);
  pinMode(I0_4, INPUT);
  pinMode(I0_5, INPUT);
  pinMode(I1_0, INPUT);
  pinMode(I1_1, INPUT);
  pinMode(I1_2, INPUT);
  pinMode(I1_3, INPUT);
  pinMode(I1_4, INPUT);
  pinMode(I1_5, INPUT);
  pinMode(I2_0, INPUT);
  pinMode(I2_1, INPUT);
  pinMode(I2_2, INPUT);
  pinMode(I2_3, INPUT);
  pinMode(I2_4, INPUT);
  pinMode(I2_5, INPUT);

  // Configure DIGITAL OUTPUT pins
  pinMode(Q0_0, OUTPUT);
  pinMode(Q0_1, OUTPUT);
  pinMode(Q0_2, OUTPUT);
  pinMode(Q1_0, OUTPUT);
  pinMode(Q1_1, OUTPUT);
  pinMode(Q1_2, OUTPUT);
  pinMode(Q2_0, OUTPUT);
  pinMode(Q2_1, OUTPUT);

  // Configure RELAY OUTPUT pins
  pinMode(R0_1, OUTPUT);
  pinMode(R0_2, OUTPUT);
  pinMode(R0_3, OUTPUT);
  pinMode(R0_4, OUTPUT);
  pinMode(R0_5, OUTPUT);
  pinMode(R0_6, OUTPUT);
  pinMode(R0_7, OUTPUT);
  pinMode(R0_8, OUTPUT);
  pinMode(R1_1, OUTPUT);
  pinMode(R1_2, OUTPUT);
  pinMode(R1_3, OUTPUT);
  pinMode(R1_4, OUTPUT);
  pinMode(R1_5, OUTPUT);
  pinMode(R1_6, OUTPUT);
  pinMode(R1_7, OUTPUT);
  pinMode(R1_8, OUTPUT);
  pinMode(R2_1, OUTPUT);
  pinMode(R2_2, OUTPUT);
  pinMode(R2_3, OUTPUT);
  pinMode(R2_4, OUTPUT);
  pinMode(R2_6, OUTPUT);
  pinMode(R2_7, OUTPUT);
  pinMode(R2_8, OUTPUT);
}

void vManageMsg(){
  boolean bDone = false;
  
  if(szMsg == "1h" || szMsg == "1H"){
    digitalWrite(R0_1, HIGH);
    Serial.println("R0_1 HIGH");
    bDone = true;
  }
  if(szMsg == "1l" || szMsg == "1L"){
    digitalWrite(R0_1, LOW);
    Serial.println("R0_1 LOW");
    bDone = true;
  }  
  if(szMsg == "2h" || szMsg == "2H"){
    digitalWrite(R0_2, HIGH);
    Serial.println("R2 HIGH");
    bDone = true;
  }
  if(szMsg == "2l" || szMsg == "2L"){
    digitalWrite(R0_2, LOW);
    Serial.println("R0_2 LOW");
    bDone = true;
  }  
  if(szMsg == "3h" || szMsg == "3H"){
    digitalWrite(R0_3, HIGH);
    Serial.println("R0_3 HIGH");
    bDone = true;
  }
  if(szMsg == "3l" || szMsg == "3L"){
    digitalWrite(R0_3, LOW);
    Serial.println("R0_3 LOW");
    bDone = true;
  }  
  if(szMsg == "4h" || szMsg == "4H"){
    digitalWrite(R0_4, HIGH);
    Serial.println("R0_4 HIGH");
    bDone = true;
  }
  if(szMsg == "4l" || szMsg == "4L"){
    digitalWrite(R0_4, LOW);
    Serial.println("R0_4 LOW");
    bDone = true;
  }
  if(!bDone){
    if(szMsg.length() <= 2){
      char szBuff[3]={0};
      szMsg.toCharArray(szBuff,szMsg.length()+1);
      int nHex = (int) strtol(szBuff, 0, 16);
      digitalWrite(R0_1, nHex & 0x01);
      digitalWrite(R0_2, nHex & 0x02);
      digitalWrite(R0_3, nHex & 0x04);
      digitalWrite(R0_4, nHex & 0x08);
      /*
      Serial.print("szMsg.length(): ");Serial.println(szMsg.length());
      Serial.print("szMsg: ");Serial.println(szMsg);
      Serial.print("szBuff: ");Serial.println(szBuff);
      Serial.print("strlen(szBuff): ");Serial.println(strlen(szBuff));
      Serial.print("nHex: ");Serial.println(nHex);
      */
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  while(Serial.available()){
    delay(3);
    char c = Serial.read();
    szMsg += c;
  }
  if(szMsg != "") vManageMsg();
  szMsg = "";
}
