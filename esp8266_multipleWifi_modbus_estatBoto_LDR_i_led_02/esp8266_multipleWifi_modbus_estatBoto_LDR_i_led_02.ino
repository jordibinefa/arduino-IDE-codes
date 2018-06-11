/*
  20180523 - wiki.binefa.cat

  Based on Modbus-Arduino Example - Test Holding Register (Modbus IP ESP8266)
  Read Switch Status on pin GPIO0
  Copyright by André Sarmento Barbosa
  http://github.com/andresarmento/modbus-arduino

*/

#include <ESP8266WiFi.h>
#include <Modbus.h>
#include <ModbusIP_ESP8266.h>

#define N_WIFIS 2
#define MAX_STRING_SIZE 15

struct stWifiList {
  String szSSID;
  String szPWD;
};

struct stWifiList stWiFi[N_WIFIS] = {
  {"IoT-eCat" , "clotClot"},
  {"JESUITESFP" , "internetcoses"}
};


//Modbus Registers Offsets (0-9999)
const int SENSOR_IREG = 100;
const int SWITCH_ISTS = 100;
const int LED_COIL = 100;
const int LED_VERMELL_COIL = 101;

//Used Pins
const int switchPin = 4; //GPIO4
const int ledPin = 2; //GPIO2
const int ledVermell = 15; //GPIO15

//ModbusIP object
ModbusIP mb;

long ts;

void vDelayESP8266(unsigned long ulMilliseconds) {
  unsigned long ulPreviousMillis = millis();

  do {
    yield();
  } while (millis() - ulPreviousMillis <= ulMilliseconds);
}

boolean bIsListed(String szSSID, int *pnWhichOne) {
  for (int i = 0; i < N_WIFIS ; i++) {
    if (stWiFi[i].szSSID == szSSID){
      *pnWhichOne = i;
      return true;
    }
  }
  return false;
}

bool bConnectModbus() {
  //mb.config("IoT-eCat", "clotClot");
  char ssid[MAX_STRING_SIZE],  pwd[MAX_STRING_SIZE];
  int n = WiFi.scanNetworks(), nWhichOne;

  Serial.print("*");
  if (n == 0) {
    Serial.println("\nNo networks found");
    vDelayESP8266(1000);
  } else {
    for (int i = 0; i < n; ++i) {
      if (bIsListed(WiFi.SSID(i), &nWhichOne)) {
        String szSsid = stWiFi[nWhichOne].szSSID;
        szSsid.toCharArray(ssid, szSsid.length() + 1);
        String szPwd = stWiFi[nWhichOne].szPWD;
        szPwd.toCharArray(pwd, szPwd.length() + 1);

        mb.config(ssid,pwd);
        return true;
      }
    }
  }
  return false;
}

void setup() {
  int nSsidDetected;
  Serial.begin(115200);


  //Config Modbus IP
  //mb.config("IoT-eCat", "clotClot");
  bConnectModbus();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  pinMode(ledPin, OUTPUT);
  mb.addCoil(LED_COIL);
  pinMode(ledVermell, OUTPUT);
  mb.addCoil(LED_VERMELL_COIL);

  //Set switchPin mode
  pinMode(switchPin, INPUT);
  // Add SWITCH_ISTS register - Use addIsts() for digital inputs
  mb.addIsts(SWITCH_ISTS);

  // Add SENSOR_IREG register - Use addIreg() for analog Inputs
  mb.addIreg(SENSOR_IREG);

  ts = millis();
}

void loop() {
  //Call once inside loop() - all magic here
  mb.task();

  //Attach switchPin to SWITCH_ISTS register
  mb.Ists(SWITCH_ISTS, digitalRead(switchPin));

  //Read each two seconds
  if (millis() > ts + 2000) {
    ts = millis();
    //Setting raw value (0-1024)
    mb.Ireg(SENSOR_IREG, analogRead(A0));
  }

  //Attach ledPin to LED_COIL register
  digitalWrite(ledPin, mb.Coil(LED_COIL));
  digitalWrite(ledVermell, mb.Coil(LED_VERMELL_COIL));
  //digitalWrite(ledVermell,digitalRead(switchPin));
}
