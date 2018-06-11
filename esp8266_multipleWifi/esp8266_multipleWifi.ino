#include "ESP8266WiFi.h"

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

boolean bWifiConnected = false;

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

void vConnectToWiFi(String szSsid, String szPwd) {
  char ssid[MAX_STRING_SIZE],  pwd[MAX_STRING_SIZE];

  szSsid.toCharArray(ssid, szSsid.length() + 1);
  szPwd.toCharArray(pwd, szPwd.length() + 1);

  Serial.println("Connecting to WiFi network: " + String(ssid));

  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
  while (WiFi.status() != WL_CONNECTED /*&& i++ < 20*/) {
    Serial.print(".");
    vDelayESP8266(500);
  }
  Serial.println();
}

boolean bTryWifiConnection() {
  int n = WiFi.scanNetworks(), nWhichOne;

  Serial.print("*");
  if (n == 0) {
    Serial.println("\nNo networks found");
    vDelayESP8266(1000);
  } else {
    for (int i = 0; i < n; ++i) {
      if (bIsListed(WiFi.SSID(i), &nWhichOne)) {
        vConnectToWiFi(stWiFi[nWhichOne].szSSID, stWiFi[nWhichOne].szPWD);
        return true;
      }
    }
  }
  return false;
}

void vShowWifiConnection() {
  static int nCmpt = 0;

  Serial.print(nCmpt++);
  Serial.print(") ");
  Serial.print(WiFi.localIP());
  Serial.print(" at ");
  Serial.println(WiFi.SSID());
}

void setup() {
  Serial.begin(115200);

  do {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    vDelayESP8266(100);
  } while (!bTryWifiConnection());
}

void loop() {
  vShowWifiConnection();
  vDelayESP8266(2000);
}

