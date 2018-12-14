#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266HTTPClient.h>
#include "wifiCredentials.h"
#include "sentiloCredentials.h"

//  WiFi{
void vDelayESP(unsigned long ulMilliseconds) {
  unsigned long ulPreviousMillis = millis();

  do {
    yield();
  } while (millis() - ulPreviousMillis <= ulMilliseconds);
}

void vConnectToWiFi(const char* szSsid, const char* szPwd) {
  char ssid[MAX_STRING_SIZE],  pwd[MAX_STRING_SIZE];

  Serial.println("Connecting to WiFi network: " + String(szSsid) + ", pwd: " + String(szPwd));

  WiFi.begin(szSsid, szPwd);

  Serial.println("Waiting for WIFI connection...");
  while (WiFi.status() != WL_CONNECTED /*&& i++ < 20*/) {
    Serial.print(".");
    vDelayESP(500);
  }
  Serial.println();
}

boolean bIsListed(String szSSID, int *pNwO) {
  for (int i = 0; i < N_WIFIS ; i++) {
    if (String(stWiFi[i].szSSID) == szSSID) {
      *pNwO = i;
      return true;
    }
  }
  return false;
}

boolean bTryWifiConnection() {
  int n = WiFi.scanNetworks(), nWhichOne;

  //Serial.print("*");
  if (n == 0) {
    Serial.println("\nNo networks found");
    vDelayESP(1000);
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

void setup_wifi() {
  do {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    vDelayESP(100);
  } while (!bTryWifiConnection());
  IPAddress ip = WiFi.localIP();
  Serial.print("Connected to wifi at "); Serial.println(ip);
}

// }WiFi


// Sentilo{
#define CLIENT_DEBUG_PRINT Serial.print

WiFiClient client;
int num_headers;
const char* headers[10];
const char* contentType;

int putMethod(const char* path, const char* body, String* response) {
  return request("PUT", path, body, response);
}

void write(const char* string) {
  Serial.print("write: ");
  CLIENT_DEBUG_PRINT(string);
  client.write(string);
}

int request(const char* method, const char* path,
            const char* body, String* response) {

  unsigned int nIdKeyMsgLen = (unsigned)strlen(apiKey) + 14 + 2 + 1;
  unsigned int nPutLineLen = (unsigned)(strlen(path) + strlen(body)) + 15 + 1;
  unsigned int nMax = (nIdKeyMsgLen > nPutLineLen) ? nIdKeyMsgLen : nPutLineLen;
  char szMsg[nMax]; // enough room for maximum string

  if (client.connect(ip, port)) {
    sprintf(szMsg, "PUT %s%s HTTP/1.1\r\n", path, body);
    write(szMsg);
    sprintf(szMsg, "%s: %s\r\n", IDENTITY_KEY_HEADER, apiKey);
    write(szMsg);
    sprintf(szMsg, "Host: %s:%d\r\n", ip, port);
    write(szMsg);
    sprintf(szMsg, "Content-Length: %d\r\n", strlen(body));
    write(szMsg);
    //sprintf(szMsg, "Content-Type: application/json\r\n");
    //write(szMsg);
    sprintf(szMsg, "\r\n%s\r\n\r\n", body);
    write(szMsg);

    write("Connection: close\r\n");
    //make sure you write all those bytes.
    delay(100);

    CLIENT_DEBUG_PRINT("HTTP: call readResponse\n");
    int statusCode = readResponse(response);
    CLIENT_DEBUG_PRINT("HTTP: return readResponse\n");

    //cleanup
    CLIENT_DEBUG_PRINT("HTTP: stop client\n");
    num_headers = 0;
    client.stop();
    delay(50);
    CLIENT_DEBUG_PRINT("HTTP: client stopped\n");

    return statusCode;
  } else {
    CLIENT_DEBUG_PRINT("HTTP Connection failed\n");
    return 0;
  }
}

int readResponse(String* response) {
  // an http request ends with a blank line
  boolean currentLineIsBlank = true;
  boolean httpBody = false;
  boolean inStatus = false;

  char statusCode[4];
  int i = 0;
  int code = 0;

  if (response == NULL) {
    CLIENT_DEBUG_PRINT("HTTP: NULL RESPONSE POINTER: \n");
  } else {
    CLIENT_DEBUG_PRINT("HTTP: NON-NULL RESPONSE POINTER: \n");
  }

  CLIENT_DEBUG_PRINT("HTTP: RESPONSE: \n");
  while (client.connected()) {
    //CLIENT_DEBUG_PRINT(".");

    if (client.available()) {
      //CLIENT_DEBUG_PRINT(",");

      char c = client.read();
      CLIENT_DEBUG_PRINT(c);

      if (c == ' ' && !inStatus) {
        inStatus = true;
      }

      if (inStatus && i < 3 && c != ' ') {
        statusCode[i] = c;
        i++;
      }
      if (i == 3) {
        statusCode[i] = '\0';
        code = atoi(statusCode);
      }

      if (httpBody) {
        //only write response if its not null
        if (response != NULL) response->concat(c);
      }
      else
      {
        if (c == '\n' && currentLineIsBlank) {
          httpBody = true;
        }

        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        }
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
  }

  CLIENT_DEBUG_PRINT("HTTP: return readResponse3\n");
  return code;
}

void setHeader(const char* header) {
  headers[num_headers] = header;
  num_headers++;
}

void vPublicaSentilo(char *buf) {
  int statusCode = -1;
  String response = "";
  char path[150];
  sprintf(path, "%s/%s/%s/", DATA_BASE_PATH, providerId, sensorId);

  Serial.println("[loop] Publishing a sample observation...");

  // Publish the observation to Sentilo Platform
  //statusCode = sentiloClient.publishObservation(providerId, sensorId, observation, apiKey, response);
  statusCode = putMethod(path, buf, &response);

  // Read response status and show an error if it is necessary
  if (statusCode != 0 && statusCode != 200) {
    Serial.print("[loop] [ERROR] Status code from server after publish the observations: ");
    Serial.println(statusCode);
    Serial.print("[loop] [ERROR] Response body from server after publish the observations: ");
    Serial.println(response);
  } else
    Serial.println("[loop] Sample observation published!");
  Serial.println("[loop] Program ended");

}

// } SENTILO

void setup(void) {
  Serial.begin(115200);

  setup_wifi();
}

void loop() {
  char buf[8] = "";
  float fLdr = (100.*(float)analogRead(A0)) / 1024;
  
  sprintf(buf, "%.2f", fLdr);
  //char buf[] = "32.1";
  vPublicaSentilo(buf);
  vDelayESP(900000);
}
