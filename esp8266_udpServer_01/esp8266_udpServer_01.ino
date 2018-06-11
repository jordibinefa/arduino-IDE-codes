#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define GPIO5 5
#define GPIO4 4
#define GPIO0 0
#define GPIO2 2
#define GPIO15 15
#define GPIO13 13
#define GPIO12 12
#define GPIO14 14
#define GPIO16 16

extern "C" {
#include "user_interface.h"
}

int status = WL_IDLE_STATUS;

const char* ssid = "IoT-eCat";
const char* pass = "clotClot";

unsigned int udpPortRx = 3334;
unsigned int udpPortTx = 3333;

byte packetBuffer[512]; //buffer to hold incoming and outgoing packets

WiFiUDP Udp;

void vSendUdp(String sz, unsigned int uiLocalPort) {
  Udp.beginPacket(Udp.remoteIP(), uiLocalPort);
  Udp.print(sz);
  Udp.endPacket();
}

bool bManageMsg(String szMsg) {
  unsigned int nLdr = analogRead(A0);
  bool bUnderstood = false;

  if (szMsg == "a" || szMsg == "A") {
    Serial.print("LDR level (0..1024): ");
    Serial.println(nLdr);
    vSendUdp(String(nLdr), udpPortTx);
    bUnderstood = true;
  }
  if (szMsg == "i" || szMsg == "I") {
    if (digitalRead(GPIO4)){
      Serial.println("Button is not pressed: GPIO4 HIGH");
      vSendUdp("H", udpPortTx);
    }else{
      Serial.println("Button is pressed: GPIO4 LOW");
      vSendUdp("L", udpPortTx);
    }
    bUnderstood = true;
  }
  if (szMsg == "5h" || szMsg == "5H") {
    digitalWrite(GPIO5, HIGH);
    Serial.println("GPIO5 HIGH");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "5l" || szMsg == "5L") {
    digitalWrite(GPIO5, LOW);
    Serial.println("GPIO5 LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "4h" || szMsg == "4H") {
    digitalWrite(GPIO4, HIGH);
    Serial.println("GPIO4 HIGH");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "4l" || szMsg == "4L") {
    digitalWrite(GPIO4, LOW);
    Serial.println("GPIO4 LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "0h" || szMsg == "0H") {
    digitalWrite(GPIO0, HIGH);
    Serial.println("GPIO0 HIGH");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "0l" || szMsg == "0L") {
    digitalWrite(GPIO0, LOW);
    Serial.println("GPIO0 LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "2h" || szMsg == "2H") {
    digitalWrite(GPIO2, HIGH);
    Serial.println("GPIO2 HIGH");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "2l" || szMsg == "2L") {
    digitalWrite(GPIO2, LOW);
    Serial.println("GPIO2 LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "15h" || szMsg == "15H") {
    digitalWrite(GPIO15, HIGH);
    Serial.println("GPIO15 HIGH -> Red ON");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "15l" || szMsg == "15L") {
    digitalWrite(GPIO15, LOW);
    Serial.println("GPIO15 LOW -> Red OFF");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "13h" || szMsg == "13H") {
    digitalWrite(GPIO13, HIGH);
    Serial.println("GPIO13 HIGH -> Blue ON");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "13l" || szMsg == "13L") {
    digitalWrite(GPIO13, LOW);
    Serial.println("GPIO13 LOW -> Blue OFF");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "12h" || szMsg == "12H") {
    digitalWrite(GPIO12, HIGH);
    Serial.println("GPIO12 HIGH -> Green ON");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "12l" || szMsg == "12L") {
    digitalWrite(GPIO12, LOW);
    Serial.println("GPIO12 LOW -> Green OFF");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "14h" || szMsg == "14H") {
    digitalWrite(GPIO14, HIGH);
    Serial.println("GPIO14 HIGH");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "14l" || szMsg == "14L") {
    digitalWrite(GPIO14, LOW);
    Serial.println("GPIO14 LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "16h" || szMsg == "16H") {
    digitalWrite(GPIO16, HIGH);
    Serial.println("GPIO16 HIGH");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "16l" || szMsg == "16L") {
    digitalWrite(GPIO16, LOW);
    Serial.println("GPIO16 LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  
  return bUnderstood;
}

void setup(){
  pinMode(GPIO2, OUTPUT); // Internal blue led
  pinMode(GPIO15, OUTPUT); // Red of RGB led
  pinMode(GPIO13, OUTPUT); // Blue of RGB led
  pinMode(GPIO12, OUTPUT); // Green of RGB led

  Serial.begin(115200);

  WiFi.begin(ssid, pass);

  Serial.print("[Connecting]");
  Serial.print(ssid);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    tries++;
    if (tries > 30) {
      break;
    }
  }
  Serial.println();

  printWifiStatus();

  Serial.println("Connected to wifi");
  Serial.print("Udp server started at port ");
  Serial.println(udpPortRx);
  Udp.begin(udpPortRx);
}

void loop(){
  int noBytes = Udp.parsePacket();
  String received_command = "";

  if ( noBytes ) {
    Serial.print(millis() / 1000);
    Serial.print(":Packet of ");
    Serial.print(noBytes);
    Serial.print(" received from ");
    Serial.print(Udp.remoteIP());
    Serial.print(":");
    Serial.println(Udp.remotePort());
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, noBytes); // read the packet into the buffer

    // display the packet contents in HEX
    for (int i = 1; i <= noBytes; i++) {
      Serial.print(packetBuffer[i - 1], HEX);
      received_command = received_command + char(packetBuffer[i - 1]);
      if (i % 32 == 0) {
        Serial.println();
      }
      else Serial.print(' ');
    } // end for
    Serial.println();

    if(! bManageMsg(received_command)){
      vSendUdp("I don't understand this command: ", udpPortTx);
      vSendUdp(received_command, udpPortTx);
    }
    Serial.println(received_command);
    Serial.println();
  } 
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi ESP8266's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}
