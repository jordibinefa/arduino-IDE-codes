/*
 *  This sketch sends random data over UDP on a ESP32 device
 *
 */
#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi network name and password:
const char * networkName = "IoT-eCat";
const char * networkPswd = "clotClot";

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "192.168.1.17";
const int udpPortTx = 3333;
const int udpPortRx = 3334;

const int nRxPacketSize = 55;
byte byPacketRxBuffer[nRxPacketSize];

//Are we currently connected?
boolean connected = false;

//The udp library class
WiFiUDP udpClient,udpServer;

void setup(){
  // Initilize hardware serial:
  Serial.begin(115200);
  
  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);
}

void loop(){
  //only send data when connected
  if(connected){
    //Send a packet
    udpClient.beginPacket(udpAddress,udpPortTx);
    udpClient.printf("Seconds since boot: %u", millis()/1000);
    udpClient.endPacket();
    vHandleUDPServer();
  }
  //Wait for 1 second
  delay(1000);
}

void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udpClient.begin(WiFi.localIP(),udpPortTx);
          connected = true;
          udpServer.begin(udpPortRx);
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
    }
}

void vHandleUDPServer() {
  int cb = udpServer.parsePacket();
  if (cb) {
    udpServer.read(byPacketRxBuffer, nRxPacketSize);
    String myData = ""; 
    for(int i = 0; i < nRxPacketSize; i++) {
      myData += (char)byPacketRxBuffer[i];
    }
    Serial.println(myData);
  }
}
