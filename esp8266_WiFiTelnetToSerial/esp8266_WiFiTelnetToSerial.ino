/* 
  WiFiTelnetToSerial.ino

  20170114 - www.electronics.cat / www.binefa.cat/blog
*/
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

//how many clients should be able to telnet to this ESP8266
#define MAX_SRV_CLIENTS 1
#define UDP_PORT_RX 45454
#define UDP_PORT_TX 45455
#define UDP_PORT_TX_ASYNC_SENSOR 45456
#define TCP_PORT 14000
#define BUTTON_PIN 4
#define IR_DETECTION_PIN 5
#define EXTERNAL_RESET_PIN 14
#define RED_RGB_LED 15
#define GREEN_RGB_LED 13
#define BLUE_RGB_LED 12
#define RPi_ADDRESS 192, 168, 42, 1
#define TIMEOUT_PERSON_DETECTION 60000
const char* ssid = "IoT-eCat";
const char* password = "clotClot";

WiFiServer server(TCP_PORT); // by default is 23
WiFiClient serverClients[MAX_SRV_CLIENTS];
WiFiUDP UDP;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];

void vDelayESP8266(unsigned long ulMilliseconds) {
  unsigned long ulPreviousMillis = millis();

  do {
    yield();
  } while (millis() - ulPreviousMillis <= ulMilliseconds);
}


void setup() {
  Serial.begin(57600);

  pinMode(0,OUTPUT);  // b1
  pinMode(2,OUTPUT);  // b2
  
  pinMode(IR_DETECTION_PIN,INPUT_PULLUP);  // External IR sensor digital input (5)
  pinMode(BUTTON_PIN,INPUT);         // button pin (4)
  pinMode(RED_RGB_LED,OUTPUT);       // red led (15)
  pinMode(GREEN_RGB_LED,OUTPUT);       // green led (13)
  pinMode(BLUE_RGB_LED,OUTPUT);       // blue led (12)
  pinMode(EXTERNAL_RESET_PIN,OUTPUT);  // External reset pin (14)
  digitalWrite(EXTERNAL_RESET_PIN,LOW);
  delay(2);
  
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  digitalWrite(GREEN_RGB_LED,LOW);
  digitalWrite(RED_RGB_LED,HIGH);
  for(;;){
    while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
    if(i >= 21){
      Serial.print("Could not connect to"); Serial.println(ssid);
      i = 0;
    }else{
      break;
    }
  }
  digitalWrite(GREEN_RGB_LED,HIGH);
  digitalWrite(RED_RGB_LED,LOW);
  //start the server
  server.begin();
  server.setNoDelay(true);
  
  Serial.print("Ready! Use 'telnet ");
  Serial.print(WiFi.localIP());
  Serial.print(" ");
  Serial.print(TCP_PORT);
  Serial.println("' to connect");
  vTrametTramaUdp(UDP_PORT_TX);
  UDP.begin(UDP_PORT_RX);
}

void writeUdp(IPAddress a, String stringData, int nUdpPort) {
  int i;

  for (i = 0; i < stringData.length(); i++) {
    packetBuffer[i] = stringData[i];   // Push each char 1 by 1 on each loop pass
  }
  packetBuffer[i] = '\0';
  UDP.beginPacket(a, nUdpPort);
  UDP.write(packetBuffer);
  UDP.endPacket();
}

String szCheckSsidNetwork(String szSsid){
  String szResposta = "";
  int n = WiFi.scanNetworks(false,true);
  
  for (int i = 0; i < n; ++i){
      if(WiFi.SSID(i) == szSsid){
        szResposta += WiFi.RSSI(i);
        szResposta += WiFi.channel(i);
        return szResposta;
      }
  }
  return "NO";
}

void vTrametTramaUdp(int nUdpPort){
  IPAddress ipRPi(RPi_ADDRESS);

  String sz = WiFi.localIP().toString() + ";" + String(TCP_PORT);
  sz += ";" + WiFi.macAddress();
  sz += ";" + String(WiFi.RSSI()) + ";" + String(WiFi.channel());
  Serial.println(sz);
  writeUdp(ipRPi,sz,nUdpPort);
}

void vTrametTramaUdpSensor(int nUdpPort){
  IPAddress ipRPi(RPi_ADDRESS);

  String sz = WiFi.localIP().toString() + ";" + String(TCP_PORT);
  sz += ";" + WiFi.macAddress();
  sz += ";" + String(WiFi.RSSI()) + ";" + String(WiFi.channel());
  if(digitalRead(IR_DETECTION_PIN)){
    sz += ";GPIO5 HIGH";
  }else{
    sz += ";GPIO5 LOW";    
  }
  Serial.println(sz);
  writeUdp(ipRPi,sz,nUdpPort);
}

void vTrametTramaUdpSensorState(int nUdpPort,bool bState){
  IPAddress ipRPi(RPi_ADDRESS);

  String sz = WiFi.localIP().toString() + ";" + String(TCP_PORT);
  sz += ";" + WiFi.macAddress();
  sz += ";" + String(WiFi.RSSI()) + ";" + String(WiFi.channel());
  if(bState){
    sz += ";H";
  }else{
    sz += ";L";    
  }
  Serial.println(sz);
  writeUdp(ipRPi,sz,nUdpPort);
}

void vReadUdp(){
  int packetSize = UDP.parsePacket();
  if (packetSize)  {
    //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, UDP.remoteIP().toString().c_str(), UDP.remotePort());
    int len = UDP.read(packetBuffer, 255);
    if (len > 0){
      packetBuffer[len] = 0;
      String szUdpRx(packetBuffer);
      if(szUdpRx == "Hola")
        vTrametTramaUdp(UDP_PORT_TX);
      if(szUdpRx == "RstS"){
        digitalWrite(EXTERNAL_RESET_PIN,HIGH);
        vDelayESP8266(1200);
        digitalWrite(EXTERNAL_RESET_PIN,LOW);
      }
      if(szUdpRx == "senS")
        vTrametTramaUdpSensor(UDP_PORT_TX);
    }
  }
}

void loop() {
  static bool bLastButtonState = HIGH,bLastSensorState = LOW,bLastPersonDetection = LOW, bPersonDetection = LOW,bPreviousWifi = true;
  static long int lnLastTimePersonDetected; 
  bool bButtonState,bSensorState;
  uint8_t i;
  
  //check if there are any new clients
  if (server.hasClient()){
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      //find free/disconnected spot
      if (!serverClients[i] || !serverClients[i].connected()){
        if(serverClients[i]) serverClients[i].stop();
        serverClients[i] = server.available();
        //Serial1.print("New client: "); Serial1.print(i);
        Serial.print("New client: "); Serial.print(i);
        continue;
      }
    }
    //no free/disconnected spot so reject
    WiFiClient serverClient = server.available();
    serverClient.stop();
  }
  //check clients for data
  for(i = 0; i < MAX_SRV_CLIENTS; i++){
    if (serverClients[i] && serverClients[i].connected()){
      if(serverClients[i].available()){
        //get data from the telnet client and push it to the UART
        while(serverClients[i].available()) Serial.write(serverClients[i].read());
      }
    }
  }
  //check UART for data
  if(Serial.available()){
    size_t len = Serial.available();
    uint8_t sbuf[len];
    Serial.readBytes(sbuf, len);
    //push UART data to all connected telnet clients
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      if (serverClients[i] && serverClients[i].connected()){
        serverClients[i].write(sbuf, len);
        delay(1);
      }
    }
  }
  bButtonState = digitalRead(BUTTON_PIN);
  if (bButtonState != bLastButtonState) {
    if (bButtonState == LOW) {
      vTrametTramaUdp(UDP_PORT_TX);
      delay(10);
    }
  }
  bSensorState = digitalRead(IR_DETECTION_PIN);
  digitalWrite(BLUE_RGB_LED,bSensorState);
  if (bSensorState != bLastSensorState) {
    if(bSensorState == HIGH){
      bPersonDetection = true;
    }
    if(bSensorState == LOW){
      lnLastTimePersonDetected = millis();
    }
  }
  bLastSensorState = bSensorState;

  if(bSensorState == LOW && bPersonDetection){
    if(lnLastTimePersonDetected + TIMEOUT_PERSON_DETECTION < millis()){
      bPersonDetection = false;
    }
  }

  if(bPersonDetection != bLastPersonDetection){
    vTrametTramaUdpSensorState(UDP_PORT_TX_ASYNC_SENSOR,bPersonDetection);
    bLastPersonDetection = bPersonDetection; 
  }
  
  if(WiFi.localIP()){
    digitalWrite(GREEN_RGB_LED,HIGH);
    digitalWrite(RED_RGB_LED,LOW);
    if(!bPreviousWifi)
      vTrametTramaUdp(UDP_PORT_TX);
    bPreviousWifi = true;
  }else{
    digitalWrite(GREEN_RGB_LED,LOW);
    digitalWrite(RED_RGB_LED,HIGH);
    bPreviousWifi = false;
  }
  vReadUdp();
}

