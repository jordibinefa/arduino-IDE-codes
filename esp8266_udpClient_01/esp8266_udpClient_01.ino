#include <ESP8266WiFi.h>
#include <WiFiUdp.h>


#ifdef ESP8266
extern "C" {
  #include "user_interface.h"
}
#endif

const char* ssid     = "IoT-eCat";
const char* password = "clotClot";

const char * udpAddress = "192.168.1.17";
const int udpPortTx = 3333;

WiFiUDP Udp;

void setup() {
  Serial.begin(115200);
  delay(10);

  Serial.print("\n\r\n\rConnecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  Udp.beginPacket(udpAddress, udpPortTx);  
}

void vGetMac(char *temp){
  static int i = 0;
  
#ifdef ESP8266
  uint8_t macaddr[6];   // array that will receive the MAC address from the chip
  wifi_get_macaddr(0x00, macaddr);
  os_sprintf(temp, "%05d) %02x:%02x:%02x:%02x:%02x:%02x\r\n", i, macaddr[0],
      macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]);
#endif

  i = (++i >= 0) ? i : 0;
}

void loop() {
  char temp[28];      // buffer for formatting the output (xx:xx:xx:xx:xx:xx\r\n) = 20 bytes

  vGetMac(temp);

  Udp.write(temp);
  Udp.endPacket();

  Serial.print(temp);
  Serial.println(" sent to UDP server");
    
  delay(5000);
}

