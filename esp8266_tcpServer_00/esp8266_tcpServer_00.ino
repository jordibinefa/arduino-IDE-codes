#include <ESP8266WiFi.h>

const char* ssid     = "IoT-eCat";
const char* password = "clotClot";

/* create a server and listen on port 8088 */
WiFiServer server(8088);

void vDelayESP8266(unsigned long ulMilliseconds) {
  unsigned long ulPreviousMillis = millis();

  do {
    yield();
  } while (millis() - ulPreviousMillis <= ulMilliseconds);
}

void setup()
{
    Serial.begin(115200);
    Serial.print("Connecting to ");
    Serial.println(ssid);
    /* connecting to WiFi */
    WiFi.begin(ssid, password);
    /*wait until ESP32 connect to WiFi*/
    while (WiFi.status() != WL_CONNECTED) {
        vDelayESP8266(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected with IP address: ");
    Serial.println(WiFi.localIP());
    /* start Server */
    server.begin();
}
void loop(){
    /* listen for client */
    WiFiClient client = server.available(); 
    uint8_t data[30]; 
    if (client) {                   
      Serial.println("new client");         
      /* check client is connected */           
      while (client.connected()) {          
          if (client.available()) {
              int len = client.read(data, 30);
              if(len < 30){
                  data[len] = '\0';  
              }else {
                  data[30] = '\0';
              }    
              Serial.print("client sent: ");            
              Serial.println((char *)data); 
          }
      } 
    }
}
