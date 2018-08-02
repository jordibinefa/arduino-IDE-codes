// Exemple de subscriptor i publicador MQTTS
//
// https://wiki.binefa.cat/index.php?title=Instal%C2%B7laci%C3%B3_de_MQTT_a_un_servidor_remot_basat_en_Debian_9
// https://wiki.binefa.cat/index.php?title=Configuraci%C3%B3_de_MQTTS_a_un_servidor_remot_basat_en_Debian_9
//
// Basat en http://www.iotsharing.com/2017/08/how-to-use-esp32-mqtts-with-mqtts-mosquitto-broker-tls-ssl.html
//
// wiki.binefa.cat - 20180802
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>

/* change it with your ssid-password */
const char* ssid = "yourWifiSsid";
const char* password = "yourWifiPassword";
const char* secureBroker = "broker.electronics.cat"; // Your MQTTS broker
int nBrokerPort = 8883; // Usually 8883

const char* mqtt_user = "mqttsUser";
const char* mqtt_password = "mqttsPwd";

const char* ca_cert = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIC8DCCAlmgAwIBAgIJAOD63PlXjJi8MA0GCSqGSIb3DQEBBQUAMIGQMQswCQYD\n" \
"VQQGEwJHQjEXMBUGA1UECAwOVW5pdGVkIEtpbmdkb20xDjAMBgNVBAcMBURlcmJ5\n" \
"MRIwEAYDVQQKDAlNb3NxdWl0dG8xCzAJBgNVBAsMAkNBMRYwFAYDVQQDDA1tb3Nx\n" \
"dWl0dG8ub3JnMR8wHQYJKoZIhvcNAQkBFhByb2dlckBhdGNob28ub3JnMB4XDTEy\n" \
"MDYyOTIyMTE1OVoXDTIyMDYyNzIyMTE1OVowgZAxCzAJBgNVBAYTAkdCMRcwFQYD\n" \
"VQQIDA5Vbml0ZWQgS2luZ2RvbTEOMAwGA1UEBwwFRGVyYnkxEjAQBgNVBAoMCU1v\n" \
"c3F1aXR0bzELMAkGA1UECwwCQ0ExFjAUBgNVBAMMDW1vc3F1aXR0by5vcmcxHzAd\n" \
"BgkqhkiG9w0BCQEWEHJvZ2VyQGF0Y2hvby5vcmcwgZ8wDQYJKoZIhvcNAQEBBQAD\n" \
"gY0AMIGJAoGBAMYkLmX7SqOT/jJCZoQ1NWdCrr/pq47m3xxyXcI+FLEmwbE3R9vM\n" \
"rE6sRbP2S89pfrCt7iuITXPKycpUcIU0mtcT1OqxGBV2lb6RaOT2gC5pxyGaFJ+h\n" \
"A+GIbdYKO3JprPxSBoRponZJvDGEZuM3N7p3S/lRoi7G5wG5mvUmaE5RAgMBAAGj\n" \
"UDBOMB0GA1UdDgQWBBTad2QneVztIPQzRRGj6ZHKqJTv5jAfBgNVHSMEGDAWgBTa\n" \
"d2QneVztIPQzRRGj6ZHKqJTv5jAMBgNVHRMEBTADAQH/MA0GCSqGSIb3DQEBBQUA\n" \
"A4GBAAqw1rK4NlRUCUBLhEFUQasjP7xfFqlVbE2cRy0Rs4o3KS0JwzQVBwG85xge\n" \
"REyPOFdGdhBY2P1FNRy0MDr6xr+D2ZOwxs63dG1nnAnWZg7qwoLgpZ4fESPD3PkA\n" \
"1ZgKJc2zbSQ9fCPxt2W3mdVav66c6fsb7els2W2Iz7gERJSX\n" \
"-----END CERTIFICATE-----\n";

/* create an instance of WiFiClientSecure */
WiFiClientSecure espClient;
PubSubClient client(espClient);

/*LED GPIO pin*/
const char led = 2; // ESP32 DEVKIT V1 internal LED
const char button = 0; // ESP32 DEVKIT V1 boot button

/* topics */
#define COUNTER_TOPIC    "/JORDI/thing00/counter"
#define LED_TOPIC     "/JORDI/thing00/led"
#define BUTTON_TOPIC     "/JORDI/thing00/button"

long lastMsg = 0;
char msg[20];
int counter = 0;

void receivedCallback(char* topic, byte* payload, unsigned int length) {
  String szTopic = String(topic);
  Serial.print("Topic: ");
  Serial.println(topic);

  Serial.print("payload: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  //Serial.print("Topic: "); Serial.println(szTopic);
  if (szTopic == LED_TOPIC) {
    /* we got '1' -> on */
    if ((char)payload[0] == '1') {
      digitalWrite(led, HIGH);
    } else {
      /* we got '0' -> on */
      digitalWrite(led, LOW);
    }
  }
}

void mqttconnect() {
  /* Loop until reconnected */
  while (!client.connected()) {
    Serial.print("MQTT connecting ...");
    /* client ID */
    String clientId = "ESP32Client";
    /* connect now */
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      //if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      /* subscribe topic */
      client.subscribe(LED_TOPIC);
    } else {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  /* set led as output to control led on-off */
  pinMode(led, OUTPUT);
  /* set button as input */
  pinMode(button, INPUT);

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  /* set SSL/TLS certificate */
  espClient.setCACert(ca_cert);
  /* configure the MQTT server with IPaddress and port */
  client.setServer(secureBroker, nBrokerPort);
  /* this receivedCallback function will be invoked
    when client received subscribed topic */
  client.setCallback(receivedCallback);

}
void loop() {
  boolean bButtonState = !digitalRead(button);
  static boolean bLastButtonState = bButtonState;

  /* if client was disconnected then try to reconnect again */
  if (!client.connected()) {
    mqttconnect();
  }
  /* this function will listen for incomming
    subscribed topic-process-invoke receivedCallback */
  client.loop();
  /* we increase counter every 3 secs
    we count until 3 secs reached to avoid blocking program if using delay()*/
  long now = millis();
  if (now - lastMsg > 3000) {
    lastMsg = now;
    if (counter < 100) {
      counter++;
      snprintf (msg, 20, "%d", counter);
      /* publish the message */
      client.publish(COUNTER_TOPIC, msg);
    } else {
      counter = 0;
    }
  }
  if (bButtonState != bLastButtonState) {
    bLastButtonState = bButtonState;
    delay(50);
    if (bButtonState)
      client.publish(BUTTON_TOPIC, "HIGH");
    else
      client.publish(BUTTON_TOPIC, "LOW");
  }
}
