#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define WIFI_SSID "ICLab"
#define WIFI_PASS "ICLab41208"
#define MQTT_SERVER "broker.emqx.io"
#define MQTT_PORT 1883
#define MQTT_ID "N96114093"

WiFiClient ESPClient;
PubSubClient client(ESPClient);

const char* esp32_dht11 = "cps/esp32/dht11";
String msg_payload;
int32_t mqtt_index = 0;

void callback(char* topic, byte* message, unsigned int length){
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  msg_payload = messageTemp;
  Serial.println();
}
void reconnect() {
  while(!client.connected()){
    if (client.connect(MQTT_ID)) {
      client.subscribe(esp32_dht11); // Subscribe to channel.
    }
  }
}

void InitWiFi(){
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while(WiFi.status() != WL_CONNECTED){
    vTaskDelay(200 / portTICK_PERIOD_MS);
    Serial.print(".");
  }
  Serial.println("連線成功");
  Serial.println(WiFi.localIP());
}

void InitMQTT(){
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);
  vTaskDelay(500 / portTICK_PERIOD_MS);
}

void setup() {
  
  Serial.begin(115200);

  InitWiFi();
  InitMQTT();
  

}

void loop() {
  if(!client.connected()){
        reconnect();
  }
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  client.publish(esp32_dht11, "N96114093");
  client.loop();
}