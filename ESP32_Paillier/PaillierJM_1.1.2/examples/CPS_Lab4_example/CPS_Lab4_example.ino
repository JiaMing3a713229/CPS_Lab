#include <Arduino.h>
#include <ArduinoJson.h>    
#include "DHT.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_SSD1306.h>
#include "PaillierJM.h"

#define CREEN_WIDTH 128 		//設定OLED螢幕的長度像素
#define CREEN_HEIGHT 64  		//設定OLED螢幕的寬度像素
#define OLED_RESET -1 			 //Reset pin如果OLED上沒有RESET腳位,將它設置為-1
Adafruit_SSD1306 display(CREEN_WIDTH, CREEN_HEIGHT, &Wire, OLED_RESET);   	//OLED顯示器使用i2C連線並宣告名為display物件
bool OLEDStatus = true;

#define WIFI_SSID "ICLab"
#define WIFI_PASS "ICLab41208"
#define MQTT_SERVER "broker.emqx.io"
#define MQTT_PORT 1883
#define MQTT_ID "N96114093"
#define DHT_PIN 32
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);
WiFiClient ESPClient;
PubSubClient client(ESPClient);

const char* esp32_dht11 = "cps/esp32/dht11";
String msg_payload;
int32_t mqtt_index = 0;



StaticJsonDocument<250> json_doc;
char json_output[100];
DeserializationError json_error;
const char* payload_tmeperature = "";
const char* payload_humidity = "";

uint64_t p = 29;
uint64_t q = 31;

Paillier paillier(p, q);


void callback(char* topic, byte* message, unsigned int length){
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    // Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  msg_payload = messageTemp;
  Serial.println(msg_payload);

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

void InitMQTT(void){
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);
  vTaskDelay(500 / portTICK_PERIOD_MS);
}

void UpdateOled(String display_text1, String display_text2, String display_text3, String display_text4){

  display.clearDisplay();      				//清除緩衝區資料

  display.setTextSize(2);      				//設定文字尺寸為1,1:6x8,2:12x16,3:18x24...etc
  display.setCursor(0, 0);     			  //設定起始點位置(0,0)
  display.setTextColor(WHITE);   	    //黑字白底
  display.println(display_text1) ;   	//將"學號"存入SSD1306 RAM


  display.setCursor(0,16);     			  //設定起始點位置(0,16)
  display.setTextSize(2);     				//設定文字尺寸為2
  display.setTextColor(WHITE); 			  //設定文字顏色為白色(亮點)
  display.println(display_text2);    		//將"N9611409X"存入RAM

  String display_t = "T:" + String(display_text3);
  display.setCursor(0,32);     			  //設定起始點位置(0,32)
  display.setTextSize(2);      				//設定文字尺寸為2
  display.println(display_t);    	//將"Temp: 23.5"存入RAM
  display.display();           				//顯示所設定文字
  
  String display_h = "H:" + String(display_text4);
  display.setTextSize(2);      				//設定文字尺寸為2
  display.println(display_h);    	//將"Humi: 66.5"存入RAM
  display.display();           				//顯示所設定文字

}


void setup() {
  Serial.begin(115200);
  
  //dht11初始化
  dht.begin();
  //wifi 初始化
  InitWiFi();
  //mqtt 初始化
  InitMQTT();

  vTaskDelay(1000 / portTICK_PERIOD_MS);
  //display.begin(SSD1306_SWITCHCAPVCC,0x3c) Oled ssd1306初始化
  if(!display.begin(SSD1306_SWITCHCAPVCC,0x3c)) {     	         //設定位址為 0x3c
    Serial.println(F("SSD1306 allocation falled"));		  	       //F(字串):將字串儲存在fash並非在RAM
  OLEDStatus = false;  								                           //開啟OLED失敗
  }
  
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  
  

}

void loop() {

  if(!client.connected()){
        reconnect();
  }

  vTaskDelay(2000/ portTICK_PERIOD_MS);
  
  // Read humidity as Celsius (the default)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  Serial.printf("temperature: %f ", t);
  Serial.printf("Humidity: %f  \r\n", h);
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
 
  // uint64_t cipher_t = Encrypt_RSA(e, n, t);
  String cipher_t = paillier.EncryptionPaillier2Str(int(t));
  vTaskDelay(100/ portTICK_PERIOD_MS);
  // uint64_t cipher_h = Encrypt_RSA(e, n, h);
  String cipher_h = paillier.EncryptionPaillier2Str(int(h));
  // Serial.printf("溫度加密後:%s \r\n", cipher_t.c_str());
  // Serial.printf("濕度加密後:%s \r\n", cipher_h.c_str());
  // Serial.printf("----------------------------- \r\n", h);

  //Temperature and humidity mapping to JSON
  json_doc["tmeperature"] = cipher_t;
  json_doc["humidity"] = cipher_h;
  serializeJson(json_doc, json_output);
  // Serial.print( "string to json:" ); 
  // Serial.println( json_output ); 
  
  client.publish(esp32_dht11, json_output);
  client.loop();

  json_error = deserializeJson(json_doc, msg_payload);
  if (!json_error) {
    payload_tmeperature = json_doc["tmeperature"];
    payload_humidity = json_doc["humidity"];
    
  }
  // Serial.println( "json to string:" ); 
  // Serial.printf("payload_tmeperature:%s, ",payload_tmeperature);
  // Serial.printf("payload_humidity:%s \r\n",payload_humidity);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  UpdateOled("Student ID", "N96114093", payload_tmeperature, payload_humidity);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // int t_cipher = atoi(payload_tmeperature);
  // int h_cipher = atoi(payload_humidity);
  // Serial.printf("t_cipher:%d", t_cipher);
  // Serial.printf(", h_cipher:%d \r\n", h_cipher);
  // //Paillier解密
  String t_plaintext = paillier.DecryptionPaillier2str(payload_tmeperature , p , q);
  String h_plaintext = paillier.DecryptionPaillier2str(payload_humidity , p , q);
  // Serial.printf("t_plaintext:%s", t_plaintext.c_str());
  // Serial.printf(" ,h_plaintext:%s \r\n", h_plaintext.c_str());



  UpdateOled("N96114093", "Decrypting...", String(t_plaintext), String(h_plaintext));
  json_doc.clear();//釋放靜態記憶體
  
  
}
