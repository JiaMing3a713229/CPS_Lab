#include <Arduino.h>
#include <ArduinoJson.h>    
#include "DHT.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_SSD1306.h>

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


uint64_t ModularEXP(uint64_t a, uint64_t n, uint64_t m){  //return a^n (mod m)
  a = a % m;  
  uint64_t r = 1;
  while(n!=0){
    if((n & 1) == 1){
      r = (a % m)*(r % m) % m;
    }
    a = (a % m)*(a % m) % m ;
    n = n >> 1;
  }
  return r;
}
//擴展歐幾里得求模反元素的演算法  
uint64_t exgcd(uint64_t a, uint64_t b, uint64_t a1, uint64_t t1, uint64_t t2){
  if(b==0){
    return (t1 + a1) % a1;
  }
  uint64_t Q = int(a / b);
  uint64_t R = a % b;
  uint64_t t = t1 - (t2 * Q);
  t1 = t2;
  t2 = t;
  a = b;
  b = R;
  return exgcd(a, b, a1, t1, t2);
}
//求模反元素  
uint64_t InverseModular(uint64_t d,uint64_t lambda_n){
  uint64_t r = exgcd(lambda_n, d, lambda_n, 0, 1);
  return r;
}

// RSA Encryption:E(m)=m^e mod n,where e is gcd(e,lambda_n)=1,uint32_t m is message.
uint64_t Encrypt_RSA(uint32_t e, uint32_t n, uint32_t message){
  uint64_t cipher = ModularEXP(message, e, n);                              // m^e (mod n)
  return cipher;
}

// RSA Decryption:D(c)=m = c^d mod n
uint64_t Decrypt_RSA(uint64_t d, uint64_t cipher, uint64_t n){
  uint64_t plaintext = ModularEXP(cipher, d, n);
  return plaintext;
}

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

  String display_t = "Temp" + String(display_text3);
  display.setCursor(0,32);     			  //設定起始點位置(0,32)
  display.setTextSize(2);      				//設定文字尺寸為2
  display.println(display_t);    	//將"Temp: 23.5"存入RAM
  display.display();           				//顯示所設定文字
  
  String display_h = "Humi" + String(display_text4);
  display.setTextSize(2);      				//設定文字尺寸為2
  display.println(display_h);    	//將"Humi: 66.5"存入RAM
  display.display();           				//顯示所設定文字

}

const uint64_t key_p = 29;
const uint64_t key_q = 31;
const uint64_t n = key_p * key_q;
const uint64_t e = 113;
const uint64_t lambda_n = (key_p-1) * (key_q-1);
const uint64_t d = InverseModular(e, lambda_n);
int message = 25;

void setup() {
  Serial.begin(115200);
  Serial.printf("p is:%d", key_p);
  Serial.printf(" q is:%d", key_q);
  Serial.printf(" n is:%d", n);
  Serial.printf(" e is:%d", e);
  Serial.printf(" lambda_n is:%d", lambda_n);
  Serial.printf(" d is:%d \r\n", d);
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
  
  Serial.println("RSA test...");
  uint64_t cipher = Encrypt_RSA(e, n, message);
  uint64_t plaintext = Decrypt_RSA(d, cipher, n);

  Serial.printf("message:%d", message);
  Serial.printf(" ,cipher:%d", cipher);
  Serial.printf(" ,decrypted message:%d \r\n", plaintext);
  

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
 
  uint64_t cipher_t = Encrypt_RSA(e, n, t);
  vTaskDelay(100/ portTICK_PERIOD_MS);
  uint64_t cipher_h = Encrypt_RSA(e, n, h);
  Serial.printf("溫度加密後:%d \r\n", cipher_t);
  Serial.printf("濕度加密後:%d \r\n", cipher_h);
  Serial.printf("----------------------------- \r\n", h);

  //Temperature and humidity mapping to JSON
  json_doc["tmeperature"] = String(cipher_t);
  json_doc["humidity"] = String(cipher_h);
  serializeJson(json_doc, json_output);
  Serial.println( "string to json:" ); 
  Serial.println( json_output ); 
  
  client.publish(esp32_dht11, json_output);
  client.loop();

  json_error = deserializeJson(json_doc, msg_payload);
  if (!json_error) {
    payload_tmeperature = json_doc["tmeperature"];
    payload_humidity = json_doc["humidity"];
    
  }
  Serial.println( "json to string:" ); 
  Serial.printf("payload_tmeperature:%s \r\n",payload_tmeperature);
  Serial.printf("payload_humidity:%s \r\n",payload_humidity);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  UpdateOled("Student ID", "N96114093", payload_tmeperature, payload_humidity);
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  int t_cipher = atoi(payload_tmeperature);
  int h_cipher = atoi(payload_humidity);
  Serial.printf("t_cipher:%d", t_cipher);
  Serial.printf(", h_cipher:%d \r\n", h_cipher);
  //RSA解密
  uint64_t t_plaintext = Decrypt_RSA(d, t_cipher, n);
  uint64_t h_plaintext = Decrypt_RSA(d, h_cipher, n);
  Serial.printf("t_plaintext:%d", t_plaintext);
  Serial.printf(" ,h_plaintext:%d \r\n", h_plaintext);



  UpdateOled("N96114093", "Decrypting...", String(t_plaintext), String(h_plaintext));
  json_doc.clear();//釋放靜態記憶體
  
  
}
