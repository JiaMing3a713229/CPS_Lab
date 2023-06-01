/* Lab6 只需選擇1歌加密演算，對DHT11加，上傳至Firebas，然後再由esp32讀取Firebase溫濕，並解密*/

#include <ArduinoJson.h>  //解析JSON格式資料
#include <WiFi.h>         //ESP32 WiFi Library
#include <Firebase_ESP_Client.h> // 引入Firebase相關library
#include <addons/TokenHelper.h>  // Provide the token generation process info.
#include <addons/RTDBHelper.h>   // Provide the RTDB payload printing info and other helper functions.
#include "DHT.h"  // DHT相關LIBRARY


// DHT相關配置
#define DHT_PIN 32
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

//JSON記憶體、變數配置
StaticJsonDocument<200> json_doc;
DeserializationError json_error;
const char* payload_tmeperature;
const char* payload_humidity;
String json_output = " ";

/* 1. 定義WiFi AP 的識別名稱及密碼 */
#define WIFI_SSID "ICLab"
#define WIFI_PASSWORD "ICLab41208"
/* 2. 定義Firebase WEB_KEY、DATA_URL和DATABASE 資料路徑*/
#define API_KEY "AIzaSyDL9Y4AuZFqdpcohFyK-doCqz04dbwUe_Y"
#define DATABASE_URL "https://esp32firebase-71531-default-rtdb.firebaseio.com/"
#define DATABASE_TEMP_PATH "dht11/temperature"
#define DATABASE_HUMI_PATH "dht11/humidity"
/* 3.定義FirebaseData、FirebaseAuth、和 FirebaseConfig*/
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
/* 4.用來判斷esp32是否成功訪問FireBase伺服器*/
bool isSignok = false;


/* 此函式用來WiFi初始化並連線基地台、熱點*/
void InitWiFi(){
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  Serial.println("");
  Serial.print("IP Address:");
  Serial.println(WiFi.localIP());
}
/*此函式初始化Firebase*/
void InitFireBase(){
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  if(Firebase.signUp(&config, &auth, "", "")){
    Serial.println("SignUp OK!");
    isSignok = true;
  }
  else{
    Serial.printf("%s\r\n", config.signer.signupError.message.c_str());
  }

  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

/* 此函式將字串資料寫入FireBase中*/
void UpdateRTDB(char* Path, String message){
  if(Firebase.RTDB.setString(&fbdo, Path, message)){
      Serial.print("--Successfully save to" + fbdo.dataPath());
      Serial.println("");
    }
  else{
    Serial.println("FAILED:" + fbdo.errorReason());
  }
}
/* 此函式從Database路徑中的資料讀取出，並return字串資料*/
String ReadRTDB(char* Path){
  String message = "";
  if(Firebase.RTDB.getString(&fbdo, Path)){
    // Serial.println("Read data from" + fbdo.dataPath());
    // Serial.println(fbdo.stringData());
    message = fbdo.stringData();
    return message;
  }
  else{
    // Serial.println("FAILED:" + fbdo.errorReason());
    message = "FAILED:" + fbdo.errorReason();
  }
  return message;
}
void setup() {

  Serial.begin(115200);
  InitWiFi();             //WiFi 初始化
  InitFireBase();         //Firebase 初始化
  dht.begin();            //dht11 初始化
  vTaskDelay(1000 / portTICK_PERIOD_MS);


}



void loop() {


  
  if(Firebase.ready() && isSignok){

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
  /*------------------------------------------------------------------------------*/

    
  //------------------------------可放置加密程式-----------------------------------

  /*------------------------------------------------------------------------------*/
    //上傳資料至Firebase RealTime database
    UpdateRTDB(DATABASE_TEMP_PATH, String(t));
    UpdateRTDB(DATABASE_HUMI_PATH, String(h));
    
    //將檔案以JSON格式上傳至Firebase需要使用FirebaseJson
    FirebaseJson json;
    json.setDoubleDigits(3);
    //將{"Temperature":String(t), "Humidity":String(h)}上傳至Firebase
    json.add("Temperature", String(t));
    json.add("Humidity", String(h));


    Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, "/DHT11/JSON", &json) ? "ok" : fbdo.errorReason().c_str());
    // Serial.printf("Get json... %s\n", ? fbdo.to<FirebaseJson>().raw() : fbdo.errorReason().c_str());
    if(Firebase.RTDB.getJSON(&fbdo, "/DHT11/JSON")){
      json_output = String(fbdo.to<FirebaseJson>().raw()).c_str();
      Serial.printf("讀取: %s \n", json_output.c_str());
    }
    else{
      Serial.println( fbdo.errorReason().c_str());
    }

    json_error = deserializeJson(json_doc, json_output);
    if (!json_error) {
      payload_tmeperature = json_doc["Temperature"];
      payload_humidity = json_doc["Humidity"];
      Serial.println( "json to string:" );
      Serial.printf("payload_tmeperature:%s \r\n",payload_tmeperature);
      Serial.printf("payload_humidity:%s \r\n",payload_humidity);
    }else{
      Serial.print("JSON 解析失败：");
      Serial.println(json_error.c_str());
    }
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
     //從Firebase RealTime database讀取資料
    Serial.println("讀取溫度" + ReadRTDB(DATABASE_TEMP_PATH));
    Serial.println("讀取濕度" + ReadRTDB(DATABASE_HUMI_PATH));
    Serial.println("-----------------------------------------------------------");
    /*------------------------------可放置解密程式-----------------------------------

    
    String t_text = ReadRTDB(DATABASE_TEMP_PATH);
    uint64_t cipher_t = atoi(t_text.c_str());


    ------------------------------------------------------------------------------*/
  }

  vTaskDelay(5000 / portTICK_PERIOD_MS);
  


}
