#include <Arduino.h>
#include <WiFi.h>
// 引入Firebase相關library
#include <Firebase_ESP_Client.h>
// Provide the token generation process info.
#include <addons/TokenHelper.h>
// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>
// DHT相關LIBRARY
#include "DHT.h"
#define DHT_PIN 32
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);





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

    //發送資料至Firebase RealTime database
    UpdateRTDB(DATABASE_TEMP_PATH, String(t));
    UpdateRTDB(DATABASE_HUMI_PATH, String(h));

    vTaskDelay(10 / portTICK_PERIOD_MS);
     //從Firebase RealTime database讀取資料
    Serial.println("從"+ fbdo.dataPath() + "讀取溫度" + ReadRTDB(DATABASE_TEMP_PATH));
    Serial.println("從"+ fbdo.dataPath() + "讀取濕度" + ReadRTDB(DATABASE_HUMI_PATH));
    Serial.println("-----------------------------------------------------------");
  }

  vTaskDelay(5000 / portTICK_PERIOD_MS);
  


}
