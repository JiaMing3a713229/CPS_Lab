#include <Adafruit_SSD1306.h>


#define CREEN_WIDTH 128 		//設定OLED螢幕的長度像素
#define CREEN_HEIGHT 64  		//設定OLED螢幕的寬度像素
#define OLED_RESET -1 			 //Reset pin如果OLED上沒有RESET腳位,將它設置為-1

Adafruit_SSD1306 display(CREEN_WIDTH, CREEN_HEIGHT, &Wire, OLED_RESET);   	//OLED顯示器使用2C連線並宣告名為display物件
bool OLEDStatus = true;


void UpdataOled(){

  display.clearDisplay();      				//清除緩衝區資料

  display.setTextSize(2);      				//設定文字尺寸為1,1:6x8,2:12x16,3:18x24...etc
  display.setCursor(0, 0);     			  //設定起始點位置(0,0)
  display.setTextColor(WHITE);   	    //黑字白底
  display.println("student ID") ;   	//將"學號"存入SSD1306 RAM


  display.setCursor(0,16);     			  //設定起始點位置(0,16)
  display.setTextSize(2);     				//設定文字尺寸為2
  display.setTextColor(WHITE); 			  //設定文字顏色為白色(亮點)
  display.println("N9611409X");    		//將"N9611409X"存入RAM

  display.setCursor(0,32);     			  //設定起始點位置(0,32)
  display.setTextSize(2);      				//設定文字尺寸為2
  display.println("Temp: 23.5");    	//將"Temp: 23.5"存入RAM
  display.display();           				//顯示所設定文字

  display.setTextSize(2);      				//設定文字尺寸為2
  display.println("Humi: 66.5");    	//將"Humi: 66.5"存入RAM
  display.display();           				//顯示所設定文字

}

void setup() {

  Serial.begin(115200);
                                                            //初始化SSD1306 OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC,0x3c)) {     	    //設定位址為 0x3c
    Serial.println(F("SSD1306 allocation falled"));		  	  //F(字串):將字串儲存在fash並非在RAM
    OLEDStatus = false;  								                    //開啟OLED失敗
  }

  UpdataOled();

  

}

void loop() {
  // put your main code here, to run repeatedly:

}
