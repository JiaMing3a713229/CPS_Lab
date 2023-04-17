#include<Wire.h>
#include<Adafruit_GFX.h>
#include<Adafruit_SSD1306.h>
#define CREEN_WIDTH 128 		//設定OLED螢幕的寬度像素
#define CREEN_HEIGHT 64  		//設定OLED螢幕的寬度像素
#define OLED_RESET -1 			 //Reset pin如果OLED上沒有RESET腳位,將它設置為-1

Adafruit_SSD1306 display(CREEN_WIDTH, CREEN_HEIGHT, &Wire, OLED_RESET);   	//OLED顯示器使用2C連線並宣告名為display物件
bool OLEDStatus = true;
void setup() {
  Serial.begin(9600);
  if(!display.begin(SSD1306_SWITCHCAPVCC,0x3c)) {     	 //設定位址為 0x3c
    Serial.println(F("SSD1306 allocation falled"));		  	 //F(字串):將字串儲存在fash並非在RAM
    OLEDStatus = false;  								 //開啟OLED失敗
  }
  
  if(OLEDStatus == true)
    testdrawchar();       //Draw characters of the default font
}

void loop() {
}

void testdrawchar(void) {
  display.clearDisplay();      				//清除緩衝區資料
  display.setTextSize(2);      				//設定文字尺寸為1,1:6x8,2:12x16,3:18x24...etc
  display.setCursor(0, 0);     			//設定起始點位置(0,0)
  display.setTextColor(WHITE);   	//黑字白底
  display.println("DHT11") ;   			//將"Hello"存入RAM


  display.setCursor(0,16);     			//設定起始點位置(0,16)
  display.setTextSize(2);     				//設定文字尺寸為2
  display.setTextColor(WHITE); 			//設定文字顏色為白色(亮點)
  display.println("T: 23.5");    			//將"Hello"存入RAM

  display.setTextSize(2);      				//設定文字尺寸為3
  display.println("H: 23.5");    			//將"Hello"存入RAM
  display.display();           				//顯示所設定文字
}