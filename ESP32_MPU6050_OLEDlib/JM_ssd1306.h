#ifndef _JM_ssd1306_H_
#define _JM_ssd1306_H_

#include "Arduino.h"

/*SSD1306 Command List*/
#define SSD1306_MEMORYMODE 0x20          ///< See datasheet
#define SSD1306_COLUMNADDR 0x21          ///< See datasheet
#define SSD1306_PAGEADDR 0x22            ///< See datasheet
#define SSD1306_SETCONTRAST 0x81         ///< See datasheet
#define SSD1306_CHARGEPUMP 0x8D          ///< See datasheet
#define SSD1306_SEGREMAP 0xA0            ///< See datasheet
#define SSD1306_DISPLAYALLON_RESUME 0xA4 ///< See datasheet
#define SSD1306_DISPLAYALLON 0xA5        ///< Not currently used
#define SSD1306_NORMALDISPLAY 0xA6       ///< See datasheet
#define SSD1306_INVERTDISPLAY 0xA7       ///< See datasheet
#define SSD1306_SETMULTIPLEX 0xA8        ///< See datasheet
#define SSD1306_DISPLAYOFF 0xAE          ///< See datasheet
#define SSD1306_DISPLAYON 0xAF           ///< See datasheet
#define SSD1306_COMSCANINC 0xC0          ///< Not currently used
#define SSD1306_COMSCANDEC 0xC8          ///< See datasheet
#define SSD1306_SETDISPLAYOFFSET 0xD3    ///< See datasheet
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5  ///< See datasheet
#define SSD1306_SETPRECHARGE 0xD9        ///< See datasheet
#define SSD1306_SETCOMPINS 0xDA          ///< See datasheet
#define SSD1306_SETVCOMDETECT 0xDB       ///< See datasheet

#define SSD1306_SETLOWCOLUMN 0x00       ///< Not currently used
#define SSD1306_SETHIGHCOLUMN 0x10      ///< Not currently used
#define SSD1306_SETSTARTLINE 0x40       ///< See datasheet
#define I2C_MASTER_TIMEOUT_MS 1000

// #define SSD1306_SLAVE_ADDR 0x3C

#define CMD_MODE 0
#define DATA_MODE 1

typedef enum{
    FONT_SIZE_F6x8 = 0,
    FONT_SIZE_F8X16 = 1
}font_size;


class JM_ssd1306{
    
    
    public:
        JM_ssd1306(uint8_t ssd1306_slave_addr);
        void OLED_Init(void);
        esp_err_t OLED_WR_Byte(uint8_t data, uint8_t cmd_);
        void OLED_Set_Pos(uint8_t x, uint8_t y);
        void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size);
        void OLED_Clear(void);
        void oled_print(uint8_t start_x, uint8_t start_y, const char* display_data, uint8_t font_size);
        void oled_draw_xaxis(uint8_t x, uint8_t y);
        void oled_draw_yaxis(uint8_t x, uint8_t y);

        uint8_t SSD1306_SLAVE_ADDR;
};


























#endif  //_JM_ssd1306_H_