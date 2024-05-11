//https :  // github.com/JamesH65/pico-examples/tree/ssd1306
//#include "SSD1306.h"
#include <stdio.h>

#define SSD1306_I2C_CLK 400
#define I2C_PORT i2c0
#define SDA 8
#define SCL 9

class DISPLAY {
    public:
        void I2C_init();
        void display_init();
        void Write_score(uint x, uint y, char ch);
};