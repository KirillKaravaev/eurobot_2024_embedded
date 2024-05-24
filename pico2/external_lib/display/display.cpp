
#include "display.h"

#include "SSD1306.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
// useful information for picotool
// bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
// bi_decl(bi_program_description("SSD1306 OLED driver I2C example for the Raspberry Pi Pico"));

// printf("Hello, SSD1306 OLED display! Look at my raspberries..\n");

// I2C is "open drain", pull ups to keep signal high when no data is being sent
SSD1306 ssd1306;

void DISPLAY::I2C_init() {
    i2c_init(I2C_PORT, SSD1306_I2C_CLK * 1000);
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_pull_up(SCL);
    gpio_pull_up(SDA);
}

void DISPLAY::display_init() {
    I2C_init();
    ssd1306.initialise();  //если не облачить все функции  SSD1306 в класс, то не получится здесь использовать их. Мы же просто объявляем класс здесь и используекм его функции
}

// На вход функция принимает целочисленную переменную, в функции sprintf(str, "%02u", ch) преобразует в массив символов и передает адрес первого элемента
// в функцию WriteBigString
void DISPLAY::Print_string(uint x, uint y, uint ch) {
    sprintf(str, "%02u", ch);
    ssd1306.ClearDisplay();
    ssd1306.WriteBigString(x, y, str);
    ssd1306.UpdateDisplay();
}

void DISPLAY::Print_char(uint x, uint y, char ch) {
    ssd1306.UpdateDisplay();
    ssd1306.WriteBigChar(x, y, ch);
}