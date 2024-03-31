#include <stdio.h>

#define IMP_CNT_PIN1 11
#define IMP_CNT_PIN2 12
#define IMP_CNT_PIN3 13
#define IMP_CNT_PIN4 14

#define CNT_DELAY_MS 200
#define ENC_MULTIPLIER 12 // Число импульсов энкодера на оборот

// Колбэк-функции. Из-за специфики инициализации не полчается включить в структуру, но оно и не надо, в данном случае это служебные функции
void impulse_counter1_callback(uint gpio, uint32_t event);
void impulse_counter2_callback(uint gpio, uint32_t event);
void impulse_counter3_callback(uint gpio, uint32_t event);
void impulse_counter4_callback(uint gpio, uint32_t event);
bool repeating_timer_callback(struct repeating_timer *t);

struct rpm
{
    int rpm1;
    int rpm2;
    int rpm3;
    int rpm4;
};

struct imp_num
{
    int imp_num1;
    int imp_num2;
    int imp_num3;
    int imp_num4;
};

void impulse_counter_init();
