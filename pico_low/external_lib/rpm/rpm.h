#include "pico/stdlib.h"
#include <cstdio>

#define IMP_CNT_PIN1 20
#define IMP_CNT_PIN2 2
#define IMP_CNT_PIN3 17
#define IMP_CNT_PIN4 21

#define CNT_DELAY_MS 200
#define ENC_MULTIPLIER 6 // Число импульсов энкодера на оборот

// Колбэк-функции. Из-за специфики инициализации не полчается включить в
// структуру, но оно и не надо, в данном случае это служебные функции
bool repeating_timer_callback(struct repeating_timer *t);

struct rpm {
  int rpm1;
  int rpm2;
  int rpm3;
  int rpm4;
};

struct imp_num {
  int imp_num1;
  int imp_num2;
  int imp_num3;
  int imp_num4;
};

void impulse_counter_init();

extern rpm current_rpm;
extern imp_num current_imp_num;
extern struct repeating_timer rpm_timer;
