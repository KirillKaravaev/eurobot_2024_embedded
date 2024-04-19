#include "servo.h"

/*https://www.webwork.co.uk/2023/06/raspberry-pi-pico-as-switching_21.html */

uint8_t servos[] = {SERVO_PWM_PIN_1, SERVO_PWM_PIN_2, SERVO_PWM_PIN_3,
                    SERVO_PWM_PIN_4, SERVO_PWM_PIN_5, SERVO_PWM_PIN_6};

void servo_set_angle(uint8_t number, int16_t angle) {
  if (number < SERVO_COUNT) {
    uint8_t pin = servos[number];
    pwm_set_gpio_level(pin, (uint16_t)(250 + angle * 5.55));
  }
}

void servo_init(uint8_t pin, uint16_t start_level_pwm) {
  // Выделяем пины PWM_PIN_i под генерацию ШИМА
  gpio_set_function(pin, GPIO_FUNC_PWM);

  // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
  uint slice_num = pwm_gpio_to_slice_num(pin);

  // Устанавливаем предделитель от тактовой частоты процессора (125[Мгц])
  pwm_set_clkdiv(slice_num, SERVO_PWM_DIV);

  // Устанавливаем частоту ШИМА равной 125[Мгц]/PWM_DIV/PWM_WRAP
  pwm_set_wrap(slice_num, SERVO_PWM_WRAP);

  // Задаем начальное заполнение шим 50%
  pwm_set_gpio_level(pin, start_level_pwm);

  // Включаем генерацию ШИМ
  pwm_set_enabled(slice_num, true);
}

// Функция инициализации ШИМ-выводов для управления сервоприводами
void servo_init_all() {
  // https://www.i-programmer.info/programming/hardware/14849-the-pico-in-c-basic-pwm.html?start=1

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    servo_init(i, 250);
  }
}
