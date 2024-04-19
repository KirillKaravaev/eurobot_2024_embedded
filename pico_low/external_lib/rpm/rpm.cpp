#include "rpm.h"

void impulse_count_callback(uint gpio, uint32_t event) {
  if (gpio == IMP_CNT_PIN1)
    current_imp_num.imp_num1++;
  if (gpio == IMP_CNT_PIN2)
    current_imp_num.imp_num2++;
  if (gpio == IMP_CNT_PIN3)
    current_imp_num.imp_num3++;
  if (gpio == IMP_CNT_PIN4)
    current_imp_num.imp_num4++;
}

bool repeating_timer_callback(struct repeating_timer *t) {
  current_rpm.rpm1 =
      (current_imp_num.imp_num1 * 60000) / (ENC_MULTIPLIER * CNT_DELAY_MS);
  // 60000 = 60*1000 в знаменателе - для перехода от
  // микросекунд к минутам.(получаем об/мин)
  current_rpm.rpm2 =
      (current_imp_num.imp_num2 * 60000) / (ENC_MULTIPLIER * CNT_DELAY_MS);
  current_rpm.rpm3 =
      (current_imp_num.imp_num3 * 60000) / (ENC_MULTIPLIER * CNT_DELAY_MS);
  current_rpm.rpm4 =
      (current_imp_num.imp_num4 * 60000) / (ENC_MULTIPLIER * CNT_DELAY_MS);
  current_imp_num.imp_num1 = 0;
  current_imp_num.imp_num2 = 0;
  current_imp_num.imp_num3 = 0;
  current_imp_num.imp_num4 = 0;
  printf("rpm1 = %d, rpm2 = %d, rpm3 = %d, rpm4 = %d\n", current_rpm.rpm1,
         current_rpm.rpm2, current_rpm.rpm3, current_rpm.rpm4);
  return true;
}

void impulse_counter_init() {
  // Инициализируем пины счетчика импульса
  gpio_init(IMP_CNT_PIN1);
  gpio_set_dir(IMP_CNT_PIN1, GPIO_IN);
  gpio_init(IMP_CNT_PIN2);
  gpio_set_dir(IMP_CNT_PIN2, GPIO_IN);
  gpio_init(IMP_CNT_PIN3);
  gpio_set_dir(IMP_CNT_PIN3, GPIO_IN);
  gpio_init(IMP_CNT_PIN4);
  gpio_set_dir(IMP_CNT_PIN4, GPIO_IN);

  // Задаем частоту, с которой будет вычисляться частота обращения
  // моторов(вызываться обработчик набранного числа импульсов). Она может
  // отличаться от частоты считывания, результаты заносятся в структуру rpm
  add_repeating_timer_ms(CNT_DELAY_MS, &repeating_timer_callback, nullptr,
                         &rpm_timer);

  // Активируем внешние прерывания на пинах по
  uint32_t event_mask = GPIO_IRQ_EDGE_FALL;
  gpio_set_irq_enabled_with_callback(IMP_CNT_PIN1, event_mask, true,
                                     &impulse_count_callback);
  gpio_set_irq_enabled_with_callback(IMP_CNT_PIN2, event_mask, true,
                                     &impulse_count_callback);
  gpio_set_irq_enabled_with_callback(IMP_CNT_PIN3, event_mask, true,
                                     &impulse_count_callback);
  gpio_set_irq_enabled_with_callback(IMP_CNT_PIN4, event_mask, true,
                                     &impulse_count_callback);

  // Обнуляем значения структуры rpm
}
