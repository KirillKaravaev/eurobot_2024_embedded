#include "locker.h"

void locker_irq_callback(uint gpio, uint32_t event) {
  if (gpio == LOCKER_PIN_UP)
    locker_state_up = !locker_state_up;
  else if (gpio == LOCKER_PIN_DOWN)
    locker_state_down = !locker_state_down;
}

void lockers_init() {
  gpio_init(LOCKER_PIN_UP);
  gpio_set_dir(LOCKER_PIN_UP, GPIO_IN);
  gpio_init(LOCKER_PIN_DOWN);
  gpio_set_dir(LOCKER_PIN_DOWN, GPIO_IN);

  uint32_t event_mask = GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE;
  gpio_set_irq_enabled_with_callback(LOCKER_PIN_UP, event_mask, true,
                                     locker_irq_callback);
  gpio_set_irq_enabled_with_callback(LOCKER_PIN_DOWN, event_mask, true,
                                     locker_irq_callback);
}
