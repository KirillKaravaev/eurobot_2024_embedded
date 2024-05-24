#include <stdlib.h>

#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "stepperV2.h"

/*https://www.webwork.co.uk/2023/06/raspberry-pi-pico-as-switching_21.html */


void STEPPER::move(int number, int direction, int angle ){
    if ((number == 1) && gpio_get(STEPPER_OVERHEAT_PIN_1) && gpio_get(STEPPER_OVERHEAT_PIN_2)) { //Сразу проверяем перегрев драйверов.
        gpio_put(STEPPER_ENABLE_PIN_1, 0);  // Включаем работу драйвера для поворота кассетницы
        if (direction == 1){
            gpio_put(STEPPER_DIR_PIN_1, 1); 
        }
        else gpio_put(STEPPER_DIR_PIN_1, 0);

        //Устанавливаем заполнение импульса 50% на необходимое для проворота на определенный градус время, а далее выставляем заполнение в 0, вращение прекращается
        pwm_set_gpio_level(STEPPER_PWM_PIN_1, 500);
        sleep_ms((angle / (1.8 * STEPPER_SPEED)) * 1000);
        pwm_set_gpio_level(STEPPER_PWM_PIN_1, 0);
        gpio_put(STEPPER_ENABLE_PIN_1, 1);  // Отключаем работу драйвера после окончания поворота (чтобы минимизировать нагрев драйвера)

    } else if ((number == 2) && gpio_get(STEPPER_OVERHEAT_PIN_3) && gpio_get(STEPPER_OVERHEAT_PIN_4)) {  // Вторая группа моторов должна оставаться включенной, чтобы обеспечивать удержание ромашки в поднятом состоянии
        gpio_put(STEPPER_ENABLE_PIN_2, 0); // Включаем работу драйвера для подъема ромашки(он и так включен, но после перегрева отключается)
        if (direction == 1){
            gpio_put(STEPPER_DIR_PIN_2, 1); 
        }
        else gpio_put(STEPPER_DIR_PIN_2, 0);

        pwm_set_gpio_level(STEPPER_PWM_PIN_2, 500);
        sleep_ms((angle / (1.8 * STEPPER_SPEED)) * 1000);
        pwm_set_gpio_level(STEPPER_PWM_PIN_2, 0);
    }
}

int STEPPER::overheat() {
    if (((!gpio_get(STEPPER_OVERHEAT_PIN_1)) || (!gpio_get(STEPPER_OVERHEAT_PIN_2))) && ((!gpio_get(STEPPER_OVERHEAT_PIN_3)) || (!gpio_get(STEPPER_OVERHEAT_PIN_4)))) {
        gpio_put(STEPPER_ENABLE_PIN_1, 1);  // Отключаем работу драйверов при перегреве (он отключается автоматически, но на всякий случай)
        gpio_put(STEPPER_ENABLE_PIN_2, 1);  // Отключаем работу драйверов при перегреве (он отключается автоматически, но на всякий случай)
        return 3;
    }
    else if ( (!gpio_get(STEPPER_OVERHEAT_PIN_1)) || (!gpio_get(STEPPER_OVERHEAT_PIN_2))){
        gpio_put(STEPPER_ENABLE_PIN_1, 1);  // Отключаем работу драйверов при перегреве (он отключается автоматически, но на всякий случай)
        return 1;
    }
    else if ( (!gpio_get(STEPPER_OVERHEAT_PIN_3)) || (!gpio_get(STEPPER_OVERHEAT_PIN_4))) {
        gpio_put(STEPPER_ENABLE_PIN_2, 1);  // Отключаем работу драйверов при перегреве (он отключается автоматически, но на всякий случай)
        return 2;
    } 
    
    else {
        return 0;
    }
}

//Функция инициализации ШИМ-выводов для управления скоростью и пинов для управления направлением вращения
void STEPPER::pwm_init() {
    //https://www.i-programmer.info/programming/hardware/14849-the-pico-in-c-basic-pwm.html?start=1
    

    //Выделяем пины STEPPER_PWM_PIN_i под генерацию ШИМА 
    gpio_set_function(STEPPER_PWM_PIN_1, GPIO_FUNC_PWM);
    gpio_set_function(STEPPER_PWM_PIN_2, GPIO_FUNC_PWM);
    
    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num1 = pwm_gpio_to_slice_num(STEPPER_PWM_PIN_1);
    uint slice_num2 = pwm_gpio_to_slice_num(STEPPER_PWM_PIN_2);
    
    //Устанавливаем предделитель от тактовой частоты процессора (125[Мгц])
    pwm_set_clkdiv(slice_num1, STEPPER_PWM_DIV);
    pwm_set_clkdiv(slice_num2, STEPPER_PWM_DIV);
    
    //Устанавливаем частоту ШИМА равной 125[Мгц]/STEPPER_PWM_DIV/PWM_WRAP
    pwm_set_wrap(slice_num1, STEPPER_PWM_WRAP);
    pwm_set_wrap(slice_num2, STEPPER_PWM_WRAP);

    //Задаем начальное заполнение шим 50%
    pwm_set_gpio_level(STEPPER_PWM_PIN_1, 0);
    pwm_set_gpio_level(STEPPER_PWM_PIN_2, 0);

    //Включаем генерацию ШИМ
    pwm_set_enabled(slice_num1, true);
    pwm_set_enabled(slice_num2, true);
}

void STEPPER::direction_init() {
    // Инициализируем пины для управления направлением вращения
    gpio_init(STEPPER_DIR_PIN_1);
    gpio_init(STEPPER_DIR_PIN_2);

    gpio_set_dir(STEPPER_DIR_PIN_1, GPIO_OUT);
    gpio_set_dir(STEPPER_DIR_PIN_2, GPIO_OUT);

    // Выделяем пины для управления включением-выключением драйверов моторов ромашки
    gpio_init(STEPPER_ENABLE_PIN_1);
    gpio_init(STEPPER_ENABLE_PIN_2);

    gpio_set_dir(STEPPER_ENABLE_PIN_1, GPIO_OUT);
    gpio_set_dir(STEPPER_ENABLE_PIN_2, GPIO_OUT);
}

void STEPPER::overheat_init() {
    // Инициализируем пины для контроля состояния драйвера
    gpio_init(STEPPER_OVERHEAT_PIN_1);
    gpio_init(STEPPER_OVERHEAT_PIN_2);
    gpio_init(STEPPER_OVERHEAT_PIN_3);
    gpio_init(STEPPER_OVERHEAT_PIN_4);

    gpio_set_dir(STEPPER_OVERHEAT_PIN_1, GPIO_IN);
    gpio_set_dir(STEPPER_OVERHEAT_PIN_2, GPIO_IN);
    gpio_set_dir(STEPPER_OVERHEAT_PIN_3, GPIO_IN);
    gpio_set_dir(STEPPER_OVERHEAT_PIN_4, GPIO_IN);
}

void STEPPER::init() {
    direction_init();
    pwm_init();
    overheat_init();
    switch_init();

    // Изначально кассетница и ромашка отключены
    gpio_put(STEPPER_ENABLE_PIN_1, 1);
    gpio_put(STEPPER_ENABLE_PIN_2, 1);
}

void STEPPER::switch_irq_callback(uint gpio, uint32_t event) {
    if (gpio == LOCKER_PIN_UP)
        locker_state_up = !locker_state_up;
    else if (gpio == LOCKER_PIN_DOWN)
        locker_state_down = !locker_state_down;
}

void STEPPER::switch_init() {
    gpio_init(SWITCH_PIN_UP);
    gpio_set_dir(SWITCH_PIN_UP, GPIO_IN);
    gpio_init(SWITCH_PIN_DOWN);
    gpio_set_dir(SWITCH_PIN_DOWN, GPIO_IN);

    uint32_t event_mask = GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE;
    gpio_set_irq_enabled_with_callback(SWITCH_PIN_UP, event_mask, true,
                                       switch_irq_callback);
    gpio_set_irq_enabled_with_callback(SWITCH_PIN_DOWN, event_mask, true,
                                       switch_irq_callback);
}