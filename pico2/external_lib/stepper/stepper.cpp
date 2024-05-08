#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "stepper.h"
#include <stdlib.h>



/*https://www.webwork.co.uk/2023/06/raspberry-pi-pico-as-switching_21.html */

void stepper(int number, int direction, int angle ){
    if(number == 1){
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

    }
    else{                                      //Вторая группа моторов должна оставаться включенной, чтобы обеспечивать удержание ромашки в поднятом состоянии
        if (direction == 1){
            gpio_put(STEPPER_DIR_PIN_2, 1); 
        }
        else gpio_put(STEPPER_DIR_PIN_2, 0);

        pwm_set_gpio_level(STEPPER_PWM_PIN_2, 500);
        sleep_ms((angle / (1.8 * STEPPER_SPEED)) * 1000);
        pwm_set_gpio_level(STEPPER_PWM_PIN_2, 0);
    }
}

int stepper_overheat(){
    if ( (!gpio_get(STEPPER_OVERHEAT_PIN_1)) || (!gpio_get(STEPPER_OVERHEAT_PIN_2))){
        gpio_put(STEPPER_ENABLE_PIN_1, 1);  // Отключаем работу драйверов при перегреве (он отключается автоматически, но на всякий случай)
        return 1;
    }
    else if ( (!gpio_get(STEPPER_OVERHEAT_PIN_3)) || (!gpio_get(STEPPER_OVERHEAT_PIN_4))) {
        gpio_put(STEPPER_ENABLE_PIN_2, 1);  // Отключаем работу драйверов при перегреве (он отключается автоматически, но на всякий случай)
        return 2;
    }
    else{
        gpio_put(STEPPER_ENABLE_PIN_1, 0);
        gpio_put(STEPPER_ENABLE_PIN_2, 0);
        return 0;
    }
}


//Функция инициализации ШИМ-выводов для управления скоростью и пинов для управления направлением вращения
void stepper_pwm_init(){
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

void stepper_direction_init(){
    // Инициализируем пины для управления направлением вращения
    gpio_init(STEPPER_DIR_PIN_1);
    gpio_init(STEPPER_DIR_PIN_2);

    gpio_set_dir(STEPPER_DIR_PIN_1, GPIO_OUT);
    gpio_set_dir(STEPPER_DIR_PIN_2, GPIO_OUT);

    // Выделяем пины для управления включением-выключением драйверов моторов ромашки
    gpio_init(STEPPER_ENABLE_PIN_1);
    gpio_set_dir(STEPPER_ENABLE_PIN_1, GPIO_OUT);
}

void stepper_overheat_init() {
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

void stepper_init(){
    stepper_direction_init();
    stepper_pwm_init();
    stepper_overheat_init();

    // Изначально кассетница отключена
    gpio_put(STEPPER_ENABLE_PIN_1, 1);
}

