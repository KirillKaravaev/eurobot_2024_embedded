#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "stepper.h"
#include <stdlib.h>



/*https://www.webwork.co.uk/2023/06/raspberry-pi-pico-as-switching_21.html */

void stepper(int number, int direction, int angle ){
    if(number == 1){
        if (direction == 1){
            gpio_put(DIR_PIN_1, 1); 
        }
        else gpio_put(DIR_PIN_1, 0);

        pwm_set_gpio_level(PWM_PIN_1, 500);
        sleep_ms((angle/(1.8*SPEED))*1000);
        //Определяем необходимую скважность шим, от которой зависит скорость мотора
        pwm_set_gpio_level(PWM_PIN_1, 0);
    }
    else{
        if (direction == 1){
            gpio_put(DIR_PIN_2, 1); 
        }
        else gpio_put(DIR_PIN_2, 0);

        pwm_set_gpio_level(PWM_PIN_2, 500);
        sleep_ms((angle/(1.8*SPEED))*1000);
        //Определяем необходимую скважность шим, от которой зависит скорость мотора
        pwm_set_gpio_level(PWM_PIN_2, 0);
    }
}


//Функция инициализации ШИМ-выводов для управления скоростью и пинов для управления направлением вращения
void stepper_pwm_init(){
    //https://www.i-programmer.info/programming/hardware/14849-the-pico-in-c-basic-pwm.html?start=1
    //Инициализируем пины для управления направлением вращения
    gpio_init(DIR_PIN_1);
    gpio_init(DIR_PIN_2);
    gpio_set_dir(DIR_PIN_1, GPIO_OUT);
    gpio_set_dir(DIR_PIN_2, GPIO_OUT);
    //Выделяем пины PWM_PIN_i под генерацию ШИМА 
    gpio_set_function(PWM_PIN_1, GPIO_FUNC_PWM);
    gpio_set_function(PWM_PIN_2, GPIO_FUNC_PWM);
    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num1 = pwm_gpio_to_slice_num(PWM_PIN_1);
    uint slice_num2 = pwm_gpio_to_slice_num(PWM_PIN_2);
    //Устанавливаем предделитель от тактовой частоты процессора (125[Мгц])
    pwm_set_clkdiv(slice_num1, PWM_DIV);
    pwm_set_clkdiv(slice_num2, PWM_DIV);
    
    //Устанавливаем частоту ШИМА равной 125[Мгц]/PWM_DIV/PWM_WRAP  
    pwm_set_wrap(slice_num1, PWM_WRAP);
    pwm_set_wrap(slice_num2, PWM_WRAP);
    
    
    //Задаем начальное заполнение шим 50%
    pwm_set_gpio_level(PWM_PIN_1, 0);
    pwm_set_gpio_level(PWM_PIN_2, 0);

    //Включаем генерацию ШИМ
    pwm_set_enabled(slice_num1, true);
    pwm_set_enabled(slice_num2, true);
}

void stepper_direction_init(){
    gpio_init(DIR_PIN_1);
    gpio_init(DIR_PIN_2);

    gpio_set_dir(DIR_PIN_1, GPIO_OUT);
    gpio_set_dir(DIR_PIN_2, GPIO_OUT);
}

void stepper_init(){
    stepper_direction_init();
    stepper_pwm_init();
}

