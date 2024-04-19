#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <cstdlib>

// Определяем пины, к которым будем подключать шим-входы контроллера мотора
#define PWM_PIN_1 4 // Подключаем к GP2 - 4 ножка микроконтроллера
#define PWM_PIN_2 3 // Подключаем к GP3 - 5 ножка микроконтроллера
#define PWM_PIN_3 18 // Подключаем к GP4 - 6 ножка микроконтроллера
#define PWM_PIN_4 0 // Подключаем к GP5 - 7 ножка микроконтроллера

#define DIR_PIN_1 19
#define DIR_PIN_2 1
#define DIR_PIN_3 16
#define DIR_PIN_4 22
// Определяем предделитель тактовой частоты микроконтроллера (125Мгц)
#define PWM_DIV 65
// Определяем число тиков, через которое таймер будет сброшен. От этого числа
// будет зависеть точность установки скважности импульсов(чем больше, тем выше
// точность)
#define PWM_WRAP 100
// Заметим, что при таких настройках частота шима 125 000 000/6250 = 20 Кгц, что
// необходимо для работы драйвера мотора

void motor_init_all();

void motor1_controller(int speed);
void motor2_controller(int speed);
void motor3_controller(int speed);
void motor4_controller(int speed);
