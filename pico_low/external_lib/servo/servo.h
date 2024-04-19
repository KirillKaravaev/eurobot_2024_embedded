#include "hardware/pwm.h"
#include "pico/stdlib.h"

#define SERVO_COUNT 6

// Определяем пины, к которым будем подключать шим-входы контроллера мотора
#define SERVO_PWM_PIN_1 10 // Подключаем к GP2 - 4 ножка микроконтроллера
#define SERVO_PWM_PIN_2 9 // Подключаем к GP3 - 5 ножка микроконтроллера
#define SERVO_PWM_PIN_3 8
#define SERVO_PWM_PIN_4 7
#define SERVO_PWM_PIN_5 6
#define SERVO_PWM_PIN_6 5

#define STATE_CLOSED
#define STATE_OPENED
#define STATE_SEMI_CLOSED

// Определяем предделитель тактовой частоты микроконтроллера (125Мгц) - один для
// всех шим-каналов #define PWM_DIV 6250
#define SERVO_PWM_DIV 250
// Определяем число тиковs, через которое таймер будет сброшен. От этого числа
// будет зависеть точность установки скважности импульсов(чем больше, тем выше
// точность)
#define SERVO_PWM_WRAP 10000
//             _____                            _____
//            |     |                          |     | 0.5ms - 0 deg |     | |
//            |                      2.5ms - 180 deg
//____________|     |__________________________|     |______
//             0.5-2.5ms
//           (250-1250)                        |
//            |<------------------------------>|
//                            20ms (10000)

void servo_init_all();
void servo_set_angle(uint8_t number, int16_t angle);
