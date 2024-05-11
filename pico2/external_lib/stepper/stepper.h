/******************************************************************Обозначения*********************************************************************************/

//Определяем пины, к которым будем подключать шим-входы контроллера мотора
#define STEPPER_PWM_PIN_1 4     //Подключаем к GP2 - 4 ножка микроконтроллера
#define STEPPER_PWM_PIN_2 11    // Подключаем к GP4 - 6 ножка микроконтроллера

#define STEPPER_DIR_PIN_1 5
#define STEPPER_DIR_PIN_2 12

//Используется только для отключения кассетницы
#define STEPPER_ENABLE_PIN_1 6
#define STEPPER_ENABLE_PIN_2 13
//#define ENABLE_PIN_2 13


#define STEPPER_OVERHEAT_PIN_1 3
#define STEPPER_OVERHEAT_PIN_2 7
#define STEPPER_OVERHEAT_PIN_3 14
#define STEPPER_OVERHEAT_PIN_4 10

//Определяем предделитель тактовой частоты микроконтроллера (125Мгц) - один для всех шим-каналов

#define STEPPER_PWM_DIV 250
//Определяем число тиков, через которое таймер будет сброшен. В данном случае настройка нужна лишь для деления частоты до значения 500-1000 Гц
#define STEPPER_PWM_WRAP 2000
//Заметим, что при таких настройках частота шима 125 000 000/250000 = 500 Гц, что необходимо для адеватного вращения шагового двигателя
#define STEPPER_SPEED 125000000 / (STEPPER_PWM_DIV * STEPPER_PWM_WRAP)

/***************************************************************Библиотечные функции***************************************************************************/

void stepper_pwm_init();
void stepper_direction_init();
void stepper_init();
void stepper_overheat_init();
int  stepper_overheat();
/*
void motor1_controller(float speed );
void motor2_controller(float speed );
void motor3_controller(float speed );
void motor4_controller(float speed );
*/
void stepper(int number, int direction, int angle );

