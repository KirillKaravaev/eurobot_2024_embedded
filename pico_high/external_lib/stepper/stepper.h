// Определяем пины, к которым будем подключать шим-входы контроллера мотора
#define PWM_PIN_VERTICAL 4 // Подключаем к GP2 - 4 ножка микроконтроллера
#define PWM_PIN_HORIZON 11 // Подключаем к GP4 - 6 ножка микроконтроллера

#define DIR_PIN_VERTICAL 5
#define DIR_PIN_HORIZON 12

#define EN_PIN_VERTICAL 6
#define EN_PIN_HORIZON 13

#define FAULT_PIN_VERTICAL_1 3
#define FAULT_PIN_VERTICAL_2 7

#define FAULT_PIN_HORIZON_3 14
#define FAULT_PIN_HORIZON_4 10

// Определяем предделитель тактовой частоты микроконтроллера (125Мгц) - один для
// всех шим-каналов #define PWM_DIV 6250
#define PWM_DIV 250
// Определяем число тиковs, через которое таймер будет сброшен. От этого числа
// будет зависеть точность установки скважности импульсов(чем больше, тем выше
// точность)
#define PWM_WRAP 1000
// Заметим, что при таких настройках частота шима 125 000 000/6250 = 20 Кгц, что
// необходимо для работы драйвера мотора
#define SPEED 125000000 / (PWM_DIV * PWM_WRAP)

void stepper_pwm_init();
void stepper_direction_init();
void stepper_init();
void stepper(int number, int direction, int angle);
