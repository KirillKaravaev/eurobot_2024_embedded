#include "pico/stdlib.h"
#include <cstdio>
#include <map>
#include <string>

#define LOCKER_PIN_UP 26
#define LOCKER_PIN_DOWN 27

extern bool locker_state_up;
extern bool locker_state_down;

void lockers_init();
