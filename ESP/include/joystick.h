#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdint.h>

typedef enum {
    JOY_CENTER = 0,
    JOY_UP,
    JOY_DOWN,
    JOY_LEFT,
    JOY_RIGHT,
    JOY_UP_LEFT,
    JOY_UP_RIGHT,
    JOY_DOWN_LEFT,
    JOY_DOWN_RIGHT
} JoyDirection;

void Joystick_Init(void);
void Joystick_StartTask(void);
JoyDirection Joystick_Get(void);
const char* Joystick_Name(JoyDirection d);

#endif