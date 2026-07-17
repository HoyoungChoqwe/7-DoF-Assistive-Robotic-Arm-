/**
 * @file    Servo.c
 * @brief   Unified servo control for ECE167 robot arm.
 *
 * Hardware map (Nucleo F446RE direct wiring):
 *   Gripper : PWM2  PA5   TIM2_CH1   TD8120  20-140 deg (hard limits, physical range 0-270)
 *   Base    : PWM4  PB6   TIM4_CH1   TD8120  0-180 deg
 *   Lower   : PWM1  PA9   TIM1_CH2
 *   Wrist   : PWM0  PA8   TIM1_CH1
 *   Arm mir : PWM3  PB5   TIM3_CH2              (mirror, opposite direction)
 *   Arm pri : PWM5  PA10  TIM1_CH3   TD8120  0-180 deg
 *   Middle  : PWM7  PB8   TIM4_CH3
 *   Top     : PWM6  PC9   TIM3_CH4
 *

 * All channels share the same 50 Hz period (TIM1, TIM2, TIM3, and TIM4 ARR = 19999).
 * Pulse width range: 500 us (0 deg) to 2500 us (max deg).
 */

#include <stdio.h>
#include <BOARD.h>
#include <Pwm.h>
#include "Servo.h"

/* 50 Hz: 1 MHz timer clock, ARR = 1,000,000/50 - 1 = 19999 */
#define SERVO_PERIOD_TICKS  19999U
#define SERVO_MIN_US        500U
#define SERVO_MAX_US        2500U

#define ARM_PHYSICAL_MAX_DEG    180
#define ARM_MIN_DEG             30
#define ARM_HOME_DEG            90
#define ARM_MAX_DEG             150
#define BASE_MAX_DEG            180
#define WRIST_MAX_DEG           180
#define TOP_PHYSICAL_MAX_DEG    180
#define TOP_MIN_DEG             0
#define TOP_HOME_DEG            74
#define TOP_MAX_DEG             160
#define MIDDLE_MIN_DEG          81
#define MIDDLE_HOME_DEG         158
#define MIDDLE_MAX_DEG          248
#define LOWER_MIN_DEG           104
#define LOWER_HOME_DEG          180
#define LOWER_MAX_DEG           265
/* Gripper: TD8120 270-deg physical servo, software-limited to [20, 140] */
#define GRIP_PHYSICAL_MAX_DEG   270     /* full servo range, used for pulse mapping */
#define GRIP_MIN_DEG            100      /* software lower limit */
#define GRIP_MAX_DEG            250     /* software upper limit */

static int arm_deg     = ARM_HOME_DEG;
static int base_deg    = 90;
static int gripper_deg = GRIP_MIN_DEG;
static int wrist_deg   = 90;
static int top_deg     = TOP_HOME_DEG;
static int middle_deg  = MIDDLE_HOME_DEG;
static int lower_deg   = LOWER_HOME_DEG;

/* Convert angle to pulse-width ticks (us == ticks at 1 MHz). */
static uint32_t angle_to_ticks(int deg, int max_deg)
{
    if (deg < 0)       deg = 0;
    if (deg > max_deg) deg = max_deg;
    return SERVO_MIN_US + (uint32_t)((deg * (SERVO_MAX_US - SERVO_MIN_US)) / max_deg);
}

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

void Servo_Init(void)
{
    if (PWM_Init() == ERROR)        { printf("ERROR: PWM_Init\r\n");       while (1); }
    /* Wrist: PWM_0 = PA8/TIM1_CH1 */
    if (PWM_AddPin(PWM_0) == ERROR) { printf("ERROR: PWM_AddPin 0\r\n");   while (1); }
    /* Lower: PWM_1 = PA9/TIM1_CH2 */
    if (PWM_AddPin(PWM_1) == ERROR) { printf("ERROR: PWM_AddPin 1\r\n");   while (1); }
    /* Gripper: PWM_2 = PA5/TIM2_CH1 */
    if (PWM_AddPin(PWM_2) == ERROR) { printf("ERROR: PWM_AddPin 2\r\n");   while (1); }
    /* Arm mirror: PWM_3 = PB5/TIM3_CH2 */
    if (PWM_AddPin(PWM_3) == ERROR) { printf("ERROR: PWM_AddPin 3\r\n");   while (1); }
    /* Base: PWM_4 = PB6/TIM4_CH1 */
    if (PWM_AddPin(PWM_4) == ERROR) { printf("ERROR: PWM_AddPin 4\r\n");   while (1); }
    /* Arm primary: PWM_5 = PA10/TIM1_CH3 */
    if (PWM_AddPin(PWM_5) == ERROR) { printf("ERROR: PWM_AddPin 5\r\n");   while (1); }
    /* Top: PWM_6 = PC9/TIM3_CH4 */
    if (PWM_AddPin(PWM_6) == ERROR) { printf("ERROR: PWM_AddPin 6\r\n");   while (1); }
    /* Middle: PWM_7 = PB8/TIM4_CH3 */
    if (PWM_AddPin(PWM_7) == ERROR) { printf("ERROR: PWM_AddPin 7\r\n");   while (1); }

    /* Override to 50 Hz after PWM_AddPin (which defaults to 1 kHz) */
    TIM1->ARR = SERVO_PERIOD_TICKS;
    TIM2->ARR = SERVO_PERIOD_TICKS;
    TIM3->ARR = SERVO_PERIOD_TICKS;
    TIM4->ARR = SERVO_PERIOD_TICKS;

    /* Move all servos to their startup positions. */
    Servo_ResetAll();

    /* Make sure every channel is actively outputting after the ARR update. */
    PWM_Start(PWM_0);
    PWM_Start(PWM_1);
    PWM_Start(PWM_2);
    PWM_Start(PWM_3);
    PWM_Start(PWM_4);
    PWM_Start(PWM_5);
    PWM_Start(PWM_6);
    PWM_Start(PWM_7);
}

void Servo_ResetAll(void)
{
    Set_Arm(ARM_HOME_DEG);
    Set_Base(90);
    Set_Gripper(GRIP_MIN_DEG);
    Set_Wrist(90);
    Set_Top(TOP_HOME_DEG);
    Set_Middle(MIDDLE_HOME_DEG);
    Set_Lower(LOWER_HOME_DEG);
}

void Servo_SetArmPose(int base, int arm, int wrist, int top, int middle, int lower)
{
    base = clamp_int(base, 0, BASE_MAX_DEG);
    arm = clamp_int(arm, ARM_MIN_DEG, ARM_MAX_DEG);
    wrist = clamp_int(wrist, 0, WRIST_MAX_DEG);
    top = clamp_int(top, TOP_MIN_DEG, TOP_MAX_DEG);
    middle = clamp_int(middle, MIDDLE_MIN_DEG, MIDDLE_MAX_DEG);
    lower = clamp_int(lower, LOWER_MIN_DEG, LOWER_MAX_DEG);

    int arm_mirror_deg = ARM_PHYSICAL_MAX_DEG - arm;

    base_deg = base;
    arm_deg = arm;
    wrist_deg = wrist;
    top_deg = top;
    middle_deg = middle;
    lower_deg = lower;

    TIM4->CCR1 = angle_to_ticks(base, BASE_MAX_DEG);                    /* Base: PWM_4 */
    TIM1->CCR3 = angle_to_ticks(arm, ARM_PHYSICAL_MAX_DEG);             /* Arm primary: PWM_5 */
    TIM3->CCR2 = angle_to_ticks(arm_mirror_deg, ARM_PHYSICAL_MAX_DEG);  /* Arm mirror: PWM_3 */
    TIM1->CCR1 = angle_to_ticks(wrist, WRIST_MAX_DEG);                  /* Wrist: PWM_0 */
    TIM3->CCR4 = angle_to_ticks(top, TOP_PHYSICAL_MAX_DEG);             /* Top: PWM_6 */
    TIM4->CCR3 = angle_to_ticks(middle, MIDDLE_MAX_DEG);                /* Middle: PWM_7 */
    TIM1->CCR2 = angle_to_ticks(lower, LOWER_MAX_DEG);                  /* Lower: PWM_1 */
}

void Set_Arm(int deg)
{
    deg = clamp_int(deg, ARM_MIN_DEG, ARM_MAX_DEG);
    arm_deg = deg;

    int mirror_deg = ARM_PHYSICAL_MAX_DEG - deg;

    TIM1->CCR3 = angle_to_ticks(deg, ARM_PHYSICAL_MAX_DEG);        /* PWM_5: PA10 primary */
    TIM3->CCR2 = angle_to_ticks(mirror_deg, ARM_PHYSICAL_MAX_DEG); /* PWM_3: PB5 mirror */
}

void Set_Base(int deg)
{
    deg = clamp_int(deg, 0, BASE_MAX_DEG);
    base_deg = deg;

    TIM4->CCR1 = angle_to_ticks(deg, BASE_MAX_DEG); /* PWM_4: PB6 */
}

void Set_Gripper(int deg)
{
    if (deg < GRIP_MIN_DEG || deg > GRIP_MAX_DEG) {
        printf("ERROR: Gripper angle %d out of range [%d, %d]\r\n",
               deg, GRIP_MIN_DEG, GRIP_MAX_DEG);
        return;
    }
    gripper_deg = deg;
    /* Map against full 270-deg physical range to preserve correct pulse width */
    TIM2->CCR1 = angle_to_ticks(deg, GRIP_PHYSICAL_MAX_DEG); /* PWM_2: PA5 */
}

void Set_Wrist(int deg)
{
    deg = clamp_int(deg, 0, WRIST_MAX_DEG);
    wrist_deg = deg;

    TIM1->CCR1 = angle_to_ticks(deg, WRIST_MAX_DEG); /* PWM_0: PA8 */
}

void Set_Top(int deg)
{
    deg = clamp_int(deg, TOP_MIN_DEG, TOP_MAX_DEG);
    top_deg = deg;

    TIM3->CCR4 = angle_to_ticks(deg, TOP_PHYSICAL_MAX_DEG); /* PWM_6: PC9 */
}

void Set_Middle(int deg)
{
    deg = clamp_int(deg, MIDDLE_MIN_DEG, MIDDLE_MAX_DEG);
    middle_deg = deg;

    TIM4->CCR3 = angle_to_ticks(deg, MIDDLE_MAX_DEG); /* PWM_7: PB8 */
}

void Set_Lower(int deg)
{
    deg = clamp_int(deg, LOWER_MIN_DEG, LOWER_MAX_DEG);
    lower_deg = deg;

    TIM1->CCR2 = angle_to_ticks(deg, LOWER_MAX_DEG); /* PWM_1: PA9 */
}

int Get_Arm(void)      { return arm_deg;     }
int Get_Base(void)     { return base_deg;    }
int Get_Gripper(void)  { return gripper_deg; }
int Get_Wrist(void)    { return wrist_deg;   }
int Get_Top(void)      { return top_deg;     }
int Get_Middle(void)   { return middle_deg;  }
int Get_Lower(void)    { return lower_deg;   }
