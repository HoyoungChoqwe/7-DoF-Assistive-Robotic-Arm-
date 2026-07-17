/**
 * @file    Servo.h
 * @brief   Unified servo control for ECE167 robot arm.
 *
 * Pin assignments (Nucleo F446RE direct wiring):
 *   Wrist   : PWM0  PA8   TIM1_CH1
 *   Lower   : PWM1  PA9   TIM1_CH2   safe range 65-270, home 124
 *   Gripper : PWM2  PA5   TIM2_CH1   TD8120  100-250 deg
 *   Base    : PWM4  PB6   TIM4_CH1   TD8120  0-180 deg
 *   Arm     : PWM5  PA10  TIM1_CH3   TD8120  30-150 deg, home 90  (primary)
 *             PWM3  PB5   TIM3_CH2              (mirror, opposite direction)
 *   Middle  : PWM7  PB8   TIM4_CH3   TD8120 81-248 deg, home 158
 *   Top     : PWM6  PC9   TIM3_CH4   safe range 0-160, home 74
 */

#ifndef SERVO_H
#define SERVO_H

/**
 * @brief  Initialize PWM hardware and move all servos to 90 degrees.
 *         Must be called after BOARD_Init() and Timers_Init().
 */
void Servo_Init(void);

/** @brief  Move every initialized servo to its startup angle. */
void Servo_ResetAll(void);

/**
 * @brief  Set all arm-positioning servos as one pose.
 *         Gripper is intentionally separate.
 */
void Servo_SetArmPose(int base, int arm, int wrist, int top, int middle, int lower);

/**
 * @brief  Set arm angle (30-150 degrees).
 *         The mirrored servo is driven automatically.
 */
void Set_Arm(int deg);

/**
 * @brief  Set base angle (0-180 degrees).
 */
void Set_Base(int deg);

/**
 * @brief  Set gripper angle (100-250 degrees).
 *         Prints error and does NOT move if deg is outside [100, 250].
 */
void Set_Gripper(int deg);

/** @brief  Set wrist angle (0-180 degrees). */
void Set_Wrist(int deg);

/** @brief  Set top joint angle (0-160 degrees). */
void Set_Top(int deg);

/** @brief  Set middle joint angle (81-248 degrees). */
void Set_Middle(int deg);

/** @brief  Set lower joint angle (65-270 degrees). */
void Set_Lower(int deg);

/** @brief  Return current arm angle in degrees. */
int Get_Arm(void);

/** @brief  Return current base angle in degrees (0-180). */
int Get_Base(void);

/** @brief  Return current gripper angle in degrees. */
int Get_Gripper(void);

/** @brief  Return current wrist angle in degrees. */
int Get_Wrist(void);

/** @brief  Return current top joint angle in degrees. */
int Get_Top(void);

/** @brief  Return current middle joint angle in degrees. */
int Get_Middle(void);

/** @brief  Return current lower joint angle in degrees. */
int Get_Lower(void);

#endif /* SERVO_H */
