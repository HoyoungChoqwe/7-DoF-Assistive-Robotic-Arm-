/*
 * File:   Leds.c
 * Author: Adam Korycki
 *
 * Created on November 15, 2023
 */

#include <stdio.h>
#include <stdint.h>
#include <BOARD.h>
#include <Leds.h>
#include "stm32f4xx_hal.h"

/**
 * @function LEDs_Init(void)
 * @param None
 * @return None
 * @brief Initializes GPIO outputs for the eight leds on the IO shield
 * @author Adam Korycki, 2023.11.15 */
int8_t LEDs_Init(void) {
    //enable GPIO clocks for ports C and B
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    //init GOIO output pins for leds
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    LEDs_Set(0x0); // reset leds

    return SUCCESS;
}

/**
 * @function LEDs_Set(uint8_t leds)
 * @param leds - byte where each bit represents the state of the leds (lsb -> LED1)
 * @return None
 * @brief sets leds according to byte parameter. Must call LEDs_Init() before.
 * @author Adam Korycki, 2023.11.15 */
void LEDs_Set(uint8_t newPattern) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, (newPattern) & 0x1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, (newPattern >> 1) & 0x1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, (newPattern >> 2) & 0x1);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, (newPattern >> 3) & 0x1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (newPattern >> 4) & 0x1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (newPattern >> 5) & 0x1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, (newPattern >> 6) & 0x1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, (newPattern >> 7) & 0x1);
}

//#define LEDS_TEST
#ifdef LEDS_TEST // LED TEST HARNESS
// SUCCESS - leds on IO shield are shown to be counting up in binary with D1 as the lsb. Counting should roll over

#include <stdio.h>
#include <stdlib.h>
#include <BOARD.h>
#include <Leds.h>


int main(void) {
    BOARD_Init();
    LEDs_Init();

    for (uint8_t i = 0; i < 256; i++) {
        LEDs_Set(i);
        HAL_Delay(200);
    }
    while (TRUE);
}
#endif
