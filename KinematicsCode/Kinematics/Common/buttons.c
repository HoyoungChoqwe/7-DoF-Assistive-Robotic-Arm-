/**
 * @file    Buttons.c
 *
 * This library provides an interface for reading the buttons on the UCSC
 * Nucleo I/O Shield development board.
 *
 * @author
 *
 * @date
 */
// Standard libraries.
#include <stdio.h>
#include <stdlib.h>

// Course libraries.
#include "Buttons.h"

#define BUTTONS_NUM_BUTTONS 4

/**
 * This function initializes the proper pins such that the buttons 1-4 may be
 * used.
 */
void Buttons_Init(void)
{
    // Enable pins PC4, PC5, PC12, and PD2 as inputs for buttons 1, 2, 3, and 4,
    // respectively.
    //
    // Each of these pins is also accessible via pin headers.
    //
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* Configure GPIO pins : PC4, PC5, PC12 */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Configure GPIO pin : PD2 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/** Buttons_End()
 */
uint8_t Buttons_End(void)
{
    return 0;
}

/**
 * ButtonsCheckEvents function checks the current button states and returns
 * any events that have occured since its last call.  This function should be
 * called repeatedly in a Timer ISR, though it can be called in main() during
 * testing.
 *
 * In normal use, this function should only be used after ButtonsInit().
 *
 * This function should assume that the buttons start in an off state with value
 * 0. Therefore if no buttons are pressed when ButtonsCheckEvents() is first
 * called, BUTTONS_EVENT_NONE should be returned.
 *
 * @return  Each bit of the return value corresponds to one ButtonEvent flag,
 *          as described in Buttons.h.  If no events are detected,
 *          BUTTONS_EVENT_NONE is returned.
 *
 * Note that more than one event can occur simultaneously, though this
 * situation is rare. To handle this, the output should be a bitwise OR of
 * all applicable event flags.  For example, if button 1 was released at the
 * same time that button 2 was pressed, this function should return
 * (BUTTON_EVENT_1UP | BUTTON_EVENT_2DOWN).
 */
uint8_t Buttons_CheckEvents(void)
{
    ButtonEventFlags event = BUTTON_EVENT_NONE;

    static uint8_t samples[BUTTONS_DEBOUNCE_PERIOD] = {0};
    static uint8_t latest_sample = 0;
    
    static uint8_t old_sums[BUTTONS_NUM_BUTTONS] = {0}; // Assume the starting status is all buttons are UP.
    static uint8_t sums[BUTTONS_NUM_BUTTONS] = {0};

    uint8_t oldest_sample = samples[latest_sample];
    samples[latest_sample] = ~(BUTTON_STATES()); // Invert so that 0 is UP, 1 is DOWN.

    // A button event shall occur if there has been N samples in a row that are opposite to the last button event.
    // Ex. BUTTON_EVENT_1DOWN shall occur if the last_event was BUTTON_EVENT_1UP
    //     and the last N samples all show that the button has been pressed.

    // Check for N samples in a row being the same via a running sum.
    // Running sum: Add the newest sample, remove the oldest sample.
    // If the sum is at 0, that is N samples in a row at 0 (UP) for button j.
    // If the sum is at BUTTONS_DEBOUNCE_PERIOD, that is N samples in a row at 1 (DOWN) for button j.
    // Anything in between is still bouncing and does not indicate a firm button press or release.
    for (uint8_t j = 0; j < BUTTONS_NUM_BUTTONS; j++)
    {
        sums[j] = sums[j] + ((samples[latest_sample] >> j) & 0x01) - ((oldest_sample >> j) & 0x01);
        // printf("sums[%d]: %d\r\n", j, sums[j]);
    }

    // For BTN1-4 (represented as 0-3), check for a change in state that warrants an event.
    for (uint8_t button_number = 0; button_number < BUTTONS_NUM_BUTTONS; button_number++)
    {
        if ((old_sums[button_number] == 0) && (sums[button_number] == BUTTONS_DEBOUNCE_PERIOD)) // UP to DOWN
        {
            event |= 0x02 << (button_number << 1); // BUTTON_EVENT_xDOWN
            old_sums[button_number] = sums[button_number];
        }
        else if ((old_sums[button_number] == BUTTONS_DEBOUNCE_PERIOD) && (sums[button_number] == 0)) // DOWN to UP
        {
            event |= 0x01 << (button_number << 1); // BUTTON_EVENT_xUP
            old_sums[button_number] = sums[button_number];
        }
    }

    latest_sample = (latest_sample + 1) % BUTTONS_DEBOUNCE_PERIOD;

    return event;
}
