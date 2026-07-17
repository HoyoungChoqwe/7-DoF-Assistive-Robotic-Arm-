#include "joystick.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// Your measured mapping
#define PIN_UP    32
#define PIN_DOWN  33
#define PIN_LEFT  26
#define PIN_RIGHT 25

#define DEBOUNCE_MS 30
#define LOOP_MS     20

static JoyDirection g_dir = JOY_CENTER;

static void config_input(gpio_num_t pin)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
}

static inline int pressed(gpio_num_t pin)
{
    return gpio_get_level(pin) == 0; // active-low
}

static uint8_t read_raw_mask(void)
{
    // bit0=U bit1=D bit2=L bit3=R
    uint8_t m = 0;
    if (pressed(PIN_UP))    m |= (1u << 0);
    if (pressed(PIN_DOWN))  m |= (1u << 1);
    if (pressed(PIN_LEFT))  m |= (1u << 2);
    if (pressed(PIN_RIGHT)) m |= (1u << 3);
    return m;
}

static JoyDirection mask_to_dir(uint8_t m)
{
    const int u = (m >> 0) & 1;
    const int d = (m >> 1) & 1;
    const int l = (m >> 2) & 1;
    const int r = (m >> 3) & 1;

    const int vert  = u - d;   // +1 up, -1 down, 0 none/both
    const int horiz = l - r;   // +1 right, -1 left, 0 none/both (joystick mounted flipped)

    if (vert == 0 && horiz == 0) return JOY_CENTER;
    if (vert == 1 && horiz == 0) return JOY_UP;
    if (vert == -1 && horiz == 0) return JOY_DOWN;
    if (vert == 0 && horiz == -1) return JOY_LEFT;
    if (vert == 0 && horiz == 1) return JOY_RIGHT;

    if (vert == 1 && horiz == -1) return JOY_UP_LEFT;
    if (vert == 1 && horiz == 1)  return JOY_UP_RIGHT;
    if (vert == -1 && horiz == -1) return JOY_DOWN_LEFT;
    return JOY_DOWN_RIGHT;
}

const char* Joystick_Name(JoyDirection d)
{
    switch (d) {
        case JOY_CENTER: return "CENTER";
        case JOY_UP: return "UP";
        case JOY_DOWN: return "DOWN";
        case JOY_LEFT: return "LEFT";
        case JOY_RIGHT: return "RIGHT";
        case JOY_UP_LEFT: return "UP_LEFT";
        case JOY_UP_RIGHT: return "UP_RIGHT";
        case JOY_DOWN_LEFT: return "DOWN_LEFT";
        case JOY_DOWN_RIGHT: return "DOWN_RIGHT";
        default: return "?";
    }
}

static void joystick_task(void *arg)
{
    (void)arg;

    uint8_t stable = read_raw_mask();
    uint8_t candidate = stable;
    int same_ticks = 0;

    const int need_ticks = (DEBOUNCE_MS + LOOP_MS - 1) / LOOP_MS;

    g_dir = mask_to_dir(stable);
    printf("Joystick: %s\n", Joystick_Name(g_dir));

    while (1) {
        uint8_t m = read_raw_mask();

        if (m == candidate) same_ticks++;
        else { candidate = m; same_ticks = 0; }

        if (same_ticks >= need_ticks && candidate != stable) {
            stable = candidate;
            JoyDirection nd = mask_to_dir(stable);
            if (nd != g_dir) {
                g_dir = nd;
                printf("Joystick: %s\n", Joystick_Name(g_dir));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}

void Joystick_Init(void)
{
    config_input(PIN_UP);
    config_input(PIN_DOWN);
    config_input(PIN_LEFT);
    config_input(PIN_RIGHT);
    g_dir = JOY_CENTER;
}

void Joystick_StartTask(void)
{
    // Run on core 1 to avoid starving system tasks on core 0
    xTaskCreatePinnedToCore(joystick_task, "joystick", 4096, NULL, 5, NULL, 1);
}

JoyDirection Joystick_Get(void)
{
    return g_dir;
}
