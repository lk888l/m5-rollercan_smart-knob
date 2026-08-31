#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "button.h"

static uint32_t fake_tick_ms;
static GPIO_PinState fake_pin_state = GPIO_PIN_SET;

uint32_t HAL_GetTick(void)
{
    return fake_tick_ms;
}

GPIO_PinState HAL_GPIO_ReadPin(void *port, uint16_t pin)
{
    (void)port;
    (void)pin;
    return fake_pin_state;
}

static void require_true(const char *test_name, int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL %s: %s\n", test_name, message);
        exit(EXIT_FAILURE);
    }
}

static void advance_ms(uint32_t duration_ms)
{
    const uint32_t end_ms = fake_tick_ms + duration_ms;
    while (fake_tick_ms < end_ms) {
        fake_tick_ms += 10U;
        button_update();
    }
}

static void press_for(uint32_t duration_ms)
{
    fake_pin_state = GPIO_PIN_RESET;
    advance_ms(duration_ms);
    fake_pin_state = GPIO_PIN_SET;
    advance_ms(30U);
}

static void test_double_click(void)
{
    static const char test_name[] = "double_click";
    button_update();
    press_for(80U);
    advance_ms(100U);
    press_for(80U);

    require_true(test_name, my_button.was_double_click,
                 "two short presses must emit a double-click event");
    require_true(test_name, !my_button.was_click,
                 "double click must suppress the delayed single click");
    require_true(test_name, !my_button.was_longpress,
                 "double click must not emit a long press");
}

static void test_long_press_is_immediate_and_suppresses_click(void)
{
    static const char test_name[] = "long_press";
    button_update();
    fake_pin_state = GPIO_PIN_RESET;
    advance_ms(BUTTON_DEBOUNCE_MS + BUTTON_LONG_PRESS_MS + 30U);

    require_true(test_name, my_button.was_longpress,
                 "long press must fire while the button is still held");
    require_true(test_name, my_button.is_pressed,
                 "button must still report held when long event fires");

    fake_pin_state = GPIO_PIN_SET;
    advance_ms(40U);
    advance_ms(BUTTON_DOUBLE_CLICK_MS + 20U);
    require_true(test_name, !my_button.was_click,
                 "long press release must not become a click");
    require_true(test_name, !my_button.was_double_click,
                 "long press release must not become a double click");
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fputs("usage: button_event_tests <case>\n", stderr);
        return EXIT_FAILURE;
    }
    if (strcmp(argv[1], "double-click") == 0) {
        test_double_click();
    } else if (strcmp(argv[1], "long-press") == 0) {
        test_long_press_is_immediate_and_suppresses_click();
    } else {
        fprintf(stderr, "unknown test case: %s\n", argv[1]);
        return EXIT_FAILURE;
    }
    printf("PASS %s\n", argv[1]);
    return EXIT_SUCCESS;
}
