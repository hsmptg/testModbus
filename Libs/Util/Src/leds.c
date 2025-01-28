#include "leds.h"
#include "stdint.h"
#include "main.h"
#include "stm32f4xx_hal.h"

struct pattern {
	uint8_t initial;
	uint8_t size;
	uint16_t toggles[10];
};

struct pattern patterns[] = {
	/* LED_OFF   */ {0, 0, {}},
	/* LED_ON    */ {1, 0, {}},
	/* LED_1HZ   */ {1, 2, {500, 500}},
	/* LED_5HZ   */ {1, 2, {100, 100}},
	/* LED_HEART */ {0, 4, {100, 100, 100, 700}}
};

struct led {
	led_pattern pattern;
	uint32_t lastTick;
	uint8_t index;
	GPIO_TypeDef* port;
	uint16_t pin;
};

#define NLEDS 1
struct led leds[NLEDS] = {
	{ 0, 0, 0, LED_USER_GPIO_Port, LED_USER_Pin}
};

void leds_proc(void) {
	struct led *p;

	for(int n=0; n<NLEDS; n++) {
		p = &leds[n];
		if (patterns[p->pattern].size) {
			if (HAL_GetTick() - p->lastTick >= patterns[p->pattern].toggles[p->index]) {
				p->lastTick = HAL_GetTick();
				HAL_GPIO_TogglePin(p->port, p->pin);
				p->index = (p->index + 1) % patterns[p->pattern].size;
			}
		}
	}
}

void set_led(led_id id, led_pattern pattern) {
	leds[id].lastTick = HAL_GetTick();
	leds[id].pattern = pattern;
	leds[id].index = 0;
	HAL_GPIO_WritePin(leds[id].port, leds[id].pin, patterns[pattern].initial);
}

