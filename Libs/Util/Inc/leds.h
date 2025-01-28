#ifndef LEDS_H_
#define LEDS_H_

typedef enum {
  LED_STATUS = 0,
  LED_ERROR
} led_id;

typedef enum {
  LED_OFF = 0,
  LED_ON,
  LED_1HZ,
  LED_5HZ,
  LED_HEART
} led_pattern;

void leds_proc(void);
void set_led(led_id id, led_pattern pattern);

#endif /* LEDS_H_ */
