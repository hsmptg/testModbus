#include "sound.h"
#include "main.h"

extern TIM_HandleTypeDef htim11;

uint16_t audio_cnt;

void start_sound(void) {
  TIM11->CNT = 0;
  TIM11->ARR = 1000;
  TIM11->CCR1 = TIM11->ARR >> 1;
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

  audio_cnt = 0;
}

void sound_proc(void) {
  audio_cnt++;
  if (audio_cnt == 1000) {
    TIM11->CNT = 0;
    TIM11->ARR = 500;
    TIM11->CCR1 = TIM11->ARR >> 1;
  }

  if (audio_cnt == 2000) {
    HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
  }
}