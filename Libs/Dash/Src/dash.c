#include "main.h"
#include "dash.h"
#include <stdint.h>

uint32_t bPwr = 0;
extern TIM_HandleTypeDef htim4;

void cbRise() {
  bPwr = 1;
  
  TIM4->CNT = 0;
  HAL_TIM_Base_Start_IT(&htim4);  
}

uint32_t pins;

void cb500us(void) {
  HAL_TIM_Base_Stop_IT(&htim4);

  pins = HAL_GPIO_ReadPin(Pin6_GPIO_Port, Pin6_Pin) ? 0 : 1;
  pins <<= 1;
  pins += HAL_GPIO_ReadPin(Pin4_GPIO_Port, Pin4_Pin)  ? 0 : 1;
  pins <<= 1;
  pins += HAL_GPIO_ReadPin(Pin3_GPIO_Port, Pin3_Pin)  ? 0 : 1;
  pins <<= 1;
  pins += HAL_GPIO_ReadPin(Pin5_GPIO_Port, Pin5_Pin)  ? 0 : 1;
}

void dashLoop() {
  static uint32_t last = 0;
  static uint32_t state = 0;

  if (HAL_GetTick() - last > 100) {
    last = HAL_GetTick();

    if (!discreteInputs[0] && bPwr) {
      inputRegisters[2] = 0;
      inputRegisters[3] = 0;
    }
    discreteInputs[0] = bPwr;
    bPwr = 0;

    switch (state) {
      case 0:
        switch (pins) {
          case 1:
          case 0:
          case 3:
            break;
          case 2:
            state = 1;
            break;
          default:
            state = 4;
            inputRegisters[3]++;
            break;
        }
        break;
      case 1:
        switch (pins) {
          case 2:
          case 0:
          case 6:
            break;
          case 4:
            state = 2;
            break;
          default:
            state = 4;
            inputRegisters[3]++;
            break;
        }
        break;
      case 2:
        switch (pins) {
          case 4:
          case 0:
          case 12:
            break;
          case 8:
            state = 3;
            break;
          default:
            state = 4;
            inputRegisters[3]++;
            break;
        }
        break;
      case 3:
        switch (pins) {
          case 8:
          case 0:
          case 9:
            break;
          case 1:
            state = 0;
            inputRegisters[2]++;
            break;
          default:
            state = 4;
            inputRegisters[3]++;
            break;
        }
        break;
      case 4:
        if (pins == 1)
          state = 0;
        break;
    }
  }
}
