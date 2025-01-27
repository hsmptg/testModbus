#include "modbus_rtu.h"
#include <stdio.h>
#include "main.h"

uint8_t uartRxData;
int state;
uint8_t dataIndex;
uint8_t dataLen;
char ModbusRx[20];

bool discreteInputs[N_DISCRETE_INPUTS];
uint16_t inputRegisters[N_INPUT_REGISTERS];

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim3;

void getByte(void) {
  HAL_UART_Receive_IT(&huart2, &uartRxData, 1);
}

void modbus_init(void) {
  // receive mode
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
  // wait for 1st byte from USART2
  state = 0;
  getByte();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint16_t rxCRC;
	uint16_t CRCValue;

  UNUSED(huart);

  TIM3->CNT = 0;
  HAL_TIM_Base_Start_IT(&htim3);

  switch (state) {
    case 0:
      dataIndex = 0;
      if (uartRxData == SLAVE) {
        ModbusRx[dataIndex++] = uartRxData;
        printf("Slave %d\r\n", uartRxData);
        state = 1;
        getByte();
      }
      break;
    case 1:
      ModbusRx[dataIndex++] = uartRxData;
      printf("Func %d\r\n", uartRxData);
      switch (uartRxData) {
        case 2:
          dataLen = 8;
          state = 2;
          break; 
        case 4:
          dataLen = 8;
          state = 2;
          break; 
        default:
          state = 0;
      }
      getByte();
      break;
    case 2:
      ModbusRx[dataIndex++] = uartRxData;
      if (dataIndex == dataLen) {
        CRCValue = MODBUS_CRC16(ModbusRx, dataLen - 2);
		    rxCRC = (ModbusRx[dataLen - 1] << 8) | (ModbusRx[dataLen - 2]);
        if(rxCRC == CRCValue) {
          uint16_t addr = (ModbusRx[2]<<8) + ModbusRx[3];
          uint16_t count = (ModbusRx[4]<<8) + ModbusRx[5]; 
          sendReply(ModbusRx[0], ModbusRx[1], addr, count);
        }
        else {
          printf("Msg KO\r\n");
        }
      }
      else
        getByte();
      break;
    default:
      ;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
    HAL_TIM_Base_Stop_IT(htim);
    state = 0;
    getByte();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  UNUSED(huart);

  HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
}

uint16_t MODBUS_CRC16(char *buf, uint8_t len)
{
	static const uint16_t table[2] = { 0x0000, 0xA001 };
	uint16_t crc = 0xFFFF;
	unsigned int i = 0;
	char bit = 0;
	unsigned int xor = 0;

	for( i = 0; i < len; i++ )
	{
		crc ^= buf[i];

		for( bit = 0; bit < 8; bit++ )
		{
			xor = crc & 0x01;
			crc >>= 1;
			crc ^= table[xor];
		}
	}

	return crc;
}

void sendReply(uint8_t slave, uint8_t func, uint16_t addr, uint16_t count)
{
  char ModbusTx[20];
	uint16_t CRCValue;

  // printf("%d %d %d %d\r\n", slave, func, addr, count);
  ModbusTx[0] = slave;
  ModbusTx[1] = func;

  uint16_t index = 3;
  switch (func) {
    case 2:
      ModbusTx[2] = (count >> 3) + 1;
      for (int n=0; n<ModbusTx[2]; n++) {
        uint8_t v = 0;
        uint8_t mask = 1;
        for (int b=0; b<8; b++) {
          v += discreteInputs[addr++] ? mask : 0;
          mask = mask << 1;
          if (--count == 0) break;
        }
        ModbusTx[index++] = v;
      }
      break;
    case 4:
      ModbusTx[2] = count << 1;
      for (int n=0; n<count; n++) {
        ModbusTx[index++] = inputRegisters[addr] >> 8;
        ModbusTx[index++] = inputRegisters[addr] & 0x00FF;
        addr++;
      }
      break;
  }

  CRCValue = MODBUS_CRC16(ModbusTx, index);
	ModbusTx[index++] = (CRCValue & 0x00FF);
	ModbusTx[index++] = (CRCValue >> 8);

  HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
  HAL_UART_Transmit_IT(&huart2, (uint8_t *) ModbusTx, index);   
}