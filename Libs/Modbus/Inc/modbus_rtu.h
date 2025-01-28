#ifndef INC_MODBUS_RTU_H_
#define INC_MODBUS_RTU_H_

#include <stdint.h>
#include "stdbool.h"

#define UNUSED(x) (void)(x)

#define N_DISCRETE_INPUTS 1
#define N_INPUT_REGISTERS 4

extern bool discreteInputs[N_DISCRETE_INPUTS];
extern uint16_t inputRegisters[N_INPUT_REGISTERS];

void set_slaveID(void);
void modbus_init(void);
uint16_t MODBUS_CRC16(char *buf, uint8_t len);
void sendReply(uint8_t slave, uint8_t func, uint16_t addr, uint16_t count);

#endif /* INC_MODBUS_RTU_H_ */