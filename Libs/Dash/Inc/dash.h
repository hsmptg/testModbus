#ifndef DASH_H_
#define DASH_H_

#include <stdint.h>
#include "stdbool.h"
#include "modbus_rtu.h"

#define UNUSED(x) (void)(x)

void cbRise();
void cb500us(void);
void dashLoop();

#endif /* DASH_H_ */