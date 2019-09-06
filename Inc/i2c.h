#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f3xx_hal.h"
#include "main.h"

 extern I2C_HandleTypeDef hi2c1;
extern void _Error_Handler(char *, int);

void MX_I2C1_Init(void);
unsigned char I2C_transmit_buffer[32];
unsigned char I2C_receive_buffer[32];
#ifdef __cplusplus
}
#endif
#endif /*__ i2c_H */
