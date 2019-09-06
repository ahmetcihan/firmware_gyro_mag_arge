#include "stm32f3xx_hal.h"
#include "i2c.h"
#include "acc_mag.h"

void write_acc_mag(unsigned char device_addr, unsigned char addr, unsigned char data){
	I2C_transmit_buffer[0] = addr;
	I2C_transmit_buffer[1] = data;
	while(HAL_I2C_Master_Transmit_IT(&hi2c1,device_addr,I2C_transmit_buffer, 2)!= HAL_OK){}
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}
}
void read_acc_mag(unsigned char device_addr, unsigned char addr, unsigned char *read_buffer){
	while(HAL_I2C_Mem_Read_IT(&hi2c1, device_addr, addr, I2C_MEMADD_SIZE_8BIT, read_buffer, 1)!= HAL_OK){}
	while( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ){}
}

