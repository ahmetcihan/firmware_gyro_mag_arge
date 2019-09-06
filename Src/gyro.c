#include "i2c.h"
#include "gyro.h"

/*
SPI_HandleTypeDef hspi1;

void read_gyro_x(void){
	X_low = gyro_read ( OUT_X_L );
	X_high = gyro_read ( OUT_X_H );
}
void read_gyro_y(void){
	Y_low = gyro_read ( OUT_Y_L );
	Y_high = gyro_read ( OUT_Y_H );
}
void read_gyro_z(void){
	Z_low = gyro_read ( OUT_Z_L );
	Z_high = gyro_read ( OUT_Z_H );
}
void write_gyro_register(unsigned char address,unsigned char data){
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	HAL_SPI_Transmit(&hspi1, &address, 1, 50);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	HAL_SPI_Transmit(&hspi1, &data, 1, 50);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);
}
uint8_t gyro_read ( uint8_t addr ){
	 uint8_t wrt = addr | OKUMA_KOMUTU;
	 uint8_t data;

	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

	 while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	 HAL_SPI_Transmit(&hspi1, &wrt, 1, 1);
	 while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	 HAL_SPI_Receive(&hspi1, &data, 1, 1);
	 while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	 return data;
}
*/
