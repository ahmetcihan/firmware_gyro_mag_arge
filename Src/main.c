/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "i2c.h"
#include "initilize.h"
#include "gyro.h"
#include "acc_mag.h"

SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	rx_buffer[rx_counter++] = rx_byte;
	rx_idle_counter = 0;
	HAL_UART_Receive_IT(&huart1,&rx_byte,1);// Sending in normal mode
}
int main(void){
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_SPI1_Init();
	MX_TIM2_Init();
	HAL_TIM_Base_Start_IT(&htim2);

	write_gyro_register(CTRL_REG1,0x0F);

	I2C_transmit_buffer[0] = 0x20;
	I2C_transmit_buffer[1] = 0x57;
	while(HAL_I2C_Master_Transmit_IT(&hi2c1,0x32,I2C_transmit_buffer, 2)!= HAL_OK){}
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

	I2C_transmit_buffer[0] = 0x23;
	I2C_transmit_buffer[1] = 0x00;
	while(HAL_I2C_Master_Transmit_IT(&hi2c1,0x32,I2C_transmit_buffer, 2)!= HAL_OK){}
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

	I2C_transmit_buffer[0] = 0x02;
	I2C_transmit_buffer[1] = 0x00;
	while(HAL_I2C_Master_Transmit_IT(&hi2c1,0x3C,I2C_transmit_buffer, 2)!= HAL_OK){}
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

	rx_counter = 0;
	rx_idle_counter = 0;
	while (1){
		if(_100_usec_counter > 999){
			_100_usec_counter = 0;
			_100_msec = 1;
		}
		if(_1_msec == 1){
			_1_msec = 0;

			rx_idle_counter++;
			if(rx_idle_counter == 2){
				rx_counter = 0;
				if((rx_buffer[0] == 'C')&&(rx_buffer[1] == 'O')&&(rx_buffer[2] == 'N')&&(rx_buffer[3] == 'V')){
					HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9); // bu LED!!

					STATUS_reg = gyro_read( STATUS_REG );
					if((STATUS_reg & 0x01) == 0x01){
						read_gyro_x();
					}
					if((STATUS_reg & 0x02) == 0x02){
						read_gyro_y();
					}
					if((STATUS_reg & 0x04) == 0x04){
						read_gyro_z();
					}

					while(HAL_I2C_Mem_Read_IT(&hi2c1, 0x33, 0x27, I2C_MEMADD_SIZE_8BIT, &I2C_receive_buffer[6], 1)!= HAL_OK){}
					while( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ){}
					I2C_ACC_status = I2C_receive_buffer[6];

					if((I2C_ACC_status & 0x01) == 0x01){	//x axis
						while(HAL_I2C_Mem_Read_IT(&hi2c1, 0x33, 0x28, I2C_MEMADD_SIZE_8BIT, &I2C_receive_buffer[0], 1)!= HAL_OK){}
						while( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ){}
						while(HAL_I2C_Mem_Read_IT(&hi2c1, 0x33, 0x29, I2C_MEMADD_SIZE_8BIT, &I2C_receive_buffer[1], 1)!= HAL_OK){}
						while( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ){}
					}
					if((I2C_ACC_status & 0x02) == 0x02){	//y axis
						while(HAL_I2C_Mem_Read_IT(&hi2c1, 0x33, 0x2A, I2C_MEMADD_SIZE_8BIT, &I2C_receive_buffer[2], 1)!= HAL_OK){}
						while( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ){}
						while(HAL_I2C_Mem_Read_IT(&hi2c1, 0x33, 0x2B, I2C_MEMADD_SIZE_8BIT, &I2C_receive_buffer[3], 1)!= HAL_OK){}
						while( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ){}
					}
					if((I2C_ACC_status & 0x04) == 0x04){	//z axis
						while(HAL_I2C_Mem_Read_IT(&hi2c1, 0x33, 0x2C, I2C_MEMADD_SIZE_8BIT, &I2C_receive_buffer[4], 1)!= HAL_OK){}
						while( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ){}
						while(HAL_I2C_Mem_Read_IT(&hi2c1, 0x33, 0x2D, I2C_MEMADD_SIZE_8BIT, &I2C_receive_buffer[5], 1)!= HAL_OK){}
						while( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ){}
					}

					while(HAL_I2C_Mem_Read_IT(&hi2c1, 0x3D, 0x09, I2C_MEMADD_SIZE_8BIT, &I2C_MAG_buffer[6], 1)!= HAL_OK){}
					while( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ){}
					I2C_MAG_status = I2C_MAG_buffer[6];

					if((I2C_MAG_status & 0x01) == 0x01){
						while(HAL_I2C_Mem_Read_IT(&hi2c1, 0x3D, 0x03, I2C_MEMADD_SIZE_8BIT, &I2C_MAG_buffer[0], 1)!= HAL_OK){}
						while( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ){}
						while(HAL_I2C_Mem_Read_IT(&hi2c1, 0x3D, 0x04, I2C_MEMADD_SIZE_8BIT, &I2C_MAG_buffer[1], 1)!= HAL_OK){}
						while( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ){}
						while(HAL_I2C_Mem_Read_IT(&hi2c1, 0x3D, 0x05, I2C_MEMADD_SIZE_8BIT, &I2C_MAG_buffer[2], 1)!= HAL_OK){}
						while( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ){}
						while(HAL_I2C_Mem_Read_IT(&hi2c1, 0x3D, 0x06, I2C_MEMADD_SIZE_8BIT, &I2C_MAG_buffer[3], 1)!= HAL_OK){}
						while( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ){}
						while(HAL_I2C_Mem_Read_IT(&hi2c1, 0x3D, 0x07, I2C_MEMADD_SIZE_8BIT, &I2C_MAG_buffer[4], 1)!= HAL_OK){}
						while( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ){}
						while(HAL_I2C_Mem_Read_IT(&hi2c1, 0x3D, 0x08, I2C_MEMADD_SIZE_8BIT, &I2C_MAG_buffer[5], 1)!= HAL_OK){}
						while( HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY ){}
					}

					tx_buffer[0] = 'G';
					tx_buffer[1] = 'Y';
					tx_buffer[2] = 'R';
					tx_buffer[3] = X_low;
					tx_buffer[4] = X_high;
					tx_buffer[5] = Y_low;
					tx_buffer[6] = Y_high;
					tx_buffer[7] = Z_low;
					tx_buffer[8] = Z_high;
					tx_buffer[9] = STATUS_reg;
					tx_buffer[10] = I2C_receive_buffer[0];
					tx_buffer[11] = I2C_receive_buffer[1];
					tx_buffer[12] = I2C_receive_buffer[2];
					tx_buffer[13] = I2C_receive_buffer[3];
					tx_buffer[14] = I2C_receive_buffer[4];
					tx_buffer[15] = I2C_receive_buffer[5];
					tx_buffer[16] = I2C_receive_buffer[6];
					tx_buffer[17] = I2C_MAG_buffer[0];
					tx_buffer[18] = I2C_MAG_buffer[1];
					tx_buffer[19] = I2C_MAG_buffer[2];
					tx_buffer[20] = I2C_MAG_buffer[3];
					tx_buffer[21] = I2C_MAG_buffer[4];
					tx_buffer[22] = I2C_MAG_buffer[5];

					HAL_UART_Transmit_IT(&huart1,tx_buffer,23);
				}
				for(unsigned char i = 0; i < 32; i++){
					rx_buffer[i] = 0;
				}
			}
		}
		if(_100_msec == 1){
			_100_msec = 0;
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8); // bu LED!!

		}
	}
}

void _Error_Handler(char * file, int line)
{
  while(1) 
  {
  }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
