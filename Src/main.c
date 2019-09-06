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

int cntr;
uint8_t data_ctrl1, address_ctrl1, address_status, data_status, receivedata;
unsigned int x_value=0, y_value=0, z_value=0;
uint8_t whoami=0, outtemp=0, STATUS_reg=0, FIFO_CTRL_reg=0, FIFO_SRC_reg=0, X_low=0, X_high=0, Y_low=0, Y_high=0, Z_low=0, Z_high=0, ctrl_reg1=0, ctrl_reg2=0, ctrl_reg3=0, ctrl_reg4=0, ctrl_reg5=0;
uint8_t INT1_cfg, INT1_src, INT1_TSH_xh, INT1_TSH_xl, INT1_TSH_yh, INT1_TSH_yl, INT1_TSH_zh, INT1_TSH_zl ,INT1_duration, reference;
uint8_t ctrl1 = 0x00, ctrl4 = 0x00;
unsigned char I2C_transmit_buffer[32];
unsigned char I2C_receive_buffer[32];
unsigned char I2C_MAG_buffer[32];
unsigned char I2C_ACC_status;
unsigned char I2C_MAG_status;

SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

#define OKUMA_KOMUTU 	0x80
#define WHO_AM_I		0x0F
#define OUT_X_L			0x28
#define OUT_X_H			0x29
#define OUT_Y_L			0x2A
#define OUT_Y_H			0x2B
#define OUT_Z_L			0x2C
#define OUT_Z_H			0x2D
#define CTRL_REG1		0x20
#define CTRL_REG2		0x21
#define CTRL_REG3		0x22
#define CTRL_REG4		0x23
#define CTRL_REG5		0x24
#define REFERENCE		0x25
#define OUT_TEMP		0x26
#define STATUS_REG		0x27
#define FIFO_CTRL_REG   0x2E
#define FIFO_SRC_REG	0x2F
#define INT1_CFG		0x30
#define INT1_SRC		0x31
#define INT1_TSH_XH		0x32
#define INT1_TSH_XL		0x33
#define INT1_TSH_YH		0x34
#define INT1_TSH_YL		0x35
#define INT1_TSH_ZH		0x36
#define INT1_TSH_ZL		0x37
#define INT1_DURATION	0x38

uint8_t Test[] = "1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 \n"; //Data to send

void my_delay(unsigned int val){
	for(unsigned int i = 0; i < val; i++){

	}
}
uint8_t L3GD20_Read ( uint8_t addr ){
	 uint8_t data;

	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
	 while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	 uint8_t wrt = addr | OKUMA_KOMUTU;

	 HAL_SPI_Transmit(&hspi1, &wrt, 1, 50);
	 while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}

	 HAL_SPI_Receive(&hspi1, &data, 1, 50);
	 while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}

	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	 return data;
}
void SystemClock_Config(void){

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
	PeriphClkInit.I2c1ClockSelection = RCC_CFGR3_I2C1SW_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
static void MX_USART1_UART_Init(void){

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK){
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_UART_Receive_IT(&huart1,&rx_byte,1);// Sending in normal mode

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	rx_buffer[rx_counter++] = rx_byte;
	rx_idle_counter = 0;
	HAL_UART_Receive_IT(&huart1,&rx_byte,1);// Sending in normal mode
}
static void MX_SPI1_Init(void){

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK){
    _Error_Handler(__FILE__, __LINE__);
  }
}
static void MX_TIM2_Init(void){

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 31;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 149;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}
static void MX_GPIO_Init(void){

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pins : PE3 PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_8, GPIO_PIN_RESET);

}
void read_gyro_x(void){
	X_low = L3GD20_Read ( OUT_X_L );
	X_high = L3GD20_Read ( OUT_X_H );
	x_value=(X_high * 256) + (X_low);
}
void read_gyro_y(void){
	Y_low = L3GD20_Read ( OUT_Y_L );
	Y_high = L3GD20_Read ( OUT_Y_H );
	y_value=(Y_high * 256) + (Y_low);
}
void read_gyro_z(void){
	Z_low = L3GD20_Read ( OUT_Z_L );
	Z_high = L3GD20_Read ( OUT_Z_H );
	z_value=(Z_high * 256) + (Z_low);
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

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_RESET);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	address_ctrl1 = CTRL_REG1;
	data_ctrl1 = 0x0F;
	HAL_SPI_Transmit(&hspi1, &address_ctrl1, 1, 50);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	HAL_SPI_Transmit(&hspi1, &data_ctrl1, 1, 50);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3, GPIO_PIN_SET);

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

	whoami = 0;
	cntr = 0;
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

					STATUS_reg = L3GD20_Read ( STATUS_REG );
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

		/*
		//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8); // bu LED!!

		*/
		/*
		// other registers
		// CTRL_REG1
		ctrl_reg1 = L3GD20_Read ( CTRL_REG1 );
		// CTRL_REG2
		ctrl_reg2 = L3GD20_Read ( CTRL_REG2 );
		// CTRL_REG3
		ctrl_reg3=L3GD20_Read ( CTRL_REG3 );
		//	CTRL_REG4
		ctrl_reg4=L3GD20_Read ( CTRL_REG4 );
		// CTRL_REG5
		ctrl_reg5=L3GD20_Read ( CTRL_REG5 );
		// REFERENCE
		reference=L3GD20_Read ( REFERENCE );
		// OUT_TEMP
		outtemp = L3GD20_Read ( OUT_TEMP );
		//STATUS_REG
		STATUS_reg = L3GD20_Read ( STATUS_REG );
		// FIFO_CTRL_REG
		FIFO_CTRL_reg = L3GD20_Read ( FIFO_CTRL_REG );
		//FIFO_SRC_REG
		FIFO_SRC_reg = L3GD20_Read ( FIFO_SRC_REG );
		//INT1_CFG
		INT1_cfg = L3GD20_Read ( INT1_CFG );
		//INT1_SRC
		INT1_src = L3GD20_Read ( INT1_SRC );
		//INT1_TSH_XH
		INT1_TSH_xh = L3GD20_Read ( INT1_TSH_XH );
		//INT1_TSH_XL
		INT1_TSH_xl = L3GD20_Read ( INT1_TSH_XL );
		//INT1_TSH_YH
		INT1_TSH_yh = L3GD20_Read ( INT1_TSH_YH );
		//INT1_TSH_YL
		INT1_TSH_yl = L3GD20_Read ( INT1_TSH_YL );
		//INT1_TSH_ZH
		INT1_TSH_zh = L3GD20_Read ( INT1_TSH_ZH );
		//INT1_TSH_ZL
		INT1_TSH_zl = L3GD20_Read ( INT1_TSH_ZL );
		//INT1_DURATION
		INT1_duration = L3GD20_Read ( INT1_DURATION );
		 */
	}
}

void _Error_Handler(char * file, int line)
{
  while(1) 
  {
  }
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
