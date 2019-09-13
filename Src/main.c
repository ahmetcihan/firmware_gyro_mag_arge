#include "main.h"
#include "stm32f3xx_hal.h"
#include "i2c.h"
#include "initilize.h"
#include "gyro.h"
#include "acc_mag.h"

typedef unsigned char u8;
typedef signed char s8;
typedef unsigned int u32;
typedef signed int s32;

SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	rx_buffer[rx_counter++] = rx_byte;
	rx_idle_counter = 0;
	HAL_UART_Receive_IT(&huart1,&rx_byte,1);// Sending in normal mode
}
unsigned int crc_chk(unsigned char* data, unsigned char length) {
	int j;
	unsigned int reg_crc=0xFFFF;

	while( length-- ) {
		reg_crc^= *data++;
		for (j=0; j<8; j++ ) {
			reg_crc = (reg_crc & 0x01) ? ((reg_crc >> 1)^0xA001) : (reg_crc>>1);
		}
	}
	return reg_crc;
}
void usart_response(void){
	u32 fcrc;
	u8 crc_low,crc_high;

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
	tx_buffer[10] = I2C_ACC_buffer[0];
	tx_buffer[11] = I2C_ACC_buffer[1];
	tx_buffer[12] = I2C_ACC_buffer[2];
	tx_buffer[13] = I2C_ACC_buffer[3];
	tx_buffer[14] = I2C_ACC_buffer[4];
	tx_buffer[15] = I2C_ACC_buffer[5];
	tx_buffer[16] = I2C_ACC_status;
	tx_buffer[17] = I2C_MAG_buffer[0];
	tx_buffer[18] = I2C_MAG_buffer[1];
	tx_buffer[19] = I2C_MAG_buffer[2];
	tx_buffer[20] = I2C_MAG_buffer[3];
	tx_buffer[21] = I2C_MAG_buffer[4];
	tx_buffer[22] = I2C_MAG_buffer[5];
	tx_buffer[23] = I2C_MAG_status;

//	for(u8 i = 3; i < 24; i++){
//		tx_buffer[i] = i;
//	}

	fcrc = crc_chk((u8*)tx_buffer,24);
	crc_high = (fcrc)%256;
	crc_low = (fcrc)/256;
	tx_buffer[24] = crc_high;
	tx_buffer[25] = crc_low;

	HAL_UART_Transmit_IT(&huart1,tx_buffer,26);
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
	write_gyro_register(CTRL_REG1,0x0F);
	write_acc_mag(0x32,0x20,0x57);
	write_acc_mag(0x32,0x23,0x00);
	write_acc_mag(0x3C,0x00,0x1C);
	write_acc_mag(0x3C,0x01,0xE0);
	write_acc_mag(0x3C,0x02,0x00);

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

					read_acc_mag(0x33,0x27,&I2C_ACC_status);

					if((I2C_ACC_status & 0x01) == 0x01){	//x axis
						read_acc_mag(0x33,0x28,&I2C_ACC_buffer[0]);
						read_acc_mag(0x33,0x29,&I2C_ACC_buffer[1]);
					}
					if((I2C_ACC_status & 0x02) == 0x02){	//y axis
						read_acc_mag(0x33,0x2A,&I2C_ACC_buffer[2]);
						read_acc_mag(0x33,0x2B,&I2C_ACC_buffer[3]);
					}
					if((I2C_ACC_status & 0x04) == 0x04){	//z axis
						read_acc_mag(0x33,0x2C,&I2C_ACC_buffer[4]);
						read_acc_mag(0x33,0x2D,&I2C_ACC_buffer[5]);
					}

					read_acc_mag(0x3D,0x09,&I2C_MAG_status);

					if((I2C_MAG_status & 0x01) == 0x01){
						read_acc_mag(0x3D,0x03,&I2C_MAG_buffer[0]);
						read_acc_mag(0x3D,0x04,&I2C_MAG_buffer[1]);
						read_acc_mag(0x3D,0x05,&I2C_MAG_buffer[2]);
						read_acc_mag(0x3D,0x06,&I2C_MAG_buffer[3]);
						read_acc_mag(0x3D,0x07,&I2C_MAG_buffer[4]);
						read_acc_mag(0x3D,0x08,&I2C_MAG_buffer[5]);
					}
					else{
						HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9); // bu LED!!

					}

					usart_response();
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
