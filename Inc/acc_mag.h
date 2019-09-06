#ifndef ACC_MAG_H_
#define ACC_MAG_H_

unsigned char I2C_ACC_buffer[32];
unsigned char I2C_MAG_buffer[32];
unsigned char I2C_ACC_status;
unsigned char I2C_MAG_status;

void write_acc_mag(unsigned char device_addr, unsigned char addr, unsigned char data);
void read_acc_mag(unsigned char device_addr, unsigned char addr, unsigned char *read_buffer);

#endif /* ACC_MAG_H_ */
