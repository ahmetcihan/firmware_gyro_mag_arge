#ifndef __MAIN_H
#define __MAIN_H

int _100_usec_counter;
char _1_msec;
char _100_msec;
unsigned char rx_byte;
unsigned char rx_counter;
unsigned char rx_buffer[32];
unsigned char tx_buffer[32];
unsigned int rx_idle_counter;

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

#endif /* __MAIN_H */
