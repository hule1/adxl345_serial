#ifndef _uart1_H
#define _uart1_H

#define HSIClockFreq    16000000   //系统时钟频率，单位为Hz
#define BaudRate        9600       //欲设定波特率

void UART1_Init(void);
void UART1_SendByte(u8 data);
int putchar(int ch);

#endif