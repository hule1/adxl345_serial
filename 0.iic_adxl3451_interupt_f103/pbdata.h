#ifndef _pbdata_H
#define _pbdata_H

#define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t
typedef unsigned char    uint8_t;
typedef unsigned short   uint16_t;
typedef unsigned long    uint32_t;

/*通用头文件*/
#include "iostm8s103F3.h"
#include "uart1.h"
#include "iic.h"
#include "adxl345.h"

/*************************常用数据类型定义*************************/



/*自定义函数声明*/
void delay(u16 Count);
#endif