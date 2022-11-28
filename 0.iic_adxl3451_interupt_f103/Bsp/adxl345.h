#ifndef _adxl345_H
#define _adxl345_H

#include "pbdata.h"
#include "iic.h"

#define DEVICE_ID		0X00 	//器件ID,0XE5
#define DATA_FORMAT	        0X31    //数据格式控制
#define BW_RATE			0X2C    //数据速率和功率模式控制
#define POWER_CTL		0X2D    //省电模式控制
#define INT_ENABLE		0X2E    //中断使能控制

#define FIFO_CTL                0x38    //FIFO控制
#define THRESH_ACT              0x24    //活动阈值
#define THRESH_INACT            0x25    //静止阈值
#define TIME_INACT              0x26    //静止时间
#define ACT_INACT_CTL           0x27    //轴使能控制活动和静止检测
#define INT_MAP                 0x2F    //中断引脚映射控制
#define OFSX			0X1E
#define OFSY			0X1F
#define OFSZ			0X20


u8 ADXL345_RD_Reg(u8 ADDR);//ADXL345读取一个字节
void ADXL345_WR_Reg(u8 ADDR,u8 DAT);//ADXL345 写一个字节
u8 ADXL345_Init(void);//ADXL345初始化
void ADXL345_RD_XYZ(short  *x,short  *y,short  *z);//读取ADXL345x,y,z中的值
void ADXL345_Read_Average10(short  *x,short *y,short  *z);//读取10次求平均值
void ADXL345_interrupt_Init(void);//中断初始化

#endif