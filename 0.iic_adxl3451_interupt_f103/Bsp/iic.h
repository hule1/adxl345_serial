#ifndef _iic_H  //若已经定义这个头文件就不重新定义
#define _iic_H

/*---------------------------头文件引用--------------------------------------*/

#define SDA_OUT    PC_ODR_ODR5 //I2C总线时钟引脚PC5(输出)
#define SDA_IN     PC_IDR_IDR5 //I2C总线时钟引脚PC5(输入)
#define SCL        PC_ODR_ODR6 //I2C总线时钟引脚PC6
void I2C_SCL_init();//I2C_SDA串行数据引脚初始化
void I2C_SDA_DDR(u8 ddr);//I2C_SDA串行数据引脚方向性配置函数声明
void I2C_START(void);//I2C总线起始信号配置函数声明
void I2C_STOP(void);//I2C总线终止信号配置函数声明
u8 I2C_Write8Bit(u8 DAT);//I2C总线单字节数据写入函数声明
u8 I2C_Read8BitNACK(void);//单字节数据读出（发送无应答）函数声明
u8 I2C_Read8BitACK(void);//单字节数据读出（发送应答）函数声明

#endif