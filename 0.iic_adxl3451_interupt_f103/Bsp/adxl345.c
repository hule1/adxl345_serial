#include "pbdata.h"


/****************************************************************/
//ADXL345读出单个字节函数ADXL345_ReadByte，有形参ADDR
//ADDR为欲读出地址，无返回值
/****************************************************************/
u8 ADXL345_RD_Reg(u8 ADDR)
{
  u8 ADXL345_DATA;//定义变量用于存放读出数据
  I2C_START();//产生I2C通信起始信号
  I2C_Write8Bit(0xA6);//写入（器件地址+写）指令
  I2C_Write8Bit(ADDR);//指定欲读取AT24Cxx芯片的地址
  I2C_START();//产生I2C通信起始信号
  I2C_Write8Bit(0xA7);//写入（器件地址+读）指令
  ADXL345_DATA=I2C_Read8BitNACK();//单字节读取（发送无应答）
  I2C_STOP();//产生I2C通信终止信号
  return ADXL345_DATA;//返回实际读取到的数据值
}
/****************************************************************/
//ADXL345写入单个字节函数ADXL345_WriteByte，有形参ADDR、DAT
//ADDR为欲写入地址，DAT为欲写入数据，无返回值
/****************************************************************/
void ADXL345_WR_Reg(u8 ADDR,u8 DAT)
{
  I2C_START();//产生I2C通信起始信号
  I2C_Write8Bit(0xA6);//写入（器件地址+写）指令
  I2C_Write8Bit(ADDR);//指定欲写入AT24Cxx芯片的地址
  I2C_Write8Bit(DAT);//写入实际数据
  I2C_STOP();//产生I2C通信终止信号
}
/****************************************************************/
//ADXL345中断配置初始化
/****************************************************************/
void ADXL345_interrupt_Init(void)
{
     ADXL345_WR_Reg(INT_ENABLE, 0x00);
    delay(200); //关闭中断
     ADXL345_WR_Reg(DATA_FORMAT, 0x0B);
    delay(200); //测量范围:中断上升沿，正负±16 g , 右对齐 ，全分辨率模式13位分辨率
     ADXL345_WR_Reg(BW_RATE, 0x1A);
    delay(200); //正常功率:低功耗操作，100Hz输出  ，频率 1hz
     ADXL345_WR_Reg(POWER_CTL, 0x18);
    delay(200); //测量模式 ，自动睡眠

     ADXL345_WR_Reg(OFSX, 0x00);
    delay(200); // X轴偏移
     ADXL345_WR_Reg(OFSY, 0x00);
    delay(200); // Y轴偏移
     ADXL345_WR_Reg(OFSZ, 0x00);
    delay(200); // Z轴偏移

     ADXL345_WR_Reg(THRESH_ACT, 0x20);
    delay(200); //活动阈值，激活睡眠activity阈值,大于时触发中断，32*62.5mg=2g(不能设置为零)
     ADXL345_WR_Reg(THRESH_INACT, 0x01);
    delay(200); //静止阈值，睡眠开始阈值，最小值，09*62.5mg = 0.9g(不能设置为零)
     ADXL345_WR_Reg(TIME_INACT, 0x0F);
    delay(200); //静止时间，小于THRESH_INACT值宣布静止，进入睡眠，02 - 2秒(不能设置为零)
     ADXL345_WR_Reg(ACT_INACT_CTL, 0xFF);
    delay(200); //直流交流触发配置，XYZ使能触发配置，此处选XYZ交流
     ADXL345_WR_Reg(INT_MAP, 0x00);
    delay(200); //中断引脚映射控制，打开活动中断引脚到 INT1 引脚上
    //ADXL345_WR_Reg(INT_MAP,0x10);
    //delay(200;中断引脚映射控制，打开活动中断引脚到 INT2 引脚上
     ADXL345_WR_Reg(INT_ENABLE, 0x10);
    delay(200); //开活动 中断
}

/****************************************************************/
//ADXL345初始化，连续读写

/****************************************************************/
u8 ADXL345_Init(void)
{				  
	//IIC_Init();	此处在主函数中初始化iic scl 和sdl						//初始化IIC总线	
	if(ADXL345_RD_Reg(DEVICE_ID)==0XE5)	//读取器件ID
	{  
          //普通配置
		ADXL345_WR_Reg(DATA_FORMAT,0X2B);	//低电平中断输出,13位全分辨率,输出数据右对齐,16g量程 
		ADXL345_WR_Reg(BW_RATE,0X0E);		//数据输出速度为100Hz
		
		ADXL345_WR_Reg(INT_ENABLE,0x80);	//不使用中断		 
	 	ADXL345_WR_Reg(OFSX,0x04);              //x轴偏移
		ADXL345_WR_Reg(OFSY,0x00);              //y轴偏移
		ADXL345_WR_Reg(OFSZ,0xF0);	        //z轴偏移
                
                ADXL345_WR_Reg(POWER_CTL,0x08);	   	//链接使能,测量模式
		return 0;
	}			
	return 1;	   								  
} 
/****************************************************************/
//ADXL345_RD_XYZ：读取x，y，z三个轴的数据

/****************************************************************/
void ADXL345_RD_XYZ(short  *x,short  *y,short  *z)
{
	uint8_t buf[6];
	uint8_t i;
        do
        {
            I2C_START();//产生I2C通信起始信号
            if(I2C_Write8Bit(0xA6))//写入（器件地址+写）指令
            {
                I2C_STOP();//产生I2C通信终止信号
            }
            break;
        }while(1);//若能跳出while(1)说明寻址成功
            
        I2C_Write8Bit(0x32);//写入欲读取地址
        I2C_START();//产生I2C通信起始信号
        I2C_Write8Bit(0xA7);//写入（器件地址+读）指令
      /*  i =I2C_Read8BitNACK();
        printf("%d",i);
        I2C_STOP();//产生I2C通信终止信号
        */
	for(i=0;i<6;i++)
	{
		if(i==5)buf[i]=I2C_Read8BitNACK();//读取一个字节,不继续再读,发送NACK  
		else buf[i]=I2C_Read8BitACK();	//读取一个字节,继续读,发送ACK 
 	}
        
        I2C_STOP();//产生I2C通信终止信号					//产生一个停止条件
	*x=(short )(((uint16_t)buf[1]<<8)+buf[0]); 	    
	*y=(short )(((uint16_t)buf[3]<<8)+buf[2]); 	    
	*z=(short )(((uint16_t)buf[5]<<8)+buf[4]); 
	   
}
/****************************************************************/
//ADXL345_RD_XYZ：读取x，y，z三个轴数据10次 后取平均值

/****************************************************************/

void ADXL345_Read_Average10(short  *x,short *y,short  *z)
{
	u8 i;
	short  tx,ty,tz;
	*x=0;
	*y=0;
	*z=0;
	if(10)//读取次数不为0
	{
		for(i=0;i<10;i++)//连续读取times次
		{
			ADXL345_RD_XYZ(&tx,&ty,&tz);
			*x+=tx;
			*y+=ty;
			*z+=tz;
			delay(1000);
		}
		*x/=10;
		*y/=10;
		*z/=10;
	}
        //printf("x is %d,y is %d,z is %d \r\n",*x,*y,*z);
//        printf("Acc of X-axis: %.1f m/s2\r\n", *x*1.0/256*9.8);
//	printf("Acc of Y-axis: %.1f m/s2\r\n", *y*1.0/256*9.8);
//	printf("Acc of Z-axis: %.1f m/s2\r\n", *z*1.0/256*9.8);
        
}