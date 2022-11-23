#include "iostm8s208mb.h"//主控芯片的头文件
#include "stdio.h"//需要使用printf()函数故而包含该头文件
/*************************常用数据类型定义*************************/
#define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t
typedef unsigned char    uint8_t;
typedef unsigned short   uint16_t;
typedef unsigned long    uint32_t;
/*************************端口/引脚定义区域************************/
#define SDA_OUT    PE_ODR_ODR2 //I2C总线时钟引脚PE2(输出)
#define SDA_IN     PE_IDR_IDR2 //I2C总线时钟引脚PE2(输入)
#define SCL        PE_ODR_ODR1 //I2C总线时钟引脚PE1

/*************************用户自定义数据区域***********************/
#define HSIClockFreq    16000000   //系统时钟频率，单位为Hz
#define BaudRate        9600       //欲设定波特率
//static u8 Read_AT24Cxx_DAT[5];  //定义读出数据存放数组
//static u8 Write_AT24Cxx_DAT[5]={0x01,0x02,0x03,0x04,0x05};
/*************************ADXL345   数据区域***********************/
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

/***************************函数声明区域***************************/
void delay(u16 Count); //延时函数声明
void delay_read_xyz(u16 Count); //读取电压数据

void UART1_Init(void);//UART1串口初始化函数声明
void UART1_SendByte(u8 data);//UART1发送单字符函数声明
int putchar(int ch);//UART1发送字符重定向函数声明

void I2C_SDA_DDR(u8 ddr);//I2C_SDA串行数据引脚方向性配置函数声明
void I2C_START(void);//I2C总线起始信号配置函数声明
void I2C_STOP(void);//I2C总线终止信号配置函数声明
u8 I2C_Write8Bit(u8 DAT);//I2C总线单字节数据写入函数声明
u8 I2C_Read8BitNACK(void);//单字节数据读出（发送无应答）函数声明
u8 I2C_Read8BitACK(void);//单字节数据读出（发送应答）函数声明

u8 ADXL345_RD_Reg(u8 ADDR);//ADXL345读取一个字节
void ADXL345_WR_Reg(u8 ADDR,u8 DAT);//ADXL345 写一个字节
u8 ADXL345_Init(void);//ADXL345初始化
void ADXL345_RD_XYZ(short  *x,short  *y,short  *z);//读取ADXL345x,y,z中的值
void ADXL345_Read_Average10(u16 *x,u16 *y,u16 *z);//读取10次求平均值
void ADXL345_interrupt_Init(void);//中断初始化

/****************************主函数区域****************************/
int main(void)
{
  //u8 i;//定义变量i用于控制循环次数
  //u16  *x_temp=&x,*y_temp=&y,*z_temp=&z;
  
  //时钟、串口、IIC、ADXL345初始化
  CLK_CKDIVR=0x00;//设置时钟为内部16MHz高速时钟
  delay(200);//延时系统时钟稳定
  UART1_Init();//串口1初始化
  delay(200);//延时等待串口初始化完成
  PE_DDR_DDR1=1;//配置PE1引脚（SCL）为输出模式
  PE_CR1_C11=1;//配置PE1引脚（SCL）为推挽输出模式
  PE_CR2_C21=0;//配置PE1引脚（SCL）低速率输出模式 
  
  
  //开PB0的外部中断
  asm("sim");//MAIN程序的优先级配置为3级（关总中断）
  //EXTI_CR1|=0x08;//配置PB为仅下降沿触发
  EXTI_CR1|=0x04;//配置PB为仅上升沿触发
  PB_DDR_DDR0=0;//配置PB0端口为输入模式
  PB_CR1_C10=1;//配置PB0端口为弱上拉输入模式
  PB_CR2_C20=1;//使能PB0端口外部中断
  asm("rim");//MAIN程序的优先级由3级降低至0级（开总中断）

  ADXL345_interrupt_Init();//中断初始化
  while(1)
    {

    }
  
}


/****************************中断函数区域****************************/

#pragma vector=0x06
__interrupt void EXTI_PORTB_IRQHandler(void)
{
  //printf("进入中断");
  u8 Data = 0x00;
   u16  x=0,y=0,z =0 ;
  PB_CR2_C20=0;//禁止PB0端口外部中断
  Data=ADXL345_RD_Reg(0x30);
  if(Data&0x10)
        //delay(500);
        delay_read_xyz(100);    
  PB_CR2_C20=1;//使能PB0端口外部中断
  
}



/****************************自定义函数区域****************************/
void delay_read_xyz(u16 Count)
{
  u8 i,j;
  short x=0,y=0,z=0;
  while (Count--)//Count形参控制延时次数
  {
    for(i=0;i<20;i++)
      for(j=0;j<10;j++);
        ADXL345_RD_XYZ(&x,&y,&z);
        printf("x:%d,y:%d,z:%d\r\n",x,y,z);
     ADXL345_RD_Reg(0x30);
  }
}


void delay(u16 Count)
{
  u8 i,j;
  while (Count--)//Count形参控制延时次数
  {
    for(i=0;i<20;i++)
      for(j=0;j<10;j++);
  }
}
/****************************************************************/
//UART1串口初始化函数UART1_Init()，无形参和返回值
/****************************************************************/
void UART1_Init(void)
{
  u16 baud_div=0;//定义变量用于计算得到波特率取值
  UART1_CR1=0x00;
  //**************************************************
  //展开UART1_CR1赋值二进制数值为：0000 0000 
  //含义：R8=0；    接收数据位不存在第9位
  //      T8=0；    发送数据位不存在第9位
  //      UARTD=0； 使能UART功能
  //      M=0；     一个起始位，8个数据位，n个停止位
  //                n取决于UART1_CR3中的STOP[1:0]位
  //      WAKE=0;   UART被空闲总线唤醒
  //      PCEN=0： （UART模式）奇偶校验控制被进制
  //      PS=0；    偶校验
  //      PIEN=0；  校验中断被禁止
  //*************************************************
  UART1_CR3=0x00;
  //**************************************************
  //展开UART1_CR3赋值二进制数值为：0000 0000 
  //含义：保留位=0；必须保持清零
  //      LINEN=0；LIN模式被禁止
  //      STOP=0；配置为10--2个停止位
  //      CLKEN、CPOL、CPHA、LBCL均为0
  //************************************************* 
  baud_div=HSIClockFreq/BaudRate;//求出分频因子
  UART1_BRR2=baud_div&0x000F;
  UART1_BRR2|=((baud_div&0xF000)>>8);//先给BRR2赋值
  UART1_BRR1=((baud_div&0x0FF0)>>4);//最后再设置BRR1
  UART1_CR2=0x08; 
  //**************************************************
  //展开UART1_CR2赋值二进制数值为：0000 1000 
  //含义：TIEN=0；  发送中断被禁止
  //      TCIEN=0；发送中断完成被禁止
  //      RIEN=0； 接收中断被禁止
  //      ILIEN=0；IDLE中断被禁止
  //      TEN=1;  发送功能使能
  //      REN=0； 接收功能使能
  //      RWU=0；（UART模式）正常工作模式
  //      PIEN=0；未发送断开字符
  //*************************************************
}
/****************************************************************/
//UART1发送单字符函数UART1_SendByte()，有形参data，无返回值
/****************************************************************/
void UART1_SendByte(u8 data)
{
  UART1_SR&=0xBF;//清零发送完成标志位TC
  UART1_DR=data;//发送数据
  while(!(UART1_SR&0x40));//等待发送完成
  UART1_SR&=0xBF;//清零发送完成标志位TC
}
/****************************************************************/
//UART1发送字符重定向函数putchar()，有形参ch,有返回值
/****************************************************************/
int putchar(int ch) 
{ 
  UART1_SendByte((u8)ch);//将Printf内容发往串口
  return (ch); 
}
/****************************************************************/
//I2C_SDA串行数据引脚方向性配置函数I2C_SDA_DDR()，有形参ddr,无返回值
/****************************************************************/
void I2C_SDA_DDR(u8 ddr)
{
  if(ddr==1)//配置为输出方式
  {
    PE_DDR_DDR2=1;//配置PE2引脚（SDA）为输出引脚
    PE_CR1_C12=1;//配置PE2引脚（SDA）为推挽输出模式
    PE_CR2_C22=0;//配置PE2引脚（SDA）低速率输出模式
  }
  else//配置为输入方式
  {
    PE_DDR_DDR2=0;//配置PE2引脚（SDA）为输入引脚
    PE_CR1_C12=1;//配置PE2引脚（SDA）为弱上拉输入模式
    PE_CR2_C22=0;//禁止PE2引脚（SDA）外部中断功能
  }
}
/****************************************************************/
//I2C总线起始信号配置函数I2C_START()，无形参，无返回值
/****************************************************************/
void I2C_START(void)
{
  I2C_SDA_DDR(1);//配置SDA引脚为推挽输出模式
  SDA_OUT=1;//SDA引脚置为高电平
  SCL=1;//SCL引脚置为高电平
  delay(1);//延时等待
  SDA_OUT=0;//将SDA置低产生下降沿（产生起始信号）
  delay(1);//延时等待
  SCL=0;//将SCL置低产生下降沿（允许SDA数据传送）
  delay(1);//延时等待
  //printf("start成功");
}
/****************************************************************/
//I2C总线终止信号配置函数I2C_STOP()，无形参，无返回值
/****************************************************************/
void I2C_STOP(void)
{
  I2C_SDA_DDR(1);//配置SDA引脚为推挽输出模式
  SDA_OUT=0;//SDA引脚置为低电平
  SCL=0;//SCL引脚置为低电平
  delay(1);//延时等待
  SCL=1;//将SCL引脚置高产生上升沿
  delay(1);//延时等待
  SDA_OUT=1;//将SDA引脚置高产生上升沿（产生终止信号）
  delay(1);//延时等待
}
/****************************************************************/
//I2C总线单字节数据写入函数I2C_Write8Bit(u8 DAT)，有形参DAT
//有返回值I2C_Write_ACK(应答信号变量值),若返回值为“0”则有从机应答
//若返回值为“1”则从机无应答
/****************************************************************/
u8 I2C_Write8Bit(u8 DAT)
{
  u8 num,I2C_Write_ACK=0;//定义循环控制变量num
  //定义应答信号变量I2C_Write_ACK
  I2C_SDA_DDR(1);//配置SDA引脚为推挽输出模式
  delay(1);//延时等待
  for(num=0x80;num!=0;num>>=1)//执行8次循环
  {
    if((DAT&num)==0)//按位“与”判断DAT每一位值
      SDA_OUT=0;//判断数值为“0”送出低电平
    else
      SDA_OUT=1;//判断数值为“1”送出高电平
    delay(1);//延时等待
    SCL=1;//拉高SCL引脚以保持SDA引脚数据稳定
    delay(1);//延时等待
    SCL=0;//拉低SCL引脚以允许SDA引脚数据变动
    delay(1);//延时等待
  }
  SDA_OUT=1;//置高SDA引脚电平（释放数据线）
  delay(1);//延时等待
  SCL=1;//拉高SCL产生应答位时钟
  delay(1);//延时等待
  I2C_SDA_DDR(0);//配置SDA引脚为弱上拉输入模式
  delay(1);//延时等待
  I2C_Write_ACK=SDA_IN;//取回SDA线上电平赋值给应答信号变量
  delay(1);//延时等待
  SCL=0;//将SCL引脚置低
  return I2C_Write_ACK;//将应答信号变量值进行返回
}
/****************************************************************/
//单字节数据读出函数（发送无应答）I2C_Read8BitNACK()
//无形参，有返回值（读出的单字节数据）
/****************************************************************/
u8 I2C_Read8BitNACK(void)
{
  u8 x,I2CDATA;//定义循环控制变量x，读出数据暂存变量I2CDATA
  I2C_SDA_DDR(1);//配置SDA引脚为推挽输出模式
  SDA_OUT=1;//首先确保主机释放SDA
  delay(1);//延时等待
  I2C_SDA_DDR(0);//配置SDA引脚为弱上拉输入模式
  delay(1);//延时等待
  for(x=0x80;x!=0;x>>=1)//从高位到低位依次进行
  {
    delay(1);//延时等待
    SCL=1;//将SCL引脚置为高电平
    if(SDA_IN==0)//读取SDA引脚的电平状态并进行判定
      I2CDATA&=~x;//判定为“0”则将I2CDATA中对应位清零
    else
      I2CDATA|=x;//判定为“1”则将I2CDATA中对应位置“1”
    delay(1);//延时等待
    SCL=0;//置低SCL引脚以允许从机发送下一位
  }
  I2C_SDA_DDR(1);//配置SDA引脚为推挽输出模式
  delay(1);//延时等待
  SDA_OUT=1;//8位数据发送后拉高SDA引脚发送“无应答信号”
  delay(1);//延时等待
  SCL=1;//将SCL引脚置为高电平
  delay(1);//延时等待
  SCL=0;//将SCL引脚置为低电平完成“无应答位”并保持总线
  return I2CDATA;//将读出的单字节数据进行返回
}
/****************************************************************/
//单字节数据读出函数（发送应答）I2C_Read8BitACK()
//无形参，有返回值（读出的单字节数据）
/****************************************************************/
u8 I2C_Read8BitACK(void)
{
  u8 x,I2CDATA;//定义循环控制变量x，读出数据暂存变量I2CDATA
  I2C_SDA_DDR(1);//配置SDA引脚为推挽输出模式
  delay(1);//延时等待
  SDA_OUT=1;//首先确保主机释放SDA
  delay(1);//延时等待
  I2C_SDA_DDR(0);//配置SDA引脚为弱上拉输入模式
  delay(1);//延时等待
  for(x=0x80;x!=0;x>>=1)//从高位到低位依次进行
  {
    delay(1);//延时等待
    SCL=1;//将SCL引脚置为高电平
    if(SDA_IN==0)//读取SDA引脚的电平状态并进行判定
      I2CDATA&=~x;//判定为“0”则将I2CDATA中对应位清零
    else
      I2CDATA|=x;//判定为“1”则将I2CDATA中对应位置“1”
    delay(1);//延时等待
    SCL=0;//置低SCL引脚以允许从机发送下一位
  }
  I2C_SDA_DDR(1);//配置SDA引脚为推挽输出模式
  delay(1);//延时等待
  SDA_OUT=0;//8位数据发送后置低SDA引脚发送“应答信号”
  delay(1);//延时等待
  SCL=1;//将SCL引脚置为高电平
  delay(1);//延时等待
  SCL=0;//将SCL引脚置为低电平完成“应答位”并保持总线
  return I2CDATA;//将读出的单字节数据进行返回
}
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

void ADXL345_Read_Average10(u16 *x,u16 *y,u16 *z)
{
	u8 i;
	u16 tx,ty,tz;
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
        printf("x is %d,y is %d,z is %d \r\n",*x,*y,*z);
//        printf("Acc of X-axis: %.1f m/s2\r\n", *x*1.0/256*9.8);
//	printf("Acc of Y-axis: %.1f m/s2\r\n", *y*1.0/256*9.8);
//	printf("Acc of Z-axis: %.1f m/s2\r\n", *z*1.0/256*9.8);
        
}