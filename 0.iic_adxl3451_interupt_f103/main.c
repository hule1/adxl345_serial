





/* Includes ------------------------------------------------------------------*/
#include "pbdata.h"//主控芯片的头文件
#include "stdio.h"//需要使用printf()函数故而包含该头文件
/* Private defines -----------------------------------------------------------*/
#define HSIClockFreq    16000000   //系统时钟频率，单位为Hz
#define BaudRate        9600       //欲设定波特率

/* Private function prototypes -----------------------------------------------*/
void init(void);
void delay_read_xyz(u16 Count);
/* Private functions ---------------------------------------------------------*/



void main(void)
{
  /* Initialize I/Os in Output Mode */
  //short x=0,y=0,z=0;
    init();
    
  while (1)
  {
     //ADXL345_RD_XYZ(&x,&y,&z);
     //delay(500);
     //printf("%d,%d,%d\r\n",x,y,z);
     //printf("aa");
  }
  
}
/****************************中断函数区域****************************/

#pragma vector=0x06 //端口B外部中断
__interrupt void EXTI_PORTB_IRQHandler(void)
{
  u8 Data = 0x00;
  PB_CR2_C24=0;//禁止PB4端口外部中断
  Data=ADXL345_RD_Reg(0x30);
  if(Data&0x10)
        //delay(500);
        delay_read_xyz(100);    
  PB_CR2_C24=1;//使能PB4端口外部中断
  
}



/****************************自定义函数区域****************************/


void init(void)
{
  //时钟、串口、IIC、ADXL345初始化
  CLK_CKDIVR=0x00;//设置时钟为内部16MHz高速时钟
  delay(200);//延时系统时钟稳定
  UART1_Init();//串口1初始化
  delay(200);//延时等待串口初始化完成
  I2C_SCL_init();//I2C_SDA串行数据引脚初始化
  
  //开PB4的外部中断
  asm("sim");//MAIN程序的优先级配置为3级（关总中断）
  //EXTI_CR1|=0x08;//配置PB为仅下降沿触发
  EXTI_CR1|=0x04;//配置PB为仅上升沿触发
  PB_DDR_DDR4=0;//配置PB4端口为输入模式
  PB_CR1_C14=1;//配置PB4端口为弱上拉输入模式
  PB_CR2_C24=1;//使能PB4端口外部中断
  asm("rim");//MAIN程序的优先级由3级降低至0级（开总中断）
  
  ADXL345_interrupt_Init();//中断初始化

}

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


#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
