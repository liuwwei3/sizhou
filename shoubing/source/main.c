// 四轴遥控器端代码
// 作者: 刘伟伟

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "stdio.h"
#include "NRF_func.h"

GPIO_InitTypeDef GPIO_InitStructure; //GPIO 状态恢复默认
USART_InitTypeDef USART_InitStructure;               //串口设置恢复默认参数
ADC_InitTypeDef ADC_InitSturature;
DMA_InitTypeDef DMA_InitStructure;
 
#define N 50
#define M 8  
char rx_data[10];
vu16 AD_Value[N][M]; //用来存放ADC转换结果，也是DMA的目标地址
vu16 After_filter[M]; //用来存放求平均值之后的结果

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Delay(vu32 nCount);
void USART_Configuration(void);
void ADC_Configuration(void);
void filter(void);
void SPI_Configuration(void);
/* Private functions ---------------------------------------------------------*/
u16 GetVolt(u16 advalue)
{         
	return (u16)(advalue * 330 / 4096);   //求的结果扩大了100倍，方便下面求出小数    
}
/*******************************************************************************
* Function Name  : main.
* Description    : main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{

#ifdef DEBUG
  debug();
#endif
  bool bflag = TRUE;
  u16 value[M];
  u8 send[32] = {1,2,3,4,5,6,7,8,
  				 1,2,3,4,5,6,7,8,
				 1,2,3,4,5,6,7,8,
				 1,2,3,4,5,6,7,8};
  u8 i;
  u8 state;
  Set_System();

  USB_Interrupts_Config();

  Set_USBClock();

  USB_Init();

  USART_Configuration(); 

  ADC_Configuration();

  SPI_Configuration();

  NRF24L01_Init();

  if(NRF24L01_Check()==0)
	printf("  nrf check success");

  TX_Mode();
  while (1)
  {
    //Delay(0xEFFFF);
	filter();
    if ((JoyState() != 0) & (bDeviceState == CONFIGURED))
    {
      Joystick_Send(JoyState());
    }
	if(bflag)
  		{GPIO_SetBits(GPIOB,GPIO_Pin_11);bflag = FALSE;}
  	else
  	 	{GPIO_ResetBits(GPIOB,GPIO_Pin_11);bflag = TRUE;}
	for(i=0;i<8;i++)
		send[i]= After_filter[i]/100;
	state = NRF24L01_TxPacket(send);
	if(state == TX_OK)
	{
		printf("send success!");
		printf("\n");
	}
	else
	{
		printf("send failed!");
		printf("\n");
	}
  }
}

fputc(int ch, FILE *f)
{
  /* Write a character to the USART */
  USART_SendData(USART1, (u8) ch);

  /* Loop until the end of transmission */
  while(!(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == SET))
  {
  }

  return ch;
}

void filter() 
{     
	vu32  sum = 0; 
	u8  count;
	u8 index ; 
	for(index=0;index<M;index++) 
	{  
		for (count=0;count<N;count++)     
		{        
			sum = sum + AD_Value[count][index];  
		}       
		After_filter[index]=sum/N; 
		sum=0;       
	}  																    
}

void USART_Configuration(void)                       //串口初始化函数
{
	//UART and LED clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO , ENABLE);//打开串口时钟 	

	//GPIOB led
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 ; //管脚位置定义，标号可
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//输出速度50MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);//B 组GPIO 初始化
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	 //  no use!!!

	//UART pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//管脚9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure); //TX初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//管脚10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//RX初始化

	//UART mode
    USART_InitStructure.USART_BaudRate = 9600;                                   //波特率2400
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //字长8位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                  //1位停止字节
    USART_InitStructure.USART_Parity = USART_Parity_No;                    //无奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//打开Rx接收和Tx发送功能
	USART_Init(USART1, &USART_InitStructure);                                          //初始化
    USART_Cmd(USART1, ENABLE);                                                        //启动串口
}

void ADC_Configuration()
{
	//UART and DMA clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO |RCC_APB2Periph_ADC1 , ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //72M/6=12,ADC最大时间不能超过14M 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能DMA传输
	
	//ADC pins config 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2 | GPIO_Pin_3|
								  GPIO_Pin_4 | GPIO_Pin_5|GPIO_Pin_6 | GPIO_Pin_7;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	//ADC config
	ADC_DeInit(ADC1);
	ADC_InitSturature.ADC_Mode = ADC_Mode_Independent;
	ADC_InitSturature.ADC_ScanConvMode =ENABLE; //模数转换工作在扫描模式
	ADC_InitSturature.ADC_ContinuousConvMode = ENABLE; //模数转换工作在连续转换模式 
	ADC_InitSturature.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //外部触发转换关闭
	ADC_InitSturature.ADC_DataAlign = ADC_DataAlign_Right; //ADC数据右对齐
	ADC_InitSturature.ADC_NbrOfChannel = M; //顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitSturature); //根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_239Cycles5 );
	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE); 
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1)); //获取ADC1复位校准寄存器的状态,设置 状态则等待
	ADC_StartCalibration(ADC1);  //开始指定ADC1的校准状态
	while(ADC_GetCalibrationStatus(ADC1));   //获取指定ADC1的校准程序,设置状态则 等待
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	//DMA config
	DMA_DeInit(DMA1_Channel1);   //将DMA的通道1寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr =  (u32)&ADC1->DR;  //DMA外设ADC基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value;  //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //内存作为数据传输的目的地
	DMA_InitStructure.DMA_BufferSize = N*M;  //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器 不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //数据宽度为16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //数据宽度 为16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //工作在循环缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMA通道 x拥有高优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道
	DMA_Cmd(DMA1_Channel1, ENABLE);  //启动DMA通道
}
/*******************************************************************************
* Function Name  : Delay
* Description    : Inserts a delay time.
* Input          : nCount: specifies the delay time length.
* Output         : None
* Return         : None
*******************************************************************************/
void Delay(vu32 nCount)
{
  for (; nCount != 0;nCount--);
}

void SPI_Configuration()
{
	SPI_InitTypeDef  SPI_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|
						   RCC_APB2Periph_GPIOB|
						   RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13; //PB13-SCK PB14-MISO PB15-MOSI 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;//PC13-CSN PC14-CE 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15; //PC15-IPU
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/*  配置SPI1模块 初始化*/

   	SPI_I2S_DeInit(SPI2);                          
   	SPI_Cmd(SPI2, DISABLE); //必须先禁用,才能改变MODE
   	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //两线全双工
   	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;       //主
   	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;      //8位
   	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;        //CPOL=0 时钟悬空低
   	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;       //CPHA=0 数据捕获第1个
   	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;        //软件NSS
   	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8 ; //256分频
   	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;      //高位在前
   	SPI_InitStructure.SPI_CRCPolynomial = 7;        //CRC7
   	SPI_Init(SPI2, &SPI_InitStructure);
   	SPI_Cmd(SPI2, ENABLE);
   	//SPI_ReadWriteByte(0xff);   //启动传输
}

#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
