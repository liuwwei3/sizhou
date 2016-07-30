// ����������˴���
// ���ߣ���ΰΰ


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stdio.h"
#include "NRF_func.h"
#include "inv_mpu.h"
#include ""

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

ErrorStatus HSEStartUpStatus;
GPIO_InitTypeDef GPIO_InitStructure; //GPIO ״̬�ָ�Ĭ��
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;	  //
USART_InitTypeDef USART_InitStructure;               //�������ûָ�Ĭ�ϲ���
TIM_OCInitTypeDef  TIM_OCInitStructure;

void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void sleep(vu32 ncount);
void PWMchannel_config(int t1,int t2,int t3,int t4);
void USART_Configuration(void);
void SPI_Configuration(void);


/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
#ifdef DEBUG
  debug();
#endif
	bool bflag = TRUE;	
	u16 value[8];
	u8 i;
	u8 recieve[32];
	u8 state;

	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
  	TIM_Cmd(TIM2, ENABLE);
	USART_Configuration();
	SPI_Configuration();
	
	if(NRF24L01_Check() == 0)
		printf("NRF check success!!!");	
	RX_Mode();
  	while (1)  
  	{  
		sleep(0xEFFFFF);
		state = NRF24L01_RxPacket(recieve);
		if(state == 0)
		{
			printf("recieve success!");
			printf("\n");
		}
		else
		{
			printf("recieve failed!");
			printf("\n");
		}
		for(i=0;i<8;i++)
		{  
			//value[i]= i;
			//printf("%d.%d\t",value[i]/100,value[i]%100);
			printf("%d\t",recieve[i]);
			sleep(0x1EFF);
		}
		
		printf("\n");
	  	if(bflag)
  		{
			GPIO_SetBits(GPIOC,GPIO_Pin_13);bflag = FALSE;
		//	PWMchannel_config(100,100,100,100);
		}
  		else
  	 	{
			GPIO_ResetBits(GPIOC,GPIO_Pin_13);bflag = TRUE;
			//PWMchannel_config(0,0,0,0);
		}
    }		
}

void GPIO_Configuration(void)
{
	//GPIOC	pin13 normal leds
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; //�ܽ�λ�ö��壬��ſ�
	GPIO_Init(GPIOC, &GPIO_InitStructure);//B ��GPIO ��ʼ��

	//PWM
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2 | GPIO_Pin_3;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOA, &GPIO_InitStructure); 	 

	//UART
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//�ܽ�9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure); //TX��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//�ܽ�10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//RX��ʼ��
}


/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{   
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
 	
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* PLLCLK = 8MHz * 9 = 72 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }

  }

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  
    /* GPIOA and AFIO clock enable */  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO  , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE);//�򿪴���ʱ��
}

void USART_Configuration(void)                       //���ڳ�ʼ������

{
    USART_InitStructure.USART_BaudRate = 9600;                                   //������2400
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //�ֳ�8λ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                  //1λֹͣ�ֽ�
    USART_InitStructure.USART_Parity = USART_Parity_No;                    //����żУ��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//��Rx���պ�Tx���͹���
	USART_Init(USART1, &USART_InitStructure);                                          //��ʼ��
    USART_Cmd(USART1, ENABLE);                                                        //��������
}

void SPI_Configuration()
{
	SPI_InitTypeDef  SPI_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|
						   RCC_APB2Periph_GPIOB|
						   RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7; //PA5-SCK PA6-MISO PA7-MOSI 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//PA4-CSN 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//PB2-CE 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0; //PB0-IPU
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/*  ����SPI1ģ�� ��ʼ��*/

   	SPI_I2S_DeInit(SPI1);                          
   	SPI_Cmd(SPI1, DISABLE); //�����Ƚ���,���ܸı�MODE
   	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����ȫ˫��
   	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;       //��
   	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;      //8λ
   	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;        //CPOL=0 ʱ�����յ�
   	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;       //CPHA=0 ���ݲ����1��
   	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;        //���NSS
   	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8 ; //256��Ƶ
   	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;      //��λ��ǰ
   	SPI_InitStructure.SPI_CRCPolynomial = 7;        //CRC7
   	SPI_Init(SPI1, &SPI_InitStructure);
   	SPI_Cmd(SPI1, ENABLE);
   	//SPI_ReadWriteByte(0xff);   //��������
}

void PWMchannel_config(int t1,int t2,int t3,int t4)  
{  
  TIM_TimeBaseStructure.TIM_Period = 665;  
  TIM_TimeBaseStructure.TIM_Prescaler = 10;  
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;  
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);  
 
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
  TIM_OCInitStructure.TIM_Pulse = t1;  
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);  
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  
 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
  TIM_OCInitStructure.TIM_Pulse = t2;  
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);  
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  

  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
  TIM_OCInitStructure.TIM_Pulse = t3;  
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);  
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  

  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  
  TIM_OCInitStructure.TIM_Pulse = t4;  
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);  
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  
  TIM_ARRPreloadConfig(TIM2, ENABLE);  
}  		  


/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
#ifdef  VECT_TAB_RAM  
  /* Set the Vector Table base location at 0x20000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x08000000 */ 
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif
}

void sleep(vu32 ncount)
{
	for(; ncount != 0; ncount--);
}


//UART PRINTF TEST

/*******************************************************************************
* Function Name  : int fputc(int ch, FILE *f)
* Description    : Retargets the C library printf function to the USART.printf�ض���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int fputc(int ch, FILE *f)
{
  /* Write a character to the USART */
  USART_SendData(USART1, (u8) ch);

  /* Loop until the end of transmission */
  while(!(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == SET))
  {
  }

  return ch;
}

/*******************************************************************************
* Function Name  : int fgetc(FILE *f)
* Description    : Retargets the C library printf function to the USART.fgetc�ض���
* Input          : None
* Output         : None
* Return         : ��ȡ�����ַ�
*******************************************************************************/
int fgetc(FILE *f)
{
  /* Loop until received a char */
  while(!(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET))
  {
  }
  
    /* Read a character from the USART and RETURN */
  return (USART_ReceiveData(USART1));
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
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
