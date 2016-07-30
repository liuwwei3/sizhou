// ����ң�����˴���
// ����: ��ΰΰ

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "stdio.h"
#include "NRF_func.h"

GPIO_InitTypeDef GPIO_InitStructure; //GPIO ״̬�ָ�Ĭ��
USART_InitTypeDef USART_InitStructure;               //�������ûָ�Ĭ�ϲ���
ADC_InitTypeDef ADC_InitSturature;
DMA_InitTypeDef DMA_InitStructure;
 
#define N 50
#define M 8  
char rx_data[10];
vu16 AD_Value[N][M]; //�������ADCת�������Ҳ��DMA��Ŀ���ַ
vu16 After_filter[M]; //���������ƽ��ֵ֮��Ľ��

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
	return (u16)(advalue * 330 / 4096);   //��Ľ��������100���������������С��    
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

void USART_Configuration(void)                       //���ڳ�ʼ������
{
	//UART and LED clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO , ENABLE);//�򿪴���ʱ�� 	

	//GPIOB led
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 ; //�ܽ�λ�ö��壬��ſ�
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//����ٶ�50MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);//B ��GPIO ��ʼ��
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	 //  no use!!!

	//UART pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//�ܽ�9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure); //TX��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//�ܽ�10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//RX��ʼ��

	//UART mode
    USART_InitStructure.USART_BaudRate = 9600;                                   //������2400
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //�ֳ�8λ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                  //1λֹͣ�ֽ�
    USART_InitStructure.USART_Parity = USART_Parity_No;                    //����żУ��
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//��Rx���պ�Tx���͹���
	USART_Init(USART1, &USART_InitStructure);                                          //��ʼ��
    USART_Cmd(USART1, ENABLE);                                                        //��������
}

void ADC_Configuration()
{
	//UART and DMA clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO |RCC_APB2Periph_ADC1 , ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //72M/6=12,ADC���ʱ�䲻�ܳ���14M 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //ʹ��DMA����
	
	//ADC pins config 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2 | GPIO_Pin_3|
								  GPIO_Pin_4 | GPIO_Pin_5|GPIO_Pin_6 | GPIO_Pin_7;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	//ADC config
	ADC_DeInit(ADC1);
	ADC_InitSturature.ADC_Mode = ADC_Mode_Independent;
	ADC_InitSturature.ADC_ScanConvMode =ENABLE; //ģ��ת��������ɨ��ģʽ
	ADC_InitSturature.ADC_ContinuousConvMode = ENABLE; //ģ��ת������������ת��ģʽ 
	ADC_InitSturature.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //�ⲿ����ת���ر�
	ADC_InitSturature.ADC_DataAlign = ADC_DataAlign_Right; //ADC�����Ҷ���
	ADC_InitSturature.ADC_NbrOfChannel = M; //˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitSturature); //����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���
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
	while(ADC_GetResetCalibrationStatus(ADC1)); //��ȡADC1��λУ׼�Ĵ�����״̬,���� ״̬��ȴ�
	ADC_StartCalibration(ADC1);  //��ʼָ��ADC1��У׼״̬
	while(ADC_GetCalibrationStatus(ADC1));   //��ȡָ��ADC1��У׼����,����״̬�� �ȴ�
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	//DMA config
	DMA_DeInit(DMA1_Channel1);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	DMA_InitStructure.DMA_PeripheralBaseAddr =  (u32)&ADC1->DR;  //DMA����ADC����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value;  //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //�ڴ���Ϊ���ݴ����Ŀ�ĵ�
	DMA_InitStructure.DMA_BufferSize = N*M;  //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ��� ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //���ݿ�� Ϊ16λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //������ѭ������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMAͨ�� xӵ�и����ȼ�
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��
	DMA_Cmd(DMA1_Channel1, ENABLE);  //����DMAͨ��
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
	/*  ����SPI1ģ�� ��ʼ��*/

   	SPI_I2S_DeInit(SPI2);                          
   	SPI_Cmd(SPI2, DISABLE); //�����Ƚ���,���ܸı�MODE
   	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����ȫ˫��
   	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;       //��
   	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;      //8λ
   	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;        //CPOL=0 ʱ�����յ�
   	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;       //CPHA=0 ���ݲ����1��
   	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;        //���NSS
   	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8 ; //256��Ƶ
   	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;      //��λ��ǰ
   	SPI_InitStructure.SPI_CRCPolynomial = 7;        //CRC7
   	SPI_Init(SPI2, &SPI_InitStructure);
   	SPI_Cmd(SPI2, ENABLE);
   	//SPI_ReadWriteByte(0xff);   //��������
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
