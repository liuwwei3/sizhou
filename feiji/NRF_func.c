#include "stm32f10x_spi.h"
#include "stm32f10x_lib.h"
#include "NRF_func.h"
#include "stdio.h"

const u8 TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ
const u8 RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ

// SPI�����ٶ����� 
#define SPI_SPEED_2   0
#define SPI_SPEED_8   1
#define SPI_SPEED_16  2
#define SPI_SPEED_256 3

extern void Delay(vu32 nCount);


void SPIx_SetSpeed(u8 SpeedSet)
{
	SPIx->CR1&=0XFFC7;//Fsck=Fcpu/256
	if(SpeedSet==SPI_SPEED_2)//����Ƶ
	{
		SPIx->CR1|=0<<3;//Fsck=Fpclk/2=36Mhz
	}else if(SpeedSet==SPI_SPEED_8)//�˷�Ƶ 
	{
		SPIx->CR1|=2<<3;//Fsck=Fpclk/8=9Mhz
	}else if(SpeedSet==SPI_SPEED_16)//ʮ����Ƶ
	{
		SPIx->CR1|=3<<3;//Fsck=Fpclk/16=4.5Mhz
	}else //256��Ƶ
	{
		SPIx->CR1|=7<<3; //Fsck=Fpclk/256=281.25Khz ����ģʽ
	}
	SPIx->CR1|=1<<6; //SPI�豸ʹ��
}

u8 SPI_ReadWriteByte(u8 data)
{ 
	u8 temp;
//	printf("  W%d \n  ",data); 
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);
    /* Send SPIx data */
    SPI_I2S_SendData(SPIx, data); 
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);
    /* Read SPIx received data */
    temp = SPI_I2S_ReceiveData(SPIx);
//	printf("  R%d \n  ",temp);      
	return temp;          //�����յ�������
}

void  NRF24L01_Init(void)
{
   CE_L();          //ʹ��24L01
   CSN_H();        //SPIƬѡȡ��
}

u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i;
//	SPIx_SetSpeed(SPI_SPEED_8); //spi�ٶ�Ϊ9Mhz��24L01�����SPIʱ��Ϊ10Mhz��    
	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //����д��ĵ�ַ  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;
	printf("i=%d\n",i);  
	if(i!=5)return 1;//���24L01����
	return 0; //��⵽24L01
}  

u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;
	CSN_L();                          //ʹ��SPI����
	status=SPI_ReadWriteByte(reg);    //���ͼĴ����� 
	SPI_ReadWriteByte(value);         //д��Ĵ�����ֵ
	CSN_H();                          //��ֹSPI����
	return(status);    //����״ֵ̬
}

u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;    
	CSN_L();                        //ʹ��SPI����
	SPI_ReadWriteByte(reg);         //���ͼĴ�����	
	reg_val=SPI_ReadWriteByte(0XFF);//��ȡ�Ĵ�������
	CSN_H();                        //��ֹSPI����    
	return(reg_val);                //����״ֵ̬
}

u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,u8_ctr;       
	CSN_L();                             //ʹ��SPI����
	status=SPI_ReadWriteByte(reg);       //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬ 
	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
		pBuf[u8_ctr]=SPI_ReadWriteByte(0XFF);//��������
	CSN_H();                             //��ֹSPI����
	return status;                       //���ض�����״ֵ̬
}

u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;    
	CSN_L();                        //ʹ��SPI����
	status = SPI_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
		SPI_ReadWriteByte(*pBuf++);     //д������  
	CSN_H();                        //�ر�SPI����
	return status;                  //���ض�����״ֵ̬
}

u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 sta;
	CE_L();
	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
	CE_H();                 //��������  ?
	while(GPIOC->IDR&1<<5); //�ȴ��������
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ  ?
	NRF24L01_Write_Reg(WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&MAX_TX)//�ﵽ����ط�����
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
		return MAX_TX; 
	}
	if(sta&TX_OK)//�������
	{
		return TX_OK;
	}
	return 0xff;//����ԭ����ʧ��
}

u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;       
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ      
	NRF24L01_Write_Reg(WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&RX_OK)//���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ��� 
		return 0; 
	}   
	return 1;//û�յ��κ�����
}    
 
void RX_Mode(void)
{
	CE_L();  
	NRF24L01_Write_Buf(WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
 
	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x01);             //ʹ��ͨ��0���Զ�Ӧ��    
	NRF24L01_Write_Reg(WRITE_REG+EN_RXADDR,0x01);         //ʹ��ͨ��0�Ľ��յ�ַ    
	NRF24L01_Write_Reg(WRITE_REG+RF_CH,40);              //����RFͨ��Ƶ��  
	NRF24L01_Write_Reg(WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��    
	NRF24L01_Write_Reg(WRITE_REG+RF_SETUP,0x0f);          //����TX�������,0db����,2Mbps,���������濪��  ?
	NRF24L01_Write_Reg(WRITE_REG+CONFIG, 0x0f);           //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
	CE_H();    //CEΪ��,�������ģʽ 
}  
 
void TX_Mode(void)
{  
	CE_L();    
	NRF24L01_Write_Buf(WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
	NRF24L01_Write_Buf(WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK  

	NRF24L01_Write_Reg(WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
	NRF24L01_Write_Reg(WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
	NRF24L01_Write_Reg(WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
	NRF24L01_Write_Reg(WRITE_REG+RF_CH,40);       //����RFͨ��Ϊ40
	NRF24L01_Write_Reg(WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��  ?
	NRF24L01_Write_Reg(WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	CE_H(); //CEΪ��,10us����������
}  
