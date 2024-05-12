#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

#include "main.h"
#include "stm32f30x_adc.h"
#include "stm32f30x.h"

#include "stm32f30x_spi.h"
#include "stm32f30x_tim.h"

#include <stdio.h>

static unsigned int i;
static unsigned int ii = 0;
static unsigned int delayTime = 400000;
static uint8_t Send_Buffer[20];

#define DUMMY          0x00
#define TIMEOUT_TIME   0x1000

static SPI_InitTypeDef spi;
static TIM_TimeBaseInitTypeDef timer;
static GPIO_InitTypeDef gpio;
static uint8_t receiveData[2];
static uint16_t timeout;
static uint8_t tempByte;
static uint16_t xResult;
static float xPosition;
static uint8_t xSign;

void sss_1(void)
{
    ii = 1;
}
void sss_2(void)
{
    ii = 2;
}
void sss_0(void)
{
    ii = 0;
}
void sss_v0(void)
{
    if(delayTime < 400000) delayTime+=100000;
}

static void delay_Time(unsigned int delayTime_)
{
    for (i = 0; i < delayTime_; i++);
}

static GPIO_InitTypeDef GPIO_InitStructure;
static ADC_InitTypeDef ADC_InitStructure;
static ADC_CommonInitTypeDef ADC_CommonInitStructure;

static void Configure_ADC(void)
{
    RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_5);
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    
    ADC_StructInit( &ADC_InitStructure );
	ADC_VoltageRegulatorCmd( ADC1, ENABLE );

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
    ADC_InitStructure.ADC_NbrOfRegChannel = 1;
    ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
    ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
    ADC_Init(ADC1,&ADC_InitStructure);

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;
	ADC_CommonInit( ADC1, &ADC_CommonInitStructure );  
    ADC_RegularChannelConfig(ADC1,ADC_Channel_2, 1, ADC_SampleTime_1Cycles5);
    ADC_Cmd(ADC1, ENABLE);
    
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &gpio); 
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;
    spi.SPI_Mode = SPI_Mode_Master;
    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);
    SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
    SPI_DataSizeConfig(SPI1, ENABLE);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 720 - 1;
    timer.TIM_Period = 2000;
    TIM_TimeBaseInit(TIM2, &timer);
}

static void USB_Write1(float data) // static void USB_Write(uint16_t data)
{
    sprintf((char*)Send_Buffer, "%f", data);
    USB_SIL_Write(EP1_IN, Send_Buffer, sizeof(Send_Buffer));
    SetEPTxValid(ENDP1);
}
static void USB_Write(uint16_t data)
{
    sprintf((char*)Send_Buffer, "%d", data);
    USB_SIL_Write(EP1_IN, Send_Buffer, sizeof(Send_Buffer));
    SetEPTxValid(ENDP1);
}

static uint16_t Read_ADC(void)
{
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));
    ADC_RegularChannelConfig(ADC1,ADC_Channel_2, 1, ADC_SampleTime_1Cycles5);
    ADC_StartConversion(ADC1); // Функция запускает преобразование
    while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
    return ADC_GetConversionValue(ADC1)*100/4096; // Получаем результат из ADC
}

uint8_t sendByte(uint8_t byteToSend);
uint8_t sendByte(uint8_t byteToSend)
{
    timeout = TIMEOUT_TIME;
    while ((SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) & (timeout != 0))
    {
        timeout--;
    } 
    SPI_SendData8(SPI1, byteToSend);
    timeout = TIMEOUT_TIME; 
    while ((SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) & (timeout != 0))
    {
        timeout--;
    }
    return (uint8_t)SPI_ReceiveData8(SPI1);
}

void writeData(uint8_t address, uint8_t dataToWrite)
{
    GPIO_ResetBits(GPIOE, GPIO_Pin_3);
    sendByte(address);
    sendByte(dataToWrite);
    GPIO_SetBits(GPIOE, GPIO_Pin_3);
}

uint8_t readData(uint8_t address)
{
    GPIO_ResetBits(GPIOE, GPIO_Pin_3);
    sendByte(address);
    tempByte = sendByte(DUMMY);
    GPIO_SetBits(GPIOE, GPIO_Pin_3);
    return tempByte; 
}

int main(void)
{
  Set_System();
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();
  Configure_ADC();
    
  __enable_irq();
  xPosition = 0;
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
  NVIC_EnableIRQ(TIM2_IRQn);
  writeData(0x20, 0x0F);
  writeData(0x23, 0x30);
    
  while (1)
  {	
      if(ii == 1)
      {
          GPIO_ResetBits(GPIOE, GPIO_Pin_12 | GPIO_Pin_8); // Сбросить бит или биты (выключить пин или пины)
          GPIO_SetBits(GPIOE, GPIO_Pin_9 | GPIO_Pin_13); // Установить бит или биты (включить пин или пины)
          USB_Write(Read_ADC());
          delay_Time(delayTime);
          GPIO_ResetBits(GPIOE, GPIO_Pin_9 | GPIO_Pin_13);
          GPIO_SetBits(GPIOE, GPIO_Pin_10 | GPIO_Pin_14);
          USB_Write(Read_ADC());
          delay_Time(delayTime);
          GPIO_ResetBits(GPIOE, GPIO_Pin_10 | GPIO_Pin_14);
          GPIO_SetBits(GPIOE, GPIO_Pin_15 | GPIO_Pin_11);
          USB_Write(Read_ADC());
          delay_Time(delayTime);
          GPIO_ResetBits(GPIOE, GPIO_Pin_15 | GPIO_Pin_11);
          GPIO_SetBits(GPIOE, GPIO_Pin_12 | GPIO_Pin_8);
          USB_Write(Read_ADC());
          delay_Time(delayTime);
          
          if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) == 1) //  В режиме входного порта получить значение бита (узнать состояние пина)
          {
            if(delayTime > 100000) delayTime-=60000;
          }
      }
  }
}
void TIM2_IRQHandler(void);
void TIM2_IRQHandler(void)
{
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    GPIO_ResetBits(GPIOE, GPIO_Pin_3);
    // z - sendByte(0xEC); y - sendByte(0xEA);  x - sendByte(0xE8);
    sendByte(0xE8);    
    receiveData[0] = sendByte(0x00);
    receiveData[1] = sendByte(0x00);
    GPIO_SetBits(GPIOE, GPIO_Pin_3);
    xResult = receiveData[0] | (receiveData[1] << 8);
    if ((xResult & 0x8000) == 0)
    {
        xSign = 0;
    } 
    else
    {
        xSign = 1;
        xResult &= 0x7FFF;
        xResult = 0x8000 - xResult;
    }
    if (xResult < 0x0A)
    {
        xResult = 0;
    }
    if (xSign == 0)
    {
        xPosition += 0.07 * xResult * 0.02;
    }
    else
    {
        xPosition -= 0.07 * xResult * 0.02;
    }
    if(ii == 2) {
        float xPosition1 = xPosition;
        USB_Write1(xPosition);
        if(xPosition < 0) xPosition1 = -1*xPosition;
         // USB_Write((uint16_t)xPosition1);
        
        if(xPosition >= 80) {GPIO_SetBits(GPIOE, GPIO_Pin_15);}
        else if(xPosition1 >= 70 && xPosition1 < 80) {GPIO_SetBits(GPIOE, GPIO_Pin_14); GPIO_ResetBits(GPIOE, GPIO_Pin_15);}
        else if(xPosition1 >= 60 && xPosition1 < 70) {GPIO_SetBits(GPIOE, GPIO_Pin_13); GPIO_ResetBits(GPIOE, GPIO_Pin_14);}
        else if(xPosition1 >= 50 && xPosition1 < 60) {GPIO_SetBits(GPIOE, GPIO_Pin_12); GPIO_ResetBits(GPIOE, GPIO_Pin_13);}
        else if(xPosition1 >= 40 && xPosition1 < 50) {GPIO_SetBits(GPIOE, GPIO_Pin_11); GPIO_ResetBits(GPIOE, GPIO_Pin_12);}
        else if(xPosition1 >= 30 && xPosition1 < 40) {GPIO_SetBits(GPIOE, GPIO_Pin_10); GPIO_ResetBits(GPIOE, GPIO_Pin_11);}
        else if(xPosition1 >= 20 && xPosition1 < 30) {GPIO_SetBits(GPIOE, GPIO_Pin_9); GPIO_ResetBits(GPIOE, GPIO_Pin_10);}
        else if(xPosition1 >= 10 && xPosition1 < 20) {GPIO_SetBits(GPIOE, GPIO_Pin_8); GPIO_ResetBits(GPIOE, GPIO_Pin_9);}
        else if(xPosition1 < 10) {GPIO_SetBits(GPIOE, GPIO_Pin_7); GPIO_ResetBits(GPIOE, GPIO_Pin_8);}
    }
}