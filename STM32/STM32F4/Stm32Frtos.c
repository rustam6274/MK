#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
//#include "croutine.h"
#include "semphr.h"
//******************************************************************************

void vLedBlinkBlue(void *pvParameters) __attribute__ ((noreturn));
void vLedBlinkBlue1(void *pvParameters) __attribute__ ((noreturn));
void vLedBlinkRed(void *pvParameters) __attribute__ ((noreturn));

//static SemaphoreHandle_t xSemaphore = NULL;

float Task1 = 0;
float Task2 = 0;
float Task3 = 0;

void Delay( void )
{
    for( uint16_t i = 0; i <= 50000; i++ )
        for( uint16_t j = 0; j <= 25; j++ );
}

#define STACK_SIZE_MIN	128

//******************************************************************************
int main(void)
{
    RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN; // включим тактирование порта
    //GPIOC->MODER = 0x55000000; // включим ножки 12,13,14,15 на выход
    //GPIOC->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
    GPIOC->MODER |= GPIO_MODER_MODER13_0;
    GPIOC->OTYPER = 0; //подтянем резистор ко всем ножкам порта
    GPIOC->OSPEEDR = 0; //установим скорость LOW на все лапки порта 
	
	xTaskCreate( vLedBlinkBlue, "Led Blink Task Blue", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
    xTaskCreate( vLedBlinkBlue1, "Led Blink Task Blue", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
	xTaskCreate( vLedBlinkRed, "Led Blink Task Red", 
		STACK_SIZE_MIN, NULL, tskIDLE_PRIORITY, NULL );
    //xSemaphore = xSemaphoreCreateBinary();
    //xSemaphoreGive(xSemaphore);
	
	vTaskStartScheduler();
}
//******************************************************************************

//******************************************************************************
void vLedBlinkBlue(void *pvParameters)
{
    (void) pvParameters;
	for(;;)
	{
		//xSemaphoreTake(xSemaphore,(TickType_t) portMAX_DELAY);
        GPIOC->BSRR |= GPIO_BSRR_BS13; // выставить в единицу
        //xSemaphoreGive(xSemaphore);
        Task1 = 1;
        Delay();
		vTaskDelay( 500 / portTICK_RATE_MS );
        GPIOC->BSRR |= GPIO_BSRR_BR13; // сбросить в ноль
        //xSemaphoreGive(xSemaphore);
        Task1 = 0;
        vTaskDelay( 1000 / portTICK_RATE_MS );
	}
}

void vLedBlinkBlue1(void *pvParameters)
{
    (void) pvParameters;
	for(;;)
	{
		//xSemaphoreTake(xSemaphore,(TickType_t) portMAX_DELAY);
        //GPIOC->BSRR |= GPIO_BSRR_BS13; // выставить в единицу
        //xSemaphoreGive(xSemaphore);
        Task2 = 1;
        //Delay();
		vTaskDelay( 500 / portTICK_RATE_MS );
        //GPIOC->BSRR |= GPIO_BSRR_BR13; // сбросить в ноль
        //xSemaphoreGive(xSemaphore);
        Task2 = 0;
        vTaskDelay( 1000 / portTICK_RATE_MS );
	}
}

void vLedBlinkRed(void *pvParameters)
{
    (void) pvParameters;
	for(;;)
	{
		//xSemaphoreTake(xSemaphore,(TickType_t) portMAX_DELAY);
        //GPIOC->BSRR |= GPIO_BSRR_BS13; // выставить в единицу
        //xSemaphoreGive(xSemaphore);
        Task3 = 1;
		vTaskDelay( 500 / portTICK_RATE_MS );
        //xSemaphoreTake(xSemaphore,(TickType_t) portMAX_DELAY);
        //GPIOC->BSRR |= GPIO_BSRR_BR13; // сбросить в ноль
        //xSemaphoreGive(xSemaphore);
        Task3 = 0;
        vTaskDelay( 500 / portTICK_RATE_MS );
	}
}