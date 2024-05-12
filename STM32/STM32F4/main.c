//#include "stm32f4xx.h"
 
/*
static void PortSetHi(void){
  GPIOC->BSRR |= GPIO_BSRR_BS13; // выставить в единицу
}
static void PortSetLow(void){
  GPIOC->BSRR |= GPIO_BSRR_BR13; // сбросить в ноль
}
 
int main(void){
    uint32_t i;
    RCC->AHB1ENR|=RCC_AHB1ENR_GPIOCEN; // включим тактирование порта
    //GPIOC->MODER = 0x55000000; // включим ножки 12,13,14,15 на выход
    //GPIOC->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
    GPIOC->MODER |= GPIO_MODER_MODER13_0;
    GPIOC->OTYPER = 0; //подтянем резистор ко всем ножкам порта
    GPIOC->OSPEEDR = 0; //установим скорость LOW на все лапки порта
    while(1){
        //GPIOC->BSRR |= GPIO_BSRR_BS13; // выставить в единицу
        PortSetHi();
        for(i=0;i<500000;i++){}
        //GPIOC->BSRR |= GPIO_BSRR_BR13; // сбросить в ноль
        PortSetLow();
        for(i=0;i<500000;i++){}
    }
}
*/
 
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
/* The task functions prototype*/
void vTask1( void *pvParameters );
void vTask2( void *pvParameters );
/* Task parameter to be sent to the task function */
static char *pvTask1  = "Task1 is running.";
static char *pvTask2  = "Task2 is running.";
static SemaphoreHandle_t xSemaphore = NULL;
/* Extern functions */
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
extern int stdout_init (void);
/*-----------------------------------------------------------*/

 
int main( void )
{
/* Board initializations */
        SystemInit();
/* This function initializes the MCU clock, PLL will be used to generate Main MCU clock */  
    SystemCoreClockUpdate();
/* Initialize the serial I/O(console ), making standard output to be send to USART1 */
    stdout_init();
    printf("Initialization is done.\r\n");
    /* Create one of the two tasks. */
    xTaskCreate(vTask1, /* Pointer to the function that implements the task. */
    "Task 1", /* Text name for the task.  This is to facilitate debugging only. */
        configMINIMAL_STACK_SIZE, /* Stack depth in words. */
    (void*)pvTask1,        /* We are not using the task parameter. */
    1,            /* This task will run at priority 1. */
    NULL );        /* We are not using the task handle. */
    
        /* Create the other task in exactly the same way. */
    xTaskCreate( vTask2, "Task 2", configMINIMAL_STACK_SIZE, (void*)pvTask2, 1, NULL );
        /* Create a binary semaphore */
    xSemaphore = xSemaphoreCreateBinary();
    /* make the semaphore token available for the first time */
        xSemaphoreGive( xSemaphore);
    /* Start the scheduler so our tasks start executing. */
    vTaskStartScheduler();
    /* If all is well we will never reach here as the scheduler will now be
    running.  If we do reach here then it is likely that there was insufficient
    heap available for the idle task to be created. */
    for( ;; );
}
/*-----------------------------------------------------------*/
 
void vTask1( void *pvParameters ) __attribute__ ((noreturn));
void vTask2( void *pvParameters ) __attribute__ ((noreturn));
 
void vTask1( void *pvParameters )
{
char *pcTaskName = (char *) pvParameters;
    /* Task is implemented in an infinite loop. */
    for( ;; )
    {
        /* Take semaphore */
        xSemaphoreTake(xSemaphore,(TickType_t) portMAX_DELAY);
        /* Print out the name of this task. */
          printf( "%s\r\n",pcTaskName );
        /* Give semaphore */
        xSemaphoreGive(xSemaphore);
        /* Delay for a period. */
        vTaskDelay( 2000 / portTICK_PERIOD_MS );
    }
}
/*-----------------------------------------------------------*/
void vTask2( void *pvParameters )
{
char *pcTaskName = (char *) pvParameters;
    /* Task is implemented in an infinite loop. */
    for( ;; )
    {
        /* Take semaphore */
        xSemaphoreTake(xSemaphore,(TickType_t) portMAX_DELAY);
        /* Print out the name of this task. */
          printf( "%s\r\n",pcTaskName );
        /* Give semaphore */
        xSemaphoreGive(xSemaphore);
        /* Delay for a period. */
        vTaskDelay( 2000 / portTICK_PERIOD_MS );
    }
}