/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-07-24     Tanek        the first version
 * 2018-11-12     Ernest Chen  modify copyright
 */
 
#include <stdint.h>
#include <rthw.h>
#include <rtthread.h>

#include <stm32l4xx.h>

extern void SystemClock_Config(void);

/*
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (huart->Instance == USART2)  // 这里要改
	{
		__HAL_RCC_USART2_CLK_ENABLE();   // 这里要改
		__HAL_RCC_GPIOA_CLK_ENABLE();
		
		GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
}
*/

// static UART_HandleTypeDef Uart2Handle; 

extern UART_HandleTypeDef huart4;

void MY_UART4_Init(void)
{
	huart4.Instance = UART4;   // 这里要改
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK )
	{
		while(1);
	}
	return ;
}


void rt_hw_console_output(const char *str)
{
        rt_size_t i = 0, size = 0;
        char a = '\r';
        
        __HAL_UNLOCK(&huart4);
        
        size = rt_strlen(str);
        for (i = 0; i < size; i++)
        {
        if (*(str + i) == '\n')
        {
                HAL_UART_Transmit(&huart4, (uint8_t*)&a, 1, 10);
        }
                HAL_UART_Transmit(&huart4, (uint8_t*)(str + i), 1, 10);
        }
}

int rt_hw_console_getchar(void)
{
        int ch = -1;
        if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE) != RESET)
        {
                ch = huart4.Instance->RDR & 0xff;
        }
        else
        {
                if(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_ORE) != RESET)
                {
                        __HAL_UART_CLEAR_OREFLAG(&huart4);
									
                }
                rt_thread_mdelay(10);
        }
        return ch;
}


#define _SCB_BASE       (0xE000E010UL)
#define _SYSTICK_CTRL   (*(rt_uint32_t *)(_SCB_BASE + 0x0))
#define _SYSTICK_LOAD   (*(rt_uint32_t *)(_SCB_BASE + 0x4))
#define _SYSTICK_VAL    (*(rt_uint32_t *)(_SCB_BASE + 0x8))
#define _SYSTICK_CALIB  (*(rt_uint32_t *)(_SCB_BASE + 0xC))
#define _SYSTICK_PRI    (*(rt_uint8_t  *)(0xE000ED23UL))

// Updates the variable SystemCoreClock and must be called 
// whenever the core clock is changed during program execution.
extern void SystemCoreClockUpdate(void);

// Holds the system core clock, which is the system clock 
// frequency supplied to the SysTick timer and the processor 
// core clock.
extern uint32_t SystemCoreClock;

static uint32_t _SysTick_Config(rt_uint32_t ticks)
{
    if ((ticks - 1) > 0xFFFFFF)
    {
        return 1;
    }
    
    _SYSTICK_LOAD = ticks - 1; 
    _SYSTICK_PRI = 0xFF;
    _SYSTICK_VAL  = 0;
    _SYSTICK_CTRL = 0x07;  
    return 0;
}


#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
#define RT_HEAP_SIZE 1024
static uint32_t rt_heap[RT_HEAP_SIZE];     // heap default size: 4K(1024 * 4)
RT_WEAK void *rt_heap_begin_get(void)
{
    return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
    return rt_heap + RT_HEAP_SIZE;
}
#endif

/**
 * This function will initial your board.
 */


void rt_hw_board_init()
{
	  HAL_Init();
//    MX_UART4_Init();
	
//    MY_UART4_Init();
	
		MX_UART4_Init();
	  rt_kprintf("Uart init successfully\n");
	
	
	  SystemClock_Config();
	
    SystemCoreClockUpdate();
    
    /* System Tick Configuration */
    _SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);

    /* Call components board initial (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
}

void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

	  HAL_IncTick();
    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}
