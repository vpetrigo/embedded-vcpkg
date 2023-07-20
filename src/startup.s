/**
  ******************************************************************************
  * @file      startup_stm32f072xb.s
  * @author    MCD Application Team
  * @brief     STM32F072x8/STM32F072xB devices vector table for GCC toolchain.
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M0 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

  .syntax unified
  .cpu cortex-m0
  .fpu softvfp
  .thumb

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss
.equ        NVIC_INT_CTRL_CONST, 0xE000ED04


CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit
  blx lr

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  b .
/* Copy the vector table to SRAM */
/*  ldr r0, =__vector_start__
  ldr r1, =__vector_end__
  ldr r2, =__vector_copy_start__
  movs r3, #0
  bl LoopCopyDataInit */

/* Copy the data segment initializers from flash to SRAM */
/*  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  bl LoopCopyDataInit */
  
/* Zero fill the bss segment. */
/*  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss */

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Call the clock system intitialization function.*/
/*  bl  SystemInit */
/* Call static constructors */
/*  bl __libc_init_array */
/* Call the application's entry point.*/
/*  bl main */

LoopForever:
    b LoopForever


.size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .align  1
    .thumb_func
    .section .text.Default_Handler
    .globl  Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    b       Default_Handler_helper

    .size   Default_Handler, . - Default_Handler

    .align  1
    .thumb_func
    .section .text.Default_Handler_helper
    .globl  Default_Handler_helper
    .type   Default_Handler_helper, %function
Default_Handler_helper:
    /* Extract the number of currently running IRQ */
    ldr     r3, =NVIC_INT_CTRL_CONST
    ldr     a1, [r3]
    uxtb    a1, a1
    /* Arg1 (R0) now contains IRQ number that caused exception */
    /* Call default_handler that prints the IRQ number for us */
    push    {lr}
    /* bl      cortexm_irq_default_handler */
    b .
    pop     {r0}
    mov     lr, r0
    b       .

    .size   Default_Handler_helper, . - Default_Handler_helper

    .align  1
    .thumb_func
    .section .text.HardFault_Handler
    .globl   HardFault_Handler
    .type   HardFault_Handler, %function
HardFault_Handler:
    ldr     r0, =4
    mov     r1, lr
    tst     r1, r0
    beq load_msp
    mrs   r0, psp
    b continue
load_msp:
    mrs   r0, msp
continue:
    push    {lr}
    /* bl      cortexm_irq_hardfault_handler */
    b .
    pop     {r0}
    mov     lr, r0
    b       .

    .size   HardFault_Handler, . - HardFault_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex M0.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
  .align 8
  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors


g_pfnVectors:
  .word  _estack
  .word  Reset_Handler
  .word  NMI_Handler
  .word  HardFault_Handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  0
  .word  0
  .word  PendSV_Handler
  .word  SysTick_Handler
  .word  WWDG_IRQHandler                   /* Window WatchDog              */
  .word  PVD_VDDIO2_IRQHandler             /* PVD and VDDIO2 through EXTI Line detect */
  .word  RTC_IRQHandler                    /* RTC through the EXTI line    */
  .word  FLASH_IRQHandler                  /* FLASH                        */
  .word  RCC_CRS_IRQHandler                /* RCC and CRS                  */
  .word  EXTI0_1_IRQHandler                /* EXTI Line 0 and 1            */
  .word  EXTI2_3_IRQHandler                /* EXTI Line 2 and 3            */
  .word  EXTI4_15_IRQHandler               /* EXTI Line 4 to 15            */
  .word  TSC_IRQHandler                    /* TSC                          */
  .word  DMA1_Channel1_IRQHandler          /* DMA1 Channel 1               */
  .word  DMA1_Channel2_3_IRQHandler        /* DMA1 Channel 2 and Channel 3 */
  .word  DMA1_Channel4_5_6_7_IRQHandler    /* DMA1 Channel 4, Channel 5, Channel 6 and Channel 7*/
  .word  ADC1_COMP_IRQHandler              /* ADC1, COMP1 and COMP2         */
  .word  TIM1_BRK_UP_TRG_COM_IRQHandler    /* TIM1 Break, Update, Trigger and Commutation */
  .word  TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */
  .word  TIM2_IRQHandler                   /* TIM2                         */
  .word  TIM3_IRQHandler                   /* TIM3                         */
  .word  TIM6_DAC_IRQHandler               /* TIM6 and DAC                 */
  .word  TIM7_IRQHandler                   /* TIM7                         */
  .word  TIM14_IRQHandler                  /* TIM14                        */
  .word  TIM15_IRQHandler                  /* TIM15                        */
  .word  TIM16_IRQHandler                  /* TIM16                        */
  .word  TIM17_IRQHandler                  /* TIM17                        */
  .word  I2C1_IRQHandler                   /* I2C1                         */
  .word  I2C2_IRQHandler                   /* I2C2                         */
  .word  SPI1_IRQHandler                   /* SPI1                         */
  .word  SPI2_IRQHandler                   /* SPI2                         */
  .word  USART1_IRQHandler                 /* USART1                       */
  .word  USART2_IRQHandler                 /* USART2                       */
  .word  USART3_4_IRQHandler               /* USART3 and USART4            */
  .word  CEC_CAN_IRQHandler                /* CEC and CAN                  */
  .word  USB_IRQHandler                    /* USB                          */

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers.
 */
    .macro  def_irq_handler     handler_name
    .weak   \handler_name
    .set    \handler_name, Default_Handler
    .endm

  def_irq_handler NMI_Handler
  def_irq_handler SVC_Handler
  def_irq_handler PendSV_Handler
  def_irq_handler SysTick_Handler
  def_irq_handler WWDG_IRQHandler
  def_irq_handler PVD_VDDIO2_IRQHandler
  def_irq_handler RTC_IRQHandler
  def_irq_handler FLASH_IRQHandler
  def_irq_handler RCC_CRS_IRQHandler
  def_irq_handler EXTI0_1_IRQHandler
  def_irq_handler EXTI2_3_IRQHandler
  def_irq_handler EXTI4_15_IRQHandler
  def_irq_handler TSC_IRQHandler
  def_irq_handler DMA1_Channel1_IRQHandler
  def_irq_handler DMA1_Channel2_3_IRQHandler
  def_irq_handler DMA1_Channel4_5_6_7_IRQHandler
  def_irq_handler ADC1_COMP_IRQHandler
  def_irq_handler TIM1_BRK_UP_TRG_COM_IRQHandler
  def_irq_handler TIM1_CC_IRQHandler
  def_irq_handler TIM2_IRQHandler
  def_irq_handler TIM3_IRQHandler
  def_irq_handler TIM6_DAC_IRQHandler
  def_irq_handler TIM7_IRQHandler
  def_irq_handler TIM14_IRQHandler
  def_irq_handler TIM15_IRQHandler
  def_irq_handler TIM16_IRQHandler
  def_irq_handler TIM17_IRQHandler
  def_irq_handler I2C1_IRQHandler
  def_irq_handler I2C2_IRQHandler
  def_irq_handler SPI1_IRQHandler
  def_irq_handler SPI2_IRQHandler
  def_irq_handler USART1_IRQHandler
  def_irq_handler USART2_IRQHandler
  def_irq_handler USART3_4_IRQHandler
  def_irq_handler CEC_CAN_IRQHandler
  def_irq_handler USB_IRQHandler

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

