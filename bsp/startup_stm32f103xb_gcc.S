/*******************************************************************************
* File Name          : startup_stm32f103xb_gcc.S
* Author             : MCD Application Team
* Description        : STM32F103xB medium-density devices vector table for GCC
*                      toolchain.  This module performs:
*                      - Set the initial SP
*                      - Set the initial PC == Reset_Handler
*                      - Set the vector table entries with the exceptions ISR
*                        address
*                      - Copies the .data segment from flash to SRAM
*                      - Zero-fills the .bss segment (the .noinit section is
*                        left untouched)
*                      - Calls __libc_init_array (C run-time init arrays)
*                      - Branches to main().
*                      After Reset the Cortex-M3 processor is in Thread mode,
*                      priority is Privileged, and the Stack is set to Main.
*******************************************************************************
* @attention
*
* Copyright (c) 2021 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
*******************************************************************************/

  .syntax unified
  .cpu cortex-m3
  .thumb


/* Vector Table Mapped to Address 0 at Reset */

  .section  .vectors,"a",%progbits
  .global  __Vectors
  .global  __Vectors_End
  .global  __Vectors_Size

__Vectors:
  /* Core exceptions */
  .long   __StackTop                        /* Top of Stack */
  .long   Reset_Handler                     /* Reset Handler */
  .long   NMI_Handler                       /* NMI Handler */
  .long   HardFault_Handler                 /* Hard Fault Handler */
  .long   MemManage_Handler                 /* MPU Fault Handler */
  .long   BusFault_Handler                  /* Bus Fault Handler */
  .long   UsageFault_Handler                /* Usage Fault Handler */
  .long   0                                 /* Reserved */
  .long   0                                 /* Reserved */
  .long   0                                 /* Reserved */
  .long   0                                 /* Reserved */
  .long   SVC_Handler                       /* SVCall Handler */
  .long   DebugMon_Handler                  /* Debug Monitor Handler */
  .long   0                                 /* Reserved */
  .long   PendSV_Handler                    /* PendSV Handler */
  .long   SysTick_Handler                   /* SysTick Handler */
  /* External Interrupts */
  .long   WWDG_IRQHandler                   /* Window Watchdog */
  .long   PVD_IRQHandler                    /* PVD through EXTI Line detection */
  .long   TAMPER_IRQHandler                 /* Tamper */
  .long   RTC_IRQHandler                    /* RTC */
  .long   FLASH_IRQHandler                  /* FLASH */
  .long   RCC_IRQHandler                    /* RCC */
  .long   EXTI0_IRQHandler                  /* EXTI Line 0 */
  .long   EXTI1_IRQHandler                  /* EXTI Line 1 */
  .long   EXTI2_IRQHandler                  /* EXTI Line 2 */
  .long   EXTI3_IRQHandler                  /* EXTI Line 3 */
  .long   EXTI4_IRQHandler                  /* EXTI Line 4 */
  .long   DMA1_Channel1_IRQHandler          /* DMA1 Channel 1 */
  .long   DMA1_Channel2_IRQHandler          /* DMA1 Channel 2 */
  .long   DMA1_Channel3_IRQHandler          /* DMA1 Channel 3 */
  .long   DMA1_Channel4_IRQHandler          /* DMA1 Channel 4 */
  .long   DMA1_Channel5_IRQHandler          /* DMA1 Channel 5 */
  .long   DMA1_Channel6_IRQHandler          /* DMA1 Channel 6 */
  .long   DMA1_Channel7_IRQHandler          /* DMA1 Channel 7 */
  .long   ADC1_2_IRQHandler                 /* ADC1 and ADC2 */
  .long   USB_HP_CAN1_TX_IRQHandler         /* USB High Priority or CAN1 TX */
  .long   USB_LP_CAN1_RX0_IRQHandler        /* USB Low Priority or CAN1 RX0 */
  .long   CAN1_RX1_IRQHandler               /* CAN1 RX1 */
  .long   CAN1_SCE_IRQHandler               /* CAN1 SCE */
  .long   EXTI9_5_IRQHandler                /* EXTI Lines 9..5 */
  .long   TIM1_BRK_IRQHandler               /* TIM1 Break */
  .long   TIM1_UP_IRQHandler                /* TIM1 Update */
  .long   TIM1_TRG_COM_IRQHandler           /* TIM1 Trigger and Commutation */
  .long   TIM1_CC_IRQHandler                /* TIM1 Capture Compare */
  .long   TIM2_IRQHandler                   /* TIM2 */
  .long   TIM3_IRQHandler                   /* TIM3 */
  .long   TIM4_IRQHandler                   /* TIM4 */
  .long   I2C1_EV_IRQHandler                /* I2C1 Event */
  .long   I2C1_ER_IRQHandler                /* I2C1 Error */
  .long   I2C2_EV_IRQHandler                /* I2C2 Event */
  .long   I2C2_ER_IRQHandler                /* I2C2 Error */
  .long   SPI1_IRQHandler                   /* SPI1 */
  .long   SPI2_IRQHandler                   /* SPI2 */
  .long   USART1_IRQHandler                 /* USART1 */
  .long   USART2_IRQHandler                 /* USART2 */
  .long   USART3_IRQHandler                 /* USART3 */
  .long   EXTI15_10_IRQHandler              /* EXTI Lines 15..10 */
  .long   RTCAlarm_IRQHandler               /* RTC Alarm through EXTI Line */
  .long   USBWakeUp_IRQHandler              /* USB Wakeup from suspend */

__Vectors_End:

  .equ  __Vectors_Size, __Vectors_End - __Vectors

  .section  .text,"ax",%progbits


/* Reset Handler */

  .global  Reset_Handler
  .type    Reset_Handler, %function
Reset_Handler:
  ldr   r0, =SystemInit
  blx   r0

/* Copy the .data segment initialisers from flash (__etext) to SRAM. */
  ldr   r1, =__etext
  ldr   r2, =__data_start__
  ldr   r3, =__data_end__

  subs  r3, r3, r2
  ble   .L_copy_done
.L_copy_loop:
  subs  r3, r3, #4
  ldr   r0, [r1, r3]
  str   r0, [r2, r3]
  bgt   .L_copy_loop
.L_copy_done:

/* Zero-fill the .bss segment. The .noinit section lies outside this
   range and is intentionally left untouched (persistent state). */
  movs  r0, #0
  ldr   r1, =__bss_start__
  ldr   r2, =__bss_end__

  subs  r2, r2, r1
  ble   .L_zero_done
.L_zero_loop:
  subs  r2, r2, #4
  str   r0, [r1, r2]
  bgt   .L_zero_loop
.L_zero_done:

/* Run C library init (preinit / init arrays). */
  ldr   r0, =__libc_init_array
  blx   r0

/* Branch to the application entry point. */
  ldr   r0, =main
  blx   r0

/* Should main ever return, trap here. */
.L_main_exited:
  b     .L_main_exited
  .size  Reset_Handler, . - Reset_Handler


/* Exception / IRQ default handlers — weak, can be overridden */

  .weak      NMI_Handler
  .thumb_set NMI_Handler, Default_Handler

  .weak      HardFault_Handler
  .thumb_set HardFault_Handler, Default_Handler

  .weak      MemManage_Handler
  .thumb_set MemManage_Handler, Default_Handler

  .weak      BusFault_Handler
  .thumb_set BusFault_Handler, Default_Handler

  .weak      UsageFault_Handler
  .thumb_set UsageFault_Handler, Default_Handler

  .weak      SVC_Handler
  .thumb_set SVC_Handler, Default_Handler

  .weak      DebugMon_Handler
  .thumb_set DebugMon_Handler, Default_Handler

  .weak      PendSV_Handler
  .thumb_set PendSV_Handler, Default_Handler

  .weak      SysTick_Handler
  .thumb_set SysTick_Handler, Default_Handler

  .weak      WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler, Default_Handler

  .weak      PVD_IRQHandler
  .thumb_set PVD_IRQHandler, Default_Handler

  .weak      TAMPER_IRQHandler
  .thumb_set TAMPER_IRQHandler, Default_Handler

  .weak      RTC_IRQHandler
  .thumb_set RTC_IRQHandler, Default_Handler

  .weak      FLASH_IRQHandler
  .thumb_set FLASH_IRQHandler, Default_Handler

  .weak      RCC_IRQHandler
  .thumb_set RCC_IRQHandler, Default_Handler

  .weak      EXTI0_IRQHandler
  .thumb_set EXTI0_IRQHandler, Default_Handler

  .weak      EXTI1_IRQHandler
  .thumb_set EXTI1_IRQHandler, Default_Handler

  .weak      EXTI2_IRQHandler
  .thumb_set EXTI2_IRQHandler, Default_Handler

  .weak      EXTI3_IRQHandler
  .thumb_set EXTI3_IRQHandler, Default_Handler

  .weak      EXTI4_IRQHandler
  .thumb_set EXTI4_IRQHandler, Default_Handler

  .weak      DMA1_Channel1_IRQHandler
  .thumb_set DMA1_Channel1_IRQHandler, Default_Handler

  .weak      DMA1_Channel2_IRQHandler
  .thumb_set DMA1_Channel2_IRQHandler, Default_Handler

  .weak      DMA1_Channel3_IRQHandler
  .thumb_set DMA1_Channel3_IRQHandler, Default_Handler

  .weak      DMA1_Channel4_IRQHandler
  .thumb_set DMA1_Channel4_IRQHandler, Default_Handler

  .weak      DMA1_Channel5_IRQHandler
  .thumb_set DMA1_Channel5_IRQHandler, Default_Handler

  .weak      DMA1_Channel6_IRQHandler
  .thumb_set DMA1_Channel6_IRQHandler, Default_Handler

  .weak      DMA1_Channel7_IRQHandler
  .thumb_set DMA1_Channel7_IRQHandler, Default_Handler

  .weak      ADC1_2_IRQHandler
  .thumb_set ADC1_2_IRQHandler, Default_Handler

  .weak      USB_HP_CAN1_TX_IRQHandler
  .thumb_set USB_HP_CAN1_TX_IRQHandler, Default_Handler

  .weak      USB_LP_CAN1_RX0_IRQHandler
  .thumb_set USB_LP_CAN1_RX0_IRQHandler, Default_Handler

  .weak      CAN1_RX1_IRQHandler
  .thumb_set CAN1_RX1_IRQHandler, Default_Handler

  .weak      CAN1_SCE_IRQHandler
  .thumb_set CAN1_SCE_IRQHandler, Default_Handler

  .weak      EXTI9_5_IRQHandler
  .thumb_set EXTI9_5_IRQHandler, Default_Handler

  .weak      TIM1_BRK_IRQHandler
  .thumb_set TIM1_BRK_IRQHandler, Default_Handler

  .weak      TIM1_UP_IRQHandler
  .thumb_set TIM1_UP_IRQHandler, Default_Handler

  .weak      TIM1_TRG_COM_IRQHandler
  .thumb_set TIM1_TRG_COM_IRQHandler, Default_Handler

  .weak      TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler, Default_Handler

  .weak      TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler, Default_Handler

  .weak      TIM3_IRQHandler
  .thumb_set TIM3_IRQHandler, Default_Handler

  .weak      TIM4_IRQHandler
  .thumb_set TIM4_IRQHandler, Default_Handler

  .weak      I2C1_EV_IRQHandler
  .thumb_set I2C1_EV_IRQHandler, Default_Handler

  .weak      I2C1_ER_IRQHandler
  .thumb_set I2C1_ER_IRQHandler, Default_Handler

  .weak      I2C2_EV_IRQHandler
  .thumb_set I2C2_EV_IRQHandler, Default_Handler

  .weak      I2C2_ER_IRQHandler
  .thumb_set I2C2_ER_IRQHandler, Default_Handler

  .weak      SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler, Default_Handler

  .weak      SPI2_IRQHandler
  .thumb_set SPI2_IRQHandler, Default_Handler

  .weak      USART1_IRQHandler
  .thumb_set USART1_IRQHandler, Default_Handler

  .weak      USART2_IRQHandler
  .thumb_set USART2_IRQHandler, Default_Handler

  .weak      USART3_IRQHandler
  .thumb_set USART3_IRQHandler, Default_Handler

  .weak      EXTI15_10_IRQHandler
  .thumb_set EXTI15_10_IRQHandler, Default_Handler

  .weak      RTCAlarm_IRQHandler
  .thumb_set RTCAlarm_IRQHandler, Default_Handler

  .weak      USBWakeUp_IRQHandler
  .thumb_set USBWakeUp_IRQHandler, Default_Handler


  .global Default_Handler
  .type   Default_Handler, %function
Default_Handler:
  b .
  .size Default_Handler, . - Default_Handler

/******************************************************************************/
