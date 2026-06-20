/**
 * @file    bsp.c
 * @version 0.1.0
 * @authors Anton Chernov
 * @date    2026-06-19
 * @date    @showdate "%Y-%m-%d"
 */

/******************************** Included files ******************************/
#include "RTE_Components.h"
#include CMSIS_device_header
#include "bsp.h"
/********************************* Definitions ********************************/
#ifndef VECT_TAB_OFFSET
/**
 * @def VECT_TAB_OFFSET
 * @brief Vector Table base offset field. This value must be a multiple of 0x200.
 */
#define VECT_TAB_OFFSET         0x00000000UL
#endif /* VECT_TAB_OFFSET */

#define UART_ENABLED

/**
 * @def NVIC_PRIORITYGROUP_4
 * @brief 4 bits for pre-emption priority, 0 bits for subpriority.
 */
#define NVIC_PRIORITYGROUP_4    0x3U

#define SYSTEM_CLOCK_HZ         72000000U

/*
 * USART1 is on APB2 (72 MHz).
 * USARTDIV = FCLK / (16 × BAUD) = 72000000 / (16 × 230400) = 19.53125
 * DIV_Mantissa = 19, DIV_Fraction = round(0.53125 × 16) = 9
 * BRR = (19 << 4) | 9 = 313  →  actual baud ≈ 230032 (error ≈ 0.16 %)
 */
#define BAUD_RATE               230400U
#define BRR_VALUE               313U


/****************************** Module variables ******************************/

/**
 * @brief System tick counter incremented every 1 ms by SysTick_Handler.
 * @details Used as the time source for the AcroSched cooperative scheduler.
 */
volatile AcroTick_t sys_tick = 0U;

static unsigned int system_clock_hz __attribute__((section(".bss.noinit")));

/***************************** Private prototypes *****************************/

/** @brief Configures Flash wait states and prefetch. */
static void flash_config(void);

/**
 * @brief Configures RCC for 72 MHz SYSCLK from the HSE PLL.
 * @note  HSE 8 MHz → PLL ×9 = 72 MHz SYSCLK.
 *        HPRE=/1, PPRE1=/2, PPRE2=/1.
 */
static void rcc_config(void);

/** @brief Configures GPIO pins used by the BSP. */
static void gpio_config(void);

/**
 * @brief Configures SysTick for 1 kHz operation.
 * @param[in] ticks - reload value (core-clock ticks per period).
 */
static void systick_config(unsigned int ticks);

/** @brief Initializes the Data Watchpoint and Trace (DWT) unit. */
static void dwt_init(void);

#ifdef UART_ENABLED
/**
 * @brief Configures USART1 for 115200 8N1 on PA9/PA10.
 * @note  USART1 is clocked from APB2 at 24 MHz.
 */
static void uart_config(void);
#endif /* UART_ENABLED */

/********************* Application Programming Interface **********************/

/** @fn SystemInit */
void SystemInit(void) {
#ifdef DEBUG
    /* Keep debug connection alive during low-power modes and halt */
    DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP |
                  DBGMCU_CR_DBG_STOP  |
                  DBGMCU_CR_DBG_STANDBY;
#endif

    /* Disable interrupts during initialization */
    __disable_irq();

    /* Configure Flash latency (must be done before raising SYSCLK) */
    flash_config();

    /* Configure PLL and switch SYSCLK to 72 MHz */
    rcc_config();

    /* Configure GPIO ports */
    gpio_config();

#ifdef UART_ENABLED
    uart_config();
#endif /* UART_ENABLED */

#ifdef TEST_IRQ
    NVIC_SetPriority(TEST_IRQn, 10U);
    NVIC_EnableIRQ(TEST_IRQn);
#endif /* TEST_IRQ */

    /* Relocate vector table to Flash */
#ifdef VECT_TAB_SRAM
    SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET;
#else
    SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET;
#endif

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* Initialize DWT cycle counter */
    dwt_init();
}
/*----------------------------------------------------------------------------*/

/** @fn get_system_core_clock */
unsigned int get_system_core_clock(void) {
    return system_clock_hz;
}
/*----------------------------------------------------------------------------*/

/** @fn bspStart */
void bspStart(void) {
    systick_config(system_clock_hz / BSP_TICKS_PER_SEC);

    /* Enable the configurable fault exceptions (MemManage, BusFault,
     * UsageFault) so they trap to their own handlers in handlers.c instead
     * of escalating to HardFault. */
    SCB->SHCSR |= (SCB_SHCSR_MEMFAULTENA_Msk |
                   SCB_SHCSR_BUSFAULTENA_Msk |
                   SCB_SHCSR_USGFAULTENA_Msk);

    __enable_irq();
}
/*----------------------------------------------------------------------------*/

/** @fn bspGetTick */
AcroTick_t bspGetTick(void) {
    /* A 32-bit aligned read is atomic on Cortex-M3. */
    return sys_tick;
}
/*----------------------------------------------------------------------------*/

/** @fn SysTick_Handler */
void SysTick_Handler(void) {
    sys_tick++;
}
/*----------------------------------------------------------------------------*/

/** @fn reset_mcu */
void reset_mcu(void) {
    NVIC_SystemReset();
}

/****************************** Private functions *****************************/

/** @fn flash_config */
static void flash_config(void) {
    /* 2 wait states required for 48 MHz < SYSCLK ≤ 72 MHz */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_2;
    /* Enable prefetch buffer */
    FLASH->ACR |= FLASH_ACR_PRFTBE;
}
/*----------------------------------------------------------------------------*/

/** @fn rcc_config */
static void rcc_config(void) {
    /* Enable HSE (external 8 MHz crystal) and wait for it to stabilise */
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0U);

    /*
     * Configure PLL before enabling it:
     *   PLLSRC   = HSE    (bit 16 = 1)
     *   PLLXTPRE = HSE/1  (bit 17 = 0) → 8 MHz into PLL
     *   PLLMUL   = ×9     (bits [21:18] = 0111) → 8 MHz × 9 = 72 MHz
     *   HPRE     = /1     (AHB   = 72 MHz)
     *   PPRE1    = /2     (PCLK1 = 36 MHz, max allowed on F103)
     *   PPRE2    = /1     (PCLK2 = 72 MHz)
     */
    RCC->CFGR = RCC_CFGR_PLLSRC
              | RCC_CFGR_PLLMULL9
              | RCC_CFGR_PPRE1_DIV2;

    /* Enable PLL and wait for lock */
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U);

    /* Switch SYSCLK to PLL */
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    /* Enable peripheral clocks */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN    /* GPIOA — USART1 pins */
                  | RCC_APB2ENR_IOPCEN    /* GPIOC — user LED    */
                  | RCC_APB2ENR_USART1EN; /* USART1              */

    system_clock_hz = SYSTEM_CLOCK_HZ;
}
/*----------------------------------------------------------------------------*/

/** @fn gpio_config */
static void gpio_config(void) {
    /*
     * Blue Pill board (STM32F103C8T6):
     *   PC13 — on-board user LED, active LOW (LED on when PC13 = 0)
     *
     * In STM32F103 each pin occupies 4 bits in CRL/CRH. PC13 lives in CRH
     * at bits [23:20] ((13 - 8) × 4):
     *   MODE = 10 → output 2 MHz push-pull (≤ 2 MHz mandated for PC13)
     *   CNF  = 00 → general-purpose push-pull
     *   ⇒ value = 0b0010 = 0x2
     */
    GPIOC->CRH = (GPIOC->CRH & ~(0xFUL << 20U))
               | (0x2UL << 20U);   /* PC13 user LED */
    GPIOC->ODR |= (1UL << 13U);    /* LED OFF (active LOW: drive HIGH) */

#ifdef UART_ENABLED
    /*
     * PA9  (USART1_TX): AF push-pull, 50 MHz → CNF=10, MODE=11 → 0xB
     *                   bits [7:4] of GPIOA->CRH
     * PA10 (USART1_RX): floating input → CNF=01, MODE=00 → 0x4
     *                   bits [11:8] of GPIOA->CRH
     */
    GPIOA->CRH = (GPIOA->CRH & ~(0xFFUL << 4U))
               | (0xBUL << 4U)    /* PA9  TX: AF-PP 50 MHz */
               | (0x4UL << 8U);   /* PA10 RX: float input  */
#endif /* UART_ENABLED */
}
/*----------------------------------------------------------------------------*/

/** @fn systick_config */
static void systick_config(unsigned int ticks) {
    if ((ticks - 1U) > 0xFFFFFFU) {
        return;
    }

    SysTick->LOAD = (unsigned int)(ticks - 1U);
    NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk    |
                    SysTick_CTRL_ENABLE_Msk;
}
/*----------------------------------------------------------------------------*/

/** @fn dwt_init */
static void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT       = 0U;
}

#ifdef UART_ENABLED
/*----------------------------------------------------------------------------*/

/** @fn uart_config */
static void uart_config(void) {
    /*
     * BRR = 208 for 115200 baud at PCLK2 = 24 MHz
     * (DIV_Mantissa = 13, DIV_Fraction = 0)
     */
    USART1->BRR = BRR_VALUE;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}
#endif /* UART_ENABLED */

/********************* Application Programming Interface *********************/

/** @fn turn_on_led_green */
void turn_on_led_green(void) {
    GPIOC->ODR &= ~(1UL << 13U);   /* Active LOW: clear = LED ON */
}
/*----------------------------------------------------------------------------*/

/** @fn turn_off_led_green */
void turn_off_led_green(void) {
    GPIOC->ODR |= (1UL << 13U);    /* Active LOW: set = LED OFF */
}
/*----------------------------------------------------------------------------*/

/** @fn uartSendChar */
void uartSendChar(char c) {
#ifdef UART_ENABLED
    while ((USART1->SR & USART_SR_TXE) == 0U) { }
    USART1->DR = (uint8_t)c;
#else
    (void)c;
#endif /* UART_ENABLED */
}
/*----------------------------------------------------------------------------*/

/** @fn uartSendStr */
void uartSendStr(const char *str) {
    while (*str != '\0') {
        uartSendChar(*str);
        str++;
    }
}
/*----------------------------------------------------------------------------*/

/** @fn uartSendUint8 */
void uartSendUint8(uint8_t n) {
    if (n >= 10U) {
        uartSendChar((char)('0' + (n / 10U)));
    }
    uartSendChar((char)('0' + (n % 10U)));
}
/*----------------------------------------------------------------------------*/

/** @fn uartSendUint16 */
void uartSendUint16(uint16_t n) {
    uint16_t div_val;
    uint8_t  digit;
    uint8_t  ucStarted = 0U;

    for (div_val = 10000U; div_val >= 1U; div_val /= 10U) {
        digit = (uint8_t)(n / div_val);
        n     = (uint16_t)(n % div_val);
        if ((digit != 0U) || (ucStarted != 0U) || (div_val == 1U)) {
            uartSendChar((char)('0' + digit));
            ucStarted = 1U;
        }
    }
}
/******************************************************************************/
