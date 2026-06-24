/**
 * @file    bsp.c
 * @version 0.3.0
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

/*
 * Debug console on USART2 (APB1 = 36 MHz), 115200 8N1.
 * USARTDIV = 36000000 / (16 × 115200) = 19.53125
 * DIV_Mantissa = 19, DIV_Fraction = round(0.53125 × 16) = 9
 * BRR = (19 << 4) | 9 = 313  →  actual baud ≈ 115033 (error ≈ 0.16 %)
 */
#define DEBUG_BAUD_RATE         115200U
#define DEBUG_BRR_VALUE         313U

/*
 * I2C1 standard-mode 100 kHz timing (PCLK1 = 36 MHz).
 *   FREQ  = 36       (input clock in MHz)
 *   CCR   = 36M / (2 × 100k) = 180
 *   TRISE = FREQ + 1 = 37
 */
#define I2C1_FREQ_MHZ           36U
#define I2C1_CCR_STD            180U
#define I2C1_TRISE_STD          37U

/**
 * @def I2C1_TIMEOUT
 * @brief Poll iterations before an I2C bus operation is abandoned.
 */
#define I2C1_TIMEOUT            50000U

/**
 * @def MOTOR_PWM_RELOAD
 * @brief TIM auto-reload for the 20 kHz PWM carrier (72 MHz / 3600 = 20 kHz).
 */
#define MOTOR_PWM_RELOAD        3599U

/**
 * @def MOTOR_A_SD_PIN
 * @brief GPIOB pin of the Motor A IR2184 shutdown/enable line.
 */
#define MOTOR_A_SD_PIN          12U

/**
 * @def MOTOR_B_SD_PIN
 * @brief GPIOB pin of the Motor B IR2184 shutdown/enable line.
 */
#define MOTOR_B_SD_PIN          14U


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
 * @brief Configures USART1 (lidar link, 230400 8N1) and USART2 (debug
 *        console, 115200 8N1).
 * @note  USART1 is clocked from APB2 (72 MHz), USART2 from APB1 (36 MHz).
 */
static void uart_config(void);
#endif /* UART_ENABLED */

/**
 * @brief Waits for an I2C1_SR1 flag with a bounded timeout.
 * @param[in] flag - SR1 bit mask to wait for.
 * @returns Nonzero if the flag was observed; 0 on timeout.
 */
static uint8_t i2cWaitSR1(uint16_t flag);

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
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN    /* GPIOA — USART1/2 pins */
                  | RCC_APB2ENR_IOPCEN    /* GPIOC — user LED      */
                  | RCC_APB2ENR_USART1EN; /* USART1 — lidar link   */

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; /* USART2 — debug console */

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

    /*
     * PA2 (USART2_TX, debug): AF push-pull, 50 MHz → CNF=10, MODE=11 → 0xB
     *                         bits [11:8] of GPIOA->CRL
     */
    GPIOA->CRL = (GPIOA->CRL & ~(0xFUL << 8U))
               | (0xBUL << 8U);   /* PA2 TX: AF-PP 50 MHz */
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
    /* USART1 — lidar link, 230400 8N1 (receiver feeds the DMA). */
    USART1->BRR = BRR_VALUE;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    /* USART2 — debug console on PA2, 115200 8N1 (transmitter only). */
    USART2->BRR = DEBUG_BRR_VALUE;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;
}
#endif /* UART_ENABLED */

/*----------------------------------------------------------------------------*/

/** @fn i2cWaitSR1 */
static uint8_t i2cWaitSR1(uint16_t flag) {
    uint8_t  ret_val   = 0U;
    uint32_t ulTimeout = I2C1_TIMEOUT;

    while (ulTimeout != 0U) {
        if ((I2C1->SR1 & flag) != 0U) {
            ret_val   = 1U;
            ulTimeout = 0U;     /* flag observed → leave the loop */
        }
        else {
            ulTimeout--;
        }
    }
    return ret_val;
}

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
    while ((USART2->SR & USART_SR_TXE) == 0U) { }
    USART2->DR = (uint8_t)c;
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
    if (n >= 100U) {
        uartSendChar((char)('0' + (n / 100U)));
    }
    if (n >= 10U) {
        uartSendChar((char)('0' + ((n / 10U) % 10U)));
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
/*----------------------------------------------------------------------------*/

/** @fn uartSendHex8 */
void uartSendHex8(uint8_t n) {
    static const char hex[] = "0123456789ABCDEF";

    uartSendChar(hex[(n >> 4U) & 0x0FU]);
    uartSendChar(hex[n & 0x0FU]);
}
/*----------------------------------------------------------------------------*/

/** @fn bspLidarRxDmaInit */
void bspLidarRxDmaInit(uint8_t *pBuf, uint16_t len) {
    /* Enable DMA1 controller clock */
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    /* USART1_RX is mapped to DMA1 Channel 5 on STM32F103 */
    DMA1_Channel5->CCR   = 0U;                  /* disable while configuring */
    DMA1_Channel5->CPAR  = (uint32_t)(&USART1->DR);
    DMA1_Channel5->CMAR  = (uint32_t)pBuf;
    DMA1_Channel5->CNDTR = len;
    DMA1_Channel5->CCR   = DMA_CCR1_MINC        /* memory increment */
                         | DMA_CCR1_CIRC        /* circular buffer  */
                         | DMA_CCR1_PL_0        /* medium priority  */
                         | DMA_CCR1_EN;         /* enable channel   */

    /* Route the USART1 receiver to DMA */
    USART1->CR3 |= USART_CR3_DMAR;
}
/*----------------------------------------------------------------------------*/

/** @fn bspLidarRxDmaIndex */
uint16_t bspLidarRxDmaIndex(uint16_t len) {
    return (uint16_t)(len - (uint16_t)DMA1_Channel5->CNDTR);
}
/*----------------------------------------------------------------------------*/

/** @fn bspI2c1Init */
void bspI2c1Init(void) {
    /* Enable GPIOB and I2C1 clocks */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /*
     * PB6 (I2C1_SCL) and PB7 (I2C1_SDA): alternate-function open-drain,
     * 50 MHz → CNF=11, MODE=11 → 0xF. PB6 = CRL[27:24], PB7 = CRL[31:28].
     */
    GPIOB->CRL = (GPIOB->CRL & ~((0xFUL << 24U) | (0xFUL << 28U)))
               | (0xFUL << 24U)    /* PB6 SCL */
               | (0xFUL << 28U);   /* PB7 SDA */

    /* Software-reset the peripheral, then program standard mode 100 kHz */
    I2C1->CR1   = I2C_CR1_SWRST;
    I2C1->CR1   = 0U;
    I2C1->CR2   = I2C1_FREQ_MHZ;
    I2C1->CCR   = I2C1_CCR_STD;
    I2C1->TRISE = I2C1_TRISE_STD;
    I2C1->CR1   = I2C_CR1_PE;
}
/*----------------------------------------------------------------------------*/

/** @fn bspI2c1WriteReg */
uint8_t bspI2c1WriteReg(uint8_t addr7, uint8_t reg, uint8_t val) {
    uint8_t ucOk = 1U;

    I2C1->CR1 |= I2C_CR1_START;
    ucOk &= i2cWaitSR1(I2C_SR1_SB);

    I2C1->DR = (uint16_t)(addr7 << 1U);        /* address + write */
    ucOk &= i2cWaitSR1(I2C_SR1_ADDR);
    (void)I2C1->SR1;
    (void)I2C1->SR2;                           /* clear ADDR */

    ucOk &= i2cWaitSR1(I2C_SR1_TXE);
    I2C1->DR = (uint16_t)reg;

    ucOk &= i2cWaitSR1(I2C_SR1_TXE);
    I2C1->DR = (uint16_t)val;

    ucOk &= i2cWaitSR1(I2C_SR1_BTF);
    I2C1->CR1 |= I2C_CR1_STOP;

    return ucOk;
}
/*----------------------------------------------------------------------------*/

/** @fn bspI2c1ReadRegs */
uint8_t bspI2c1ReadRegs(uint8_t addr7, uint8_t reg, uint8_t *pBuf, uint8_t len) {
    uint8_t ucOk = 1U;
    uint8_t i    = 0U;

    /* Phase 1 — point the device at the starting register */
    I2C1->CR1 |= I2C_CR1_START;
    ucOk &= i2cWaitSR1(I2C_SR1_SB);
    I2C1->DR = (uint16_t)(addr7 << 1U);        /* address + write */
    ucOk &= i2cWaitSR1(I2C_SR1_ADDR);
    (void)I2C1->SR1;
    (void)I2C1->SR2;                           /* clear ADDR */
    ucOk &= i2cWaitSR1(I2C_SR1_TXE);
    I2C1->DR = (uint16_t)reg;
    ucOk &= i2cWaitSR1(I2C_SR1_BTF);

    /* Phase 2 — repeated start, switch to receiver, read len bytes */
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_START;
    ucOk &= i2cWaitSR1(I2C_SR1_SB);
    I2C1->DR = (uint16_t)((addr7 << 1U) | 1U); /* address + read */
    ucOk &= i2cWaitSR1(I2C_SR1_ADDR);
    (void)I2C1->SR1;
    (void)I2C1->SR2;                           /* clear ADDR */

    /* Read all but the last three bytes with ACK enabled */
    while ((uint8_t)(len - i) > 3U) {
        ucOk &= i2cWaitSR1(I2C_SR1_RXNE);
        pBuf[i] = (uint8_t)I2C1->DR;
        i++;
    }

    /* Closing sequence for the final three bytes (RM0008 N > 2 path) */
    ucOk &= i2cWaitSR1(I2C_SR1_BTF);
    I2C1->CR1 &= (uint16_t)~I2C_CR1_ACK;
    pBuf[i] = (uint8_t)I2C1->DR;               /* data N-2 */
    i++;

    ucOk &= i2cWaitSR1(I2C_SR1_BTF);
    I2C1->CR1 |= I2C_CR1_STOP;
    pBuf[i] = (uint8_t)I2C1->DR;               /* data N-1 */
    i++;

    ucOk &= i2cWaitSR1(I2C_SR1_RXNE);
    pBuf[i] = (uint8_t)I2C1->DR;               /* data N   */

    return ucOk;
}
/*----------------------------------------------------------------------------*/

/** @fn bspMotorInit */
void bspMotorInit(void) {
    /* Clock the motor peripherals (GPIOA already enabled by rcc_config). */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN     /* GPIOA — Motor A PWM pins  */
                  | RCC_APB2ENR_IOPBEN     /* GPIOB — Motor B + SD pins */
                  | RCC_APB2ENR_TIM1EN;    /* TIM1  — Motor A PWM        */
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;    /* TIM4  — Motor B PWM        */

    /*
     * Motor A PWM: PA8 (TIM1_CH1) and PA11 (TIM1_CH4), AF push-pull 50 MHz
     * (0xB). PA8 = CRH[3:0], PA11 = CRH[15:12].
     */
    GPIOA->CRH = (GPIOA->CRH & ~((0xFUL << 0U) | (0xFUL << 12U)))
               | (0xBUL << 0U)     /* PA8  AF-PP */
               | (0xBUL << 12U);   /* PA11 AF-PP */

    /*
     * Motor B PWM: PB8 (TIM4_CH3) and PB9 (TIM4_CH4), AF push-pull 50 MHz
     * (0xB). SD/EN lines PB12 / PB14, general-purpose push-pull 2 MHz (0x2).
     * CRH offsets: PB8 [3:0], PB9 [7:4], PB12 [19:16], PB14 [27:24].
     */
    GPIOB->CRH = (GPIOB->CRH & ~((0xFUL << 0U)  | (0xFUL << 4U)
                               | (0xFUL << 16U) | (0xFUL << 24U)))
               | (0xBUL << 0U)     /* PB8  AF-PP  */
               | (0xBUL << 4U)     /* PB9  AF-PP  */
               | (0x2UL << 16U)    /* PB12 out PP */
               | (0x2UL << 24U);   /* PB14 out PP */

    /* SD/EN default OFF (disabled): drive both lines low before arming. */
    GPIOB->BRR = (1UL << MOTOR_A_SD_PIN) | (1UL << MOTOR_B_SD_PIN);

    /* TIM1 (Motor A): CH1 forward leg, CH4 reverse leg, 20 kHz PWM mode 1. */
    TIM1->PSC   = 0U;
    TIM1->ARR   = MOTOR_PWM_RELOAD;
    TIM1->CCR1  = 0U;
    TIM1->CCR4  = 0U;
    TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
    TIM1->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC4E;
    TIM1->BDTR  = TIM_BDTR_MOE;            /* advanced timer: main output en */
    TIM1->CR1   = TIM_CR1_ARPE;
    TIM1->EGR   = TIM_EGR_UG;              /* load preload registers */
    TIM1->CR1  |= TIM_CR1_CEN;

    /* TIM4 (Motor B): CH3 forward leg, CH4 reverse leg, 20 kHz PWM mode 1. */
    TIM4->PSC   = 0U;
    TIM4->ARR   = MOTOR_PWM_RELOAD;
    TIM4->CCR3  = 0U;
    TIM4->CCR4  = 0U;
    TIM4->CCMR2 = TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE
                | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;
    TIM4->CCER  = TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM4->CR1   = TIM_CR1_ARPE;
    TIM4->EGR   = TIM_EGR_UG;
    TIM4->CR1  |= TIM_CR1_CEN;
}
/*----------------------------------------------------------------------------*/

/** @fn bspMotorSetDutyA */
void bspMotorSetDutyA(uint16_t fwd, uint16_t rev) {
    TIM1->CCR1 = fwd;       /* forward leg (PA8)  */
    TIM1->CCR4 = rev;       /* reverse leg (PA11) */
}
/*----------------------------------------------------------------------------*/

/** @fn bspMotorSetDutyB */
void bspMotorSetDutyB(uint16_t fwd, uint16_t rev) {
    TIM4->CCR3 = fwd;       /* forward leg (PB8) */
    TIM4->CCR4 = rev;       /* reverse leg (PB9) */
}
/*----------------------------------------------------------------------------*/

/** @fn bspMotorEnable */
void bspMotorEnable(uint8_t enable) {
    if (enable != 0U) {
        /* SD high arms both IR2184 bridges. */
        GPIOB->BSRR = (1UL << MOTOR_A_SD_PIN) | (1UL << MOTOR_B_SD_PIN);
    }
    else {
        /* SD low forces both bridges into shutdown (coast). */
        GPIOB->BRR = (1UL << MOTOR_A_SD_PIN) | (1UL << MOTOR_B_SD_PIN);
    }
}
/******************************************************************************/
