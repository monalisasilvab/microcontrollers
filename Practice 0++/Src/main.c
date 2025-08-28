#include "stm32f1xx.h"

#define DELAY_TIME  20000

void delay(volatile int tempo) {
    while (tempo--);
}

void ConfigurarClockHSI_1MHz(void) {
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_HSI;

    RCC->CFGR |= RCC_CFGR_HPRE_DIV8;
    RCC->CFGR &= ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
}

void ConfigurarGPIO(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
    GPIOC->CRH |= GPIO_CRH_MODE13_1; // 2 MHz push-pull
}

int main(void) {
    ConfigurarClockHSI_1MHz();
    ConfigurarGPIO();

    while (1) {
        GPIOC->ODR ^= GPIO_ODR_ODR13;  // Alterna o estado do pino PC13
        delay(DELAY_TIME);
    }
}