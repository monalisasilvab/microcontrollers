#include <stdint.h>
#include "stm32f1xx.h"

void delay_ms(volatile uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 8000; i++);
}

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {
        delay_ms(20);
        if ((GPIOA->IDR & (1 << 0)) == 0) {
            GPIOC->ODR ^= (1 << 13);
        }
        EXTI->PR = EXTI_PR_PR0;
    }
}

void EXTI1_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR1) {
        delay_ms(20);
        if ((GPIOA->IDR & (1 << 1)) == 0) {
            GPIOC->ODR ^= (1 << 14);
        }
        EXTI->PR = EXTI_PR_PR1;
    }
}

void EXTI2_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR2) {
        delay_ms(20);
        if ((GPIOA->IDR & (1 << 2)) == 0) {
            GPIOC->ODR ^= (1 << 15);
        }
        EXTI->PR = EXTI_PR_PR2;
    }
}

int main(void)
{
    // Inicializações básicas
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;

    // Configura os pinos PC13, PC14, PC15 como saída push-pull
    GPIOC->CRH &= ~((0xF << 20) | (0xF << 24) | (0xF << 28));
    GPIOC->CRH |= ((0x2 << 20) | (0x2 << 24) | (0x2 << 28)); 

    // Configura PA0, PA1, PA2 como entrada pull-up
    GPIOA->CRL &= ~((0xF << 0) | (0xF << 4) | (0xF << 8));
    GPIOA->CRL |= ((0x8 << 0) | (0x8 << 4) | (0x8 << 8)); 
    GPIOA->ODR |= (1 << 0) | (1 << 1) | (1 << 2); 

    // Conecta EXTI0, EXTI1, EXTI2 aos pinos PA0, PA1, PA2
    AFIO->EXTICR[0] &= ~((0xF << 0) | (0xF << 4) | (0xF << 8)); 

    // Configura EXTI0, EXTI1, EXTI2 como interrupção de borda de descida
    EXTI->IMR |= (1 << 0) | (1 << 1) | (1 << 2);
    EXTI->FTSR |= (1 << 0) | (1 << 1) | (1 << 2);

    // Habilita interrupções no NVIC
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);

    while (1) {
        // loop principal vazio; ações via interrupções
    }
}