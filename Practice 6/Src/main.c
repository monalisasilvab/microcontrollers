#include "stm32f1xx.h"
#include <stdint.h>

uint16_t adc_data[3] = {0};

// Inicialização do ADC com DMA (usando PB0, PB1, PA4)
void ADC_DMA_Init(void) {
    // Clocks (GPIOA, GPIOB, ADC1, DMA1)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_ADC1EN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // PB0, PB1, PA4 como entradas analógicas
    GPIOB->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_CNF1 | GPIO_CRL_MODE0 | GPIO_CRL_MODE1);
    GPIOA->CRL &= ~(GPIO_CRL_CNF4 | GPIO_CRL_MODE4);

    // Configura DMA
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)adc_data;
    DMA1_Channel1->CNDTR = 3;
    DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_EN;

    // Configura ADC1
    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->CR2 = ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_ADON;
    ADC1->SQR1 = (2 << 20);
    ADC1->SQR3 = (8 << 0) | (9 << 5) | (4 << 10);

    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL);

    ADC1->CR2 |= ADC_CR2_ADON;
}

void PWM_Init(void) {
    // Clocks (GPIOA e TIM1)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_TIM1EN;

    // PA8, PA9, PA10 como saídas alternadas push-pull
    GPIOA->CRH &= ~(GPIO_CRH_CNF8 | GPIO_CRH_CNF9 | GPIO_CRH_CNF10);
    GPIOA->CRH |= (GPIO_CRH_CNF8_1 | GPIO_CRH_CNF9_1 | GPIO_CRH_CNF10_1);
    GPIOA->CRH |= (GPIO_CRH_MODE8_1 | GPIO_CRH_MODE9_1 | GPIO_CRH_MODE10_1);

    // Configura TIM1 para PWM em 3 canais
    TIM1->PSC = 72 - 1;
    TIM1->ARR = 4095;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    // Configuração dos canais PWM
    TIM1->CCMR1 |= (6 << 4) | TIM_CCMR1_OC1PE;
    TIM1->CCMR1 |= (6 << 12) | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 |= (6 << 4) | TIM_CCMR2_OC3PE;

    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->CR1 |= TIM_CR1_CEN;
}

int main(void) {
    ADC_DMA_Init();
    PWM_Init();

    while (1) {
        TIM1->CCR1 = adc_data[0]; // Vermelho (PB0)
        TIM1->CCR2 = adc_data[1]; // Verde (PB1)
        TIM1->CCR3 = adc_data[2]; // Azul (PA4)

        for(volatile int i = 0; i < 10000; i++);
    }
}

