#include <stdint.h>

#if !defined(_SOFT_FP) && defined(_ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include "stm32f1xx.h"

volatile int duty_cycle = 0;
volatile int increment = 1;
volatile int color_state = 0;

void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF)
    {
        // Incrementar ou decrementar o duty cycle
        duty_cycle += increment;

        if (duty_cycle >= 100 || duty_cycle <= 0)
        {
            increment = -increment;

            if (increment > 0)
            {
                color_state = (color_state + 1) % 7; // Próxima cor
            }
        }

        // Ajustar o duty cycle de acordo com a cor atual
        switch (color_state)
        {
            case 0:  // Vermelho
                TIM2->CCR1 = duty_cycle;
                TIM2->CCR2 = 0;
                TIM2->CCR3 = 0;
                break;
            case 1:  // Verde
                TIM2->CCR1 = 0;
                TIM2->CCR2 = duty_cycle;
                TIM2->CCR3 = 0;
                break;
            case 2:  // Azul
                TIM2->CCR1 = 0;
                TIM2->CCR2 = 0;
                TIM2->CCR3 = duty_cycle;
                break;
            case 3:  // Amarelo (Vermelho + Verde)
                TIM2->CCR1 = duty_cycle;
                TIM2->CCR2 = duty_cycle;
                TIM2->CCR3 = 0;
                break;
            case 4:  // Ciano (Verde + Azul)
                TIM2->CCR1 = 0;
                TIM2->CCR2 = duty_cycle;
                TIM2->CCR3 = duty_cycle;
                break;
            case 5:  // Roxo (Vermelho + Azul)
                TIM2->CCR1 = duty_cycle;
                TIM2->CCR2 = 0;
                TIM2->CCR3 = duty_cycle;
                break;
            case 6:  // Branco (Vermelho + Verde + Azul)
                TIM2->CCR1 = duty_cycle;
                TIM2->CCR2 = duty_cycle;
                TIM2->CCR3 = duty_cycle;
                break;
        }

        TIM3->SR &= ~TIM_SR_UIF; // Limpar a flag de interrupção
    }
}

int main(void)
{
    RCC->APB2ENR |= (1 << 2);

    GPIOA->CRL &= ~(0xFFF);
    GPIOA->CRL |=  (0xBBB);

    RCC->APB2ENR |= (1 << 0);

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = 799;   
    TIM2->ARR = 99;  

    TIM2->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM2->CCER |= TIM_CCER_CC1E;

    TIM2->CCMR1 |= (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM2->CCER |= TIM_CCER_CC2E;

    TIM2->CCMR2 |= (6 << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM2->CCER |= TIM_CCER_CC3E;

    TIM2->CR1 |= TIM_CR1_CEN;

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 1999;   
    TIM3->ARR = 99;    

    TIM3->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM3_IRQn);

    TIM3->CR1 |= TIM_CR1_CEN;

    while(1)
    {

    }
}