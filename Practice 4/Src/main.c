#include "stm32f1xx.h"
#include <stdint.h>

typedef enum{
	BLACK  = 0,
	RED    = 1,
	GREEN  = 2,
	BLUE   = 3,
	YELLOW = 4,
	CYAN   = 5,
	PURPLE = 6,
	WHITE  = 7
}RGB;

void setup(void);
void tim_init(void);
void toggle_RGB(RGB);

unsigned int estadoLed = 1;
unsigned int cor = 0;

void TIM2_IRQHandler(void)
{
	if(TIM2->SR & TIM_SR_UIF)
	{
		TIM2->SR &=~TIM_SR_UIF;

		if(estadoLed){
			toggle_RGB(cor);
			estadoLed = 0;
		}
		else{
				toggle_RGB(0);
				estadoLed = 1;
		}
	}
}

void TIM3_IRQHandler(void){
	if(TIM3->SR & TIM_SR_UIF)
		{
			TIM3->SR &=~TIM_SR_UIF;
			if(((cor+1) % 8) == 0){
				cor = 1;
			}
			else{ cor = (cor+1) % 8; }
		}
}

#if !defined(_SOFT_FP) && defined(_ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void)
{
    setup();
    tim_init();
	for(;;);
}

void setup(void){
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;

	GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 | GPIO_CRL_MODE1 | GPIO_CRL_CNF1 | GPIO_CRL_MODE2 | GPIO_CRL_CNF2);
	GPIOA->CRL |= (GPIO_CRL_MODE0_1 | GPIO_CRL_MODE1_1 | GPIO_CRL_MODE2_1);

}

void tim_init(void){
    // TIM2: Pisca o LED 2 vezes por segundo (a cada 0.25s alterna o estado)
    TIM2->PSC = 8000 - 1;     // Clock base: 8 MHz / 8000 = 1 kHz
    TIM2->ARR = 250 - 1;      // 250 ms (0.25 s)
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;

    // TIM3: Muda a cor a cada 1 segundo (ou mantenha como preferir)
    TIM3->PSC = 8000 - 1;
    TIM3->ARR = 1000 - 1;     // 1000 ms = 1 s
    TIM3->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 |= TIM_CR1_CEN;

    NVIC_SetPriority(TIM2_IRQn, 1);
}

void toggle_RGB(RGB color){
	switch(color){
		case BLACK:
			GPIOA->ODR &= ~(GPIO_ODR_ODR0 | GPIO_ODR_ODR1 | GPIO_ODR_ODR2);
			break;
		case RED:
			GPIOA->ODR |= GPIO_ODR_ODR0;
			break;
		case GREEN:
			GPIOA->ODR |= GPIO_ODR_ODR1;
			break;
		case BLUE:
			GPIOA->ODR |= GPIO_ODR_ODR0 | GPIO_ODR_ODR1;
			break;
		case YELLOW:
			GPIOA->ODR |= GPIO_ODR_ODR2;
			break;
		case CYAN:
			GPIOA->ODR |= GPIO_ODR_ODR2 | GPIO_ODR_ODR0;
			break;
		case PURPLE:
			GPIOA->ODR |= GPIO_ODR_ODR2 | GPIO_ODR_ODR1;
			break;
		case WHITE:
			GPIOA->ODR |= GPIO_ODR_ODR2 | GPIO_ODR_ODR1 | GPIO_ODR_ODR0;
			break;
		default:
			break;
	}
}
