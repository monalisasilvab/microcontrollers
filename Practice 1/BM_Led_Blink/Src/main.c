#include "stm32f1xx.h"

int main(void){

	RCC->APB2ENR |= (1 << 4); //Enable GPIOC Clock
	RCC->APB2ENR |= (1 << 2); //Enable GPIOA Clock

	GPIOC->CRH &= ~((0xF << 20) | (0xF << 24) | (0xF << 28)); //Set C13, C14, C15 as Output
	GPIOC->CRH |= (0x2 << 20) | (0x2 << 24) | (0x2 << 28);

	GPIOA->CRL &= ~((0xF << 0) | (0xF << 4) | (0xF << 8)); //Set A0, A1, A2 as Input
	GPIOA->CRL |= (0X8 << 0) | (0x8 << 4) | (0x8 << 8);
	GPIOA->ODR |= (1 << 0) | (1 << 1) | (1 << 2);

	while(1){ //Blink LED in Blue-Pill C13, C14, C15 pin
		if ((GPIOA->IDR & (1 << 0)) == 0){
			GPIOC->ODR |= (1 << 13);
		}else{
			GPIOC->ODR &= ~(1 << 13);
		}

		if ((GPIOA->IDR & (1 << 1)) == 0){
			GPIOC->ODR |= (1 << 14);
		}else{
			GPIOC->ODR &= ~(1 << 14);
		}

		if ((GPIOA->IDR & (1 << 2)) == 0){
			GPIOC->ODR |= (1 << 15);
		}else{
			GPIOC->ODR &= ~(1 << 15);
		}
	}
}
