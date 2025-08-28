#include "stm32f1xx.h"
#include <stdlib.h>
#include <stdio.h>

void SystemClock_Config(void);
void USART1_Init(void);
void USART1_SendChar(char c);
char USART1_ReadChar(void);
void GPIO_PWM_Init(void);
void TIM3_PWM_Init(void);
void PWM_SetDuty(uint8_t r, uint8_t g, uint8_t b);
int  __io_putchar(int ch);

static char    rx_buffer[16];
static uint8_t rx_index = 0;

int main(void)
{
    SystemClock_Config();
    USART1_Init();
    GPIO_PWM_Init();
    TIM3_PWM_Init();

    printf("Pronto! Comandos: R<0‑100>, G<0‑100>, B<0‑100> + <Enter>\n");

    uint8_t r = 0, g = 0, b = 0;

    while (1) {
        char c = USART1_ReadChar();

        if (c == '\r' || c == '\n') {
            if (rx_index == 0) continue;

            rx_buffer[rx_index] = '\0';
            rx_index = 0;

            char cor   = rx_buffer[0];
            int  valor = atoi(&rx_buffer[1]);

            if (valor < 0)   valor = 0;
            if (valor > 100) valor = 100;

            switch (cor) {
                case 'r': case 'R': r = (uint8_t)valor; break;
                case 'g': case 'G': g = (uint8_t)valor; break;
                case 'b': case 'B': b = (uint8_t)valor; break;
                default:
                    printf("Comando desconhecido: %s\n", rx_buffer);
                    continue;
            }

            PWM_SetDuty(r, g, b);
            printf("PWM => R:%u G:%u B:%u\n", r, g, b);
        }
        else if (rx_index < sizeof(rx_buffer) - 1) {
            rx_buffer[rx_index++] = c;
        }
    }
}

void SystemClock_Config(void)
{
    RCC->CR   |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;
    RCC->CR   |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void USART1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

    GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
    GPIOA->CRH |=  (GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0) | GPIO_CRH_CNF9_1;

    GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
    GPIOA->CRH |=  GPIO_CRH_CNF10_0;

    USART1->BRR = 72000000UL / 9600UL;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

char USART1_ReadChar(void)
{
    while (!(USART1->SR & USART_SR_RXNE));
    return (char)USART1->DR;
}

void USART1_SendChar(char c)
{
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = (uint16_t)c;
}

int __io_putchar(int ch)
{
    USART1_SendChar((char)ch);
    return ch;
}

void GPIO_PWM_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;

    GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
    GPIOA->CRL |=  (GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0) | GPIO_CRL_CNF6_1;

    GPIOA->CRL &= ~(GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
    GPIOA->CRL |=  (GPIO_CRL_MODE7_1 | GPIO_CRL_MODE7_0) | GPIO_CRL_CNF7_1;

    GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
    GPIOB->CRL |=  (GPIO_CRL_MODE0_1 | GPIO_CRL_MODE0_0) | GPIO_CRL_CNF0_1;
}

void TIM3_PWM_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 71;
    TIM3->ARR = 100;

    TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE);
    TIM3->CCMR1 |=  (6 << 4) | TIM_CCMR1_OC1PE;

    TIM3->CCMR1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE);
    TIM3->CCMR1 |=  (6 << 12) | TIM_CCMR1_OC2PE;

    TIM3->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC3PE);
    TIM3->CCMR2 |=  (6 << 4) | TIM_CCMR2_OC3PE;

    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM3->CR1  |= TIM_CR1_ARPE;
    TIM3->EGR  |= TIM_EGR_UG;
    TIM3->CR1  |= TIM_CR1_CEN;
}

void PWM_SetDuty(uint8_t r, uint8_t g, uint8_t b)
{
    TIM3->CCR1 = r;
    TIM3->CCR2 = g;
    TIM3->CCR3 = b;
}
