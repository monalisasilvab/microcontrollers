#include "main.h"
 
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
 
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
 
int main(void)
{
    uint16_t AD_RES = 0, Vamb, DC_Multiplier;
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
   
    HAL_ADCEx_Calibration_Start(&hadc1);
 
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1);
    Vamb = HAL_ADC_GetValue(&hadc1);
    DC_Multiplier = 65535/(4096-Vamb);
 
    while (1)
    {
        HAL_ADC_Start(&hadc1);
       
        HAL_ADC_PollForConversion(&hadc1, 1);
       
        AD_RES = HAL_ADC_GetValue(&hadc1);
        TIM2->CCR1 = (AD_RES-Vamb)*DC_Multiplier;
        HAL_Delay(1);
    }
}