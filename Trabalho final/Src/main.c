#include "stm32f1xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// === Definições do LDR ===
#define LED_LDR_PIN     (1 << 2)  // PA2 - Indica escuridão (LDR)
#define LDR_PIN         (1 << 0)  // PA0 - Canal ADC1_IN0

// === Definições do LED RGB (Temperatura) ===
#define LED_TEMP_R_PIN  (1 << 8)   // PB8
#define LED_TEMP_G_PIN  (1 << 9)   // PB9
#define LED_TEMP_B_PIN  (1 << 10)  // PB10

// === Definições do LED RGB (Status NRF) ===
#define LED_NRF_R_PIN   (1 << 12)  // PB12
#define LED_NRF_G_PIN   (1 << 13)  // PB13
#define LED_NRF_B_PIN   (1 << 14)  // PB14

// === Endereço do AHT10 ===
#define AHT10_ADDR (0x38 << 1)

// === Limiares ===
#define LIGHT_THRESHOLD     1000
#define TEMP_THRESHOLD_C    28.0f

// === NRF24L01 DEFINIÇÕES ===
#define NRF24L01_CSN_LOW()   (GPIOA->ODR &= ~(1 << 4))  // CSN = PA4
#define NRF24L01_CSN_HIGH()  (GPIOA->ODR |=  (1 << 4))

#define NRF24L01_CE_LOW()    (GPIOA->ODR &= ~(1 << 3))  // CE = PA3
#define NRF24L01_CE_HIGH()   (GPIOA->ODR |=  (1 << 3))

#define NRF_CHANNEL      10
#define NRF_ADDRESS_LEN  5
#define NRF_DATA_RATE    0x08  // 2Mbps
#define NRF_CRC_ENABLE   0x08  // CRC ON (2 bytes)
#define NRF_AUTO_ACK     0x3F  // Auto-ack em todos os pipes

// === Prototipagem ===
void ms_delay(uint32_t ms);
void porta_gpio_inicializa(void);
void adc1_inicializa(void);
uint16_t ler_ldr(void);

void i2c1_inicializa(void);
uint8_t aht10_configura(void);
void aht10_dispara_medicao(void);
void aht10_le_dados(uint8_t *dados);
float temperatura_dados(uint8_t *dados);
float temperatura_ler(void);
void config_relogio_sistema(void);

void spi1_inicializa(void);
uint8_t spi1_transmite_recebe(uint8_t dado);
void nrf24_escreve_registro(uint8_t reg, uint8_t valor);
void nrf24_escreve_registro_multi(uint8_t reg, uint8_t *dados, uint8_t tamanho);
void nrf24_inicializa_tx(uint8_t *endereco);
bool nrf24_envia_dados(uint8_t *dados, uint8_t tamanho);

void usart1_inicializa(void);
void led_rgb_status_envio(bool sucesso);

// === MAIN ===
int main(void) {
    // Clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_ADC1EN;

    // Inicializações
    porta_gpio_inicializa();
    adc1_inicializa();
    i2c1_inicializa();
    ms_delay(50);
    aht10_configura();
    spi1_inicializa();
    config_relogio_sistema();

    uint8_t endereco_tx[5] = {0x04, 0xDD, 0xCC, 0xBB, 0xAA};
    nrf24_inicializa_tx(endereco_tx);

    while (1) {
        // === LDR ===
        uint16_t valor_ldr = ler_ldr();
        if (valor_ldr > LIGHT_THRESHOLD) {
            GPIOA->ODR &= ~LED_LDR_PIN; // Escuro: acende LED LDR
        } else {
            GPIOA->ODR |= LED_LDR_PIN;  // Claro: apaga LED LDR
        }

        // === Temperatura (AHT10) ===
        float temp = temperatura_ler();

        // Apaga LEDs do RGB temperatura
        GPIOB->ODR &= ~(LED_TEMP_R_PIN | LED_TEMP_G_PIN | LED_TEMP_B_PIN);

        if (temp >= 38.0f) {
            GPIOB->ODR |= LED_TEMP_R_PIN;
        } else if (temp <= 29.0f) {
            GPIOB->ODR |= LED_TEMP_B_PIN;
        } else {
            GPIOB->ODR |= LED_TEMP_G_PIN;
        }

        ms_delay(100);

        uint8_t payload[32];
        snprintf((char *)payload, sizeof(payload), "Temp: %.1fC | LDR: %u", temp, valor_ldr);

        // Envia dados via NRF24L01 e indica status via LED RGB extra
        bool envio_sucesso = nrf24_envia_dados(payload, 32);
        led_rgb_status_envio(envio_sucesso);

        ms_delay(1000);
    }
}

// === Implementações ===
// Configura os pinos como entrada e saída
void porta_gpio_inicializa(void) {
    // PA1 e PA2 saída push-pull (LED LDR)
    GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1 |
                    GPIO_CRL_MODE2 | GPIO_CRL_CNF2);
    GPIOA->CRL |= (GPIO_CRL_MODE1_0 | GPIO_CRL_MODE2_0);

    // PA0 entrada analógica (LDR)
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);

    // PB8, PB9, PB10 saída push-pull (LED RGB Temperatura)
    GPIOB->CRH &= ~(GPIO_CRH_MODE8_Msk | GPIO_CRH_CNF8_Msk |
                    GPIO_CRH_MODE9_Msk | GPIO_CRH_CNF9_Msk |
                    GPIO_CRH_MODE10_Msk | GPIO_CRH_CNF10_Msk);
    GPIOB->CRH |= (0b10 << GPIO_CRH_MODE8_Pos) | (0b00 << GPIO_CRH_CNF8_Pos) |
                  (0b10 << GPIO_CRH_MODE9_Pos) | (0b00 << GPIO_CRH_CNF9_Pos) |
                  (0b10 << GPIO_CRH_MODE10_Pos) | (0b00 << GPIO_CRH_CNF10_Pos);

    // PB12, PB13, PB14 saída push-pull (LED RGB Status NRF)
    GPIOB->CRH &= ~(GPIO_CRH_MODE12_Msk | GPIO_CRH_CNF12_Msk |
                    GPIO_CRH_MODE13_Msk | GPIO_CRH_CNF13_Msk |
                    GPIO_CRH_MODE14_Msk | GPIO_CRH_CNF14_Msk);
    GPIOB->CRH |= (0b10 << GPIO_CRH_MODE12_Pos) | (0b00 << GPIO_CRH_CNF12_Pos) |
                  (0b10 << GPIO_CRH_MODE13_Pos) | (0b00 << GPIO_CRH_CNF13_Pos) |
                  (0b10 << GPIO_CRH_MODE14_Pos) | (0b00 << GPIO_CRH_CNF14_Pos);

    // Apaga todos os LEDs inicialmente
    GPIOB->ODR &= ~(LED_TEMP_R_PIN | LED_TEMP_G_PIN | LED_TEMP_B_PIN | LED_NRF_R_PIN | LED_NRF_G_PIN | LED_NRF_B_PIN);
    GPIOA->ODR &= ~(LED_LDR_PIN);
}

// Inicializa o ADC1 para ler o LDR
void adc1_inicializa(void) {
    ADC1->SMPR2 |= ADC_SMPR2_SMP0; // Canal 0 - amostragem longa
    ADC1->CR2 |= ADC_CR2_ADON;
    ms_delay(1);
    ADC1->CR2 |= ADC_CR2_ADON;
    ms_delay(1);
}

// Lê o valor do LDR
uint16_t ler_ldr(void) {
    ADC1->SQR3 = 0;
    ADC1->CR2 |= ADC_CR2_ADON;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

// Função de delay em milissegundos
// Utiliza o SysTick configurado para 72MHz
void ms_delay(uint32_t ms) {
    SysTick->LOAD = 72000 - 1; // 1ms @ 72MHz
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
    for (uint32_t i = 0; i < ms; i++) {
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    }
    SysTick->CTRL = 0;
}

// Inicializa o I2C1 para comunicação com o AHT10
void i2c1_inicializa(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB6/PB7 alternate open-drain
    GPIOB->CRL &= ~(GPIO_CRL_MODE6_Msk | GPIO_CRL_CNF6_Msk |
                    GPIO_CRL_MODE7_Msk | GPIO_CRL_CNF7_Msk);
    GPIOB->CRL |= (0b11 << GPIO_CRL_MODE6_Pos) | (0b11 << GPIO_CRL_CNF6_Pos) |
                  (0b11 << GPIO_CRL_MODE7_Pos) | (0b11 << GPIO_CRL_CNF7_Pos);

    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->CR2 = 36;
    I2C1->CCR = 180;
    I2C1->TRISE = 37;
    I2C1->CR1 |= I2C_CR1_PE;
}

// Configura o AHT10
// Retorna 1 se a configuração foi bem-sucedida
// Retorna 0 se falhou (não implementado, mas pode ser adicionado)
uint8_t aht10_configura(void) {
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = AHT10_ADDR;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    I2C1->DR = 0xE1;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
    I2C1->CR1 |= I2C_CR1_STOP;
    ms_delay(10);
    return 1;
}

// Dispara medição no AHT10
void aht10_dispara_medicao(void) {
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = AHT10_ADDR;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    I2C1->DR = 0xAC; while (!(I2C1->SR1 & I2C_SR1_BTF));
    I2C1->DR = 0x33; while (!(I2C1->SR1 & I2C_SR1_BTF));
    I2C1->DR = 0x00; while (!(I2C1->SR1 & I2C_SR1_BTF));
    I2C1->CR1 |= I2C_CR1_STOP;
}

// Lê os dados do AHT10
void aht10_le_dados(uint8_t *dados) {
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = AHT10_ADDR | 1;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    for (int i = 0; i < 5; i++) {
        while (!(I2C1->SR1 & I2C_SR1_RXNE));
        dados[i] = I2C1->DR;
    }

    I2C1->CR1 &= ~I2C_CR1_ACK;
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    dados[5] = I2C1->DR;
    I2C1->CR1 |= I2C_CR1_STOP;
    I2C1->CR1 |= I2C_CR1_ACK;
}

// Converte os dados lidos do AHT10 em temperatura
// Retorna a temperatura em graus Celsius
float temperatura_dados(uint8_t *dados) {
    uint32_t temp_bruto = ((dados[3] & 0x0F) << 16) | (dados[4] << 8) | dados[5];
    return ((temp_bruto * 200.0f) / 1048576.0f) - 50.0f;
}

// Coordena o processo completo de leitura da temperatura do sensor AHT10
float temperatura_ler(void) {
    uint8_t dados[6];
    aht10_dispara_medicao();
    ms_delay(80);
    aht10_le_dados(dados);
    return temperatura_dados(dados);
}

// Inicializa o SPI1 para comunicação com o NRF24L01
void spi1_inicializa(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_SPI1EN;

    // PA5 = SCK, PA6 = MISO, PA7 = MOSI
    GPIOA->CRL &= ~(GPIO_CRL_MODE5_Msk | GPIO_CRL_CNF5_Msk |
                    GPIO_CRL_MODE6_Msk | GPIO_CRL_CNF6_Msk |
                    GPIO_CRL_MODE7_Msk | GPIO_CRL_CNF7_Msk);
    GPIOA->CRL |= (0b11 << GPIO_CRL_MODE5_Pos) | (0b10 << GPIO_CRL_CNF5_Pos) |
                  (0b00 << GPIO_CRL_MODE6_Pos) | (0b01 << GPIO_CRL_CNF6_Pos) |
                  (0b11 << GPIO_CRL_MODE7_Pos) | (0b10 << GPIO_CRL_CNF7_Pos);

    // PA4 (CSN), PA3 (CE)
    GPIOA->CRL &= ~(GPIO_CRL_MODE3_Msk | GPIO_CRL_CNF3_Msk |
                    GPIO_CRL_MODE4_Msk | GPIO_CRL_CNF4_Msk);
    GPIOA->CRL |= (0b10 << GPIO_CRL_MODE3_Pos) | (0b00 << GPIO_CRL_CNF3_Pos) |
                  (0b10 << GPIO_CRL_MODE4_Pos) | (0b00 << GPIO_CRL_CNF4_Pos);

    NRF24L01_CSN_HIGH();
    NRF24L01_CE_LOW();

    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_SPE | SPI_CR1_SSI | SPI_CR1_SSM;
}

// Transmite e recebe um byte via SPI1
uint8_t spi1_transmite_recebe(uint8_t dado) {
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = dado;
    while (!(SPI1->SR & SPI_SR_RXNE));
    return SPI1->DR;
}

// Escreve um único registro no NRF24L01
void nrf24_escreve_registro(uint8_t reg, uint8_t valor) {
    NRF24L01_CSN_LOW();
    spi1_transmite_recebe(0x20 | reg);
    spi1_transmite_recebe(valor);
    NRF24L01_CSN_HIGH();
}

// Escreve múltiplos bytes em um registro do NRF24L01
void nrf24_escreve_registro_multi(uint8_t reg, uint8_t *dados, uint8_t tamanho) {
    NRF24L01_CSN_LOW();
    spi1_transmite_recebe(0x20 | reg);
    for (uint8_t i = 0; i < tamanho; i++) {
        spi1_transmite_recebe(dados[i]);
    }
    NRF24L01_CSN_HIGH();
}

// Inicializa o NRF24L01 em modo transmissor
void nrf24_inicializa_tx(uint8_t *endereco) {
    ms_delay(5);
    nrf24_escreve_registro(0x00, 0x0A | NRF_CRC_ENABLE); // CONFIG: PWR_UP, PRIM_TX, CRC
    nrf24_escreve_registro(0x01, NRF_AUTO_ACK);          // EN_AA
    nrf24_escreve_registro(0x02, 0x01);                  // Enable pipe 0
    nrf24_escreve_registro(0x03, NRF_ADDRESS_LEN - 1);   // Setup address width
    nrf24_escreve_registro(0x04, 0x00);                  // Disable retransmit
    nrf24_escreve_registro(0x05, NRF_CHANNEL);           // RF channel = 10
    nrf24_escreve_registro(0x06, 0x0E);                  // Data rate = 2Mbps, 0dBm
    nrf24_escreve_registro_multi(0x10, endereco, NRF_ADDRESS_LEN); // TX_ADDR
    nrf24_escreve_registro_multi(0x0A, endereco, NRF_ADDRESS_LEN); // RX_ADDR_P0
    nrf24_escreve_registro(0x11, 32);                    // Payload width
    NRF24L01_CE_HIGH();
    ms_delay(2);
}

// Envia dados via NRF24L01
// Retorna true se o envio foi bem-sucedido, false se falhou
// Se o envio falhar, o LED RGB de status será vermelho
// Se o envio for bem-sucedido, o LED RGB de status será verde
bool nrf24_envia_dados(uint8_t *dados, uint8_t tamanho) {
    NRF24L01_CE_LOW();

    // Envia payload
    NRF24L01_CSN_LOW();
    spi1_transmite_recebe(0xA0);  // Comando para escrever TX payload
    for (uint8_t i = 0; i < tamanho; i++) {
        spi1_transmite_recebe(dados[i]);
    }
    NRF24L01_CSN_HIGH();

    // Pulso de CE para iniciar envio
    NRF24L01_CE_HIGH();
    ms_delay(1);
    NRF24L01_CE_LOW();

    ms_delay(5);  // Aguarda envio

    // Lê o status
    NRF24L01_CSN_LOW();
    uint8_t status = spi1_transmite_recebe(0xFF); // NOP para ler status
    NRF24L01_CSN_HIGH();

    // Limpa os flags
    nrf24_escreve_registro(0x07, status);

    if (status & (1 << 5)) {
        return true; // TX_DS: sucesso
    } else if (status & (1 << 4)) {
        return false; // MAX_RT: falhou
    }

    return false;
}

// Indica o status do envio via LED RGB
void led_rgb_status_envio(bool sucesso) {
    // Apaga LEDs do RGB status
    GPIOB->ODR &= ~(LED_NRF_R_PIN | LED_NRF_G_PIN | LED_NRF_B_PIN);

    if (sucesso) {
        GPIOB->ODR |= LED_NRF_G_PIN;  // Verde se sucesso
    } else {
        GPIOB->ODR |= LED_NRF_R_PIN;  // Vermelho se falha
    }
}

// Configura o relógio do sistema para 72 MHz usando HSE (8 MHz externo)
void config_relogio_sistema(void) {
    // Ativa HSE (8 MHz externo)
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    // Configura PLL: HSE * 9 = 72 MHz
    RCC->CFGR |= RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9;
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    // Flash latency
    FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    // Seleciona PLL como clock principal
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}
