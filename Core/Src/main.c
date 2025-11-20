/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "liquidcrystal_i2c.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

typedef enum {
    STATE_NORMAL,
    STATE_DIAGNOSTIC,
    STATE_EMERGENCY
} SystemState;

volatile SystemState currentState = STATE_NORMAL;

// DMA buffers
uint32_t adc1_buffer[2]; // ADC1: temp, humidity
uint32_t adc2_buffer[2]; // ADC2: water, LDR

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void Buzzer_Alert(uint8_t times, uint16_t duration);

void Buzzer_Alert(uint8_t times, uint16_t duration) {
    for(uint8_t i=0;i<times;i++){
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_Delay(duration);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_Delay(duration);
    }
}

/* Interrupt callback for buttons --------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == GPIO_PIN_0) currentState = STATE_DIAGNOSTIC;
    else if(GPIO_Pin == GPIO_PIN_1) currentState = STATE_EMERGENCY;
}

/* MAIN ----------------------------------------------------------------------*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();

    // Start ADCs with DMA
    HAL_ADC_Start_DMA(&hadc1, adc1_buffer, 2);
    HAL_ADC_Start_DMA(&hadc2, adc2_buffer, 2);

    HD44780_Init(2);
    HD44780_Clear();
    HD44780_SetCursor(0,0);
    HD44780_PrintStr("System Ready");

    while (1)
    {
        switch (currentState)
        {
            case STATE_NORMAL:
            {
                char buf[32];
                HD44780_Clear();
                HD44780_SetCursor(0,0);

                // Convert ADC readings
                int temp_c = (int)(((float)adc1_buffer[0]/4095.0f)*330); // *10 for 1 decimal
                int humidity = (int)(((float)adc1_buffer[1]/4095.0f)*100);

                int water = (int)(((float)adc2_buffer[0]/4095.0f)*100);
                int ldr = (int)(((float)adc2_buffer[1]/4095.0f)*100);

                sprintf(buf, "T:%d.%d H:%d%%", temp_c/10, temp_c%10, humidity);
                HD44780_SetCursor(0,0);
                HD44780_PrintStr(buf);
                HAL_Delay(2000);

                HD44780_Clear();
                sprintf(buf, "W:%d%% LDR:%d%%", water, ldr);
                HD44780_SetCursor(0,0);
                HD44780_PrintStr(buf);

                HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1); // LED clignote
                HAL_Delay(2000);
                break;
            }

            case STATE_DIAGNOSTIC:
            {
                HD44780_Clear();
                HD44780_SetCursor(0,0);
                HD44780_PrintStr("Diagnostic...");
                HAL_Delay(1000);

                Buzzer_Alert(2,150);
                HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
                HAL_Delay(500);

                HD44780_Clear();
                HD44780_SetCursor(0,0);
                HD44780_PrintStr("Diag OK");
                HAL_Delay(2000);
                currentState = STATE_NORMAL;
                break;
            }

            case STATE_EMERGENCY:
            {
                HD44780_Clear();
                HD44780_SetCursor(0,0);
                HD44780_PrintStr("Emergency!");
                Buzzer_Alert(3,300);

                for(int i=0;i<10;i++){
                    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);
                    HAL_Delay(200);
                }
                currentState = STATE_NORMAL;
                break;
            }
        }
    }
}

/* System Clock Configuration ------------------------------------------------*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        Error_Handler();
}

/* GPIO Init -----------------------------------------------------------------*/
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

    // PE0 bouton diagnostic
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    // PD1 bouton urgence
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // PE1 LED verte
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    // PC1 buzzer
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // NVIC pour interruptions boutons
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

/* I2C Init -----------------------------------------------------------------*/
void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

/* DMA Init -----------------------------------------------------------------*/
void MX_DMA_Init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    // DMA ADC1
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_adc1);
    __HAL_LINKDMA(&hadc1,DMA_Handle,hdma_adc1);
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn,0,0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    // DMA ADC2
    hdma_adc2.Instance = DMA2_Stream2;
    hdma_adc2.Init.Channel = DMA_CHANNEL_1;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc2.Init.Mode = DMA_CIRCULAR;
    hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_adc2);
    __HAL_LINKDMA(&hadc2,DMA_Handle,hdma_adc2);
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn,0,0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

/* ADC1 Init: Channels 0=Temp, 1=Humidity */
void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 2;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&hadc1);

    sConfig.Rank = 1; sConfig.Channel = ADC_CHANNEL_0; HAL_ADC_ConfigChannel(&hadc1,&sConfig);
    sConfig.Rank = 2; sConfig.Channel = ADC_CHANNEL_1; HAL_ADC_ConfigChannel(&hadc1,&sConfig);
}

/* ADC2 Init: Channels 2=Water, 3=LDR */
void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode = ENABLE;
    hadc2.Init.ContinuousConvMode = ENABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 2;
    hadc2.Init.DMAContinuousRequests = ENABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&hadc2);

    sConfig.Rank = 1; sConfig.Channel = ADC_CHANNEL_2; HAL_ADC_ConfigChannel(&hadc2,&sConfig);
    sConfig.Rank = 2; sConfig.Channel = ADC_CHANNEL_3; HAL_ADC_ConfigChannel(&hadc2,&sConfig);
}

/* Error Handler ------------------------------------------------------------*/
void Error_Handler(void)
{
    __disable_irq();
    while(1) {}
}
