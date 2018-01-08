

#include "main.h"
#include "uart_print.h"

/* Timer frequency (unit: Hz). With SysClk set to 32MHz, timer frequency TIMER_FREQUENCY_HZ range is min=1Hz, max=32.719kHz. */
#define TIMER_FREQUENCY_HZ          4000

/* ADC parameters */
/* Size of array containing ADC converted values: set to ADC sequencer number of ranks converted, to have a rank in each address */
#define ADC_CHANNELS 4
#define ADC_CONVERSIONS_BUFFER_SIZE 200

/**
  * @brief  Computation of voltage (unit: mV) from ADC measurement digital
  *         value on range 12 bits.
  *         Calculation validity conditioned to settings: 
  *          - ADC resolution 12 bits (need to scale value if using a different 
  *            resolution).
  *          - Power supply of analog voltage Vdda 3.3V (need to scale value 
  *            if using a different analog voltage supply value).
  * @param ADC_DATA: Digital value measured by ADC
  * @retval None
  */
#define RANGE_12BITS          4095    /* Max digital value with a full range of 12 bits */
#define VOLTAGE_12B(ADC_DATA) ( ((ADC_DATA) * VDD_VALUE) / RANGE_12BITS)

TIM_HandleTypeDef TimHandle;

ADC_HandleTypeDef AdcHandle;

/* Variable containing ADC conversions results */
__IO uint16_t aADCxConvertedValues[ADC_CONVERSIONS_BUFFER_SIZE];

/* Variables to manage push button on board: interface between ExtLine interruption and main program */
//__IO uint8_t ubUserButtonClickEvent = RESET;  /* Event detection: Set after User Button interrupt */

/* Variable to report ADC sequencer status */
__IO uint8_t ubSequenceCompleted = RESET;     /* Set when all ranks of the sequence have been converted */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void Error_Handler(uint32_t code);

static void TIM_Config(void);

static void ADC_Config(void);


int main(void)
{
  /* STM32L1xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 32 MHz */
  SystemClock_Config();

  /*## Configure peripherals #################################################*/

  /* Initialize LED on board */
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);

  BSP_LED_On(LED4);
  BSP_LED_On(LED3);
  HAL_Delay(500);
  BSP_LED_Off(LED4);
  BSP_LED_Off(LED3);
  HAL_Delay(500);

  /* Configure User push-button in Interrupt mode */
//    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /*## Enable peripherals ####################################################*/
  /* Configure the ADCx peripheral */
  ADC_Config();

  /* Configure the TIM peripheral */
  TIM_Config();

  /*## Start ADC conversions #################################################*/
  /* Start ADC conversion on regular group with transfer by DMA and start timer that triggers conversion */
  if (uart_init() != HAL_OK ||
      HAL_TIM_Base_Start(&TimHandle) != HAL_OK ||
//      HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK ||
      HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *) aADCxConvertedValues, ADC_CONVERSIONS_BUFFER_SIZE) != HAL_OK) {
    Error_Handler(1);
  }

  for (;;) {
    while (ubSequenceCompleted == RESET) {}
////      uhADCChannelToDAC_mVolt    = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(aADCxConvertedValues[0]);
////      uhVrefInt_mVolt            = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(aADCxConvertedValues[1]);
////      wTemperature_DegreeCelsius = COMPUTATION_TEMPERATURE_TEMP30_TEMP110(aADCxConvertedValues[2]);

//        uint32_t voltage = 0;
//        for(int i = 0; i< ADCCONVERTEDVALUES_BUFFER_SIZE; i++)
//            voltage += aADCxConvertedValues[i];
//        voltage /= ADCCONVERTEDVALUES_BUFFER_SIZE;
//        voltage = VOLTAGE_12B(voltage);

//    for(int frame = 0; frame < ADC_CONVERSIONS_BUFFER_SIZE; frame += 4) {
//      uart_print(snprintf(buffer, STRING_LENGTH, "V(a0) = %04d, V(a1) = %04d, V(a2) = %04d, V(a3) = %04d\r\n",
//                          VOLTAGE_12B(aADCxConvertedValues[frame]),
//                          VOLTAGE_12B(aADCxConvertedValues[frame + 1]),
//                          VOLTAGE_12B(aADCxConvertedValues[frame + 2]),
//                          VOLTAGE_12B(aADCxConvertedValues[frame + 3])));
//    }


//    uart_print(snprintf(buffer, STRING_LENGTH, "\r\nV = %04d\r\n", VOLTAGE_12B(aADCxConvertedValues[0])));

    BSP_LED_Toggle(LED3);

//    HAL_Delay(200);
    ubSequenceCompleted = RESET;
  }
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLLMUL                         = 6
  *            PLLDIV                         = 3
  *            Flash Latency(WS)              = 1
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable HSE Oscillator and Activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    /* Initialization Error */
    while (1);
  }

  /* Set Voltage scale1 as MCU will run at 32MHz */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {};

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 |
                                 RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    /* Initialization Error */
    while (1);
  }
}

static void TIM_Config(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;// = {0};

  /* Time Base configuration */
  TimHandle.Instance = TIMx;

  /* Configure timer frequency */
  /* Note: Setting of timer prescaler to 489 to increase the maximum range    */
  /*       of the timer, to fit within timer range of 0xFFFF.                 */
  /*       Setting of reload period to SysClk/489 to maintain a base          */
  /*       frequency of 1us.                                                  */
  /*       With SysClk set to 32MHz, timer frequency (defined by label        */
  /*       TIMER_FREQUENCY_HZ range) is min=1Hz, max=32.719kHz.               */
  /* Note: Timer clock source frequency is retrieved with function            */
  /*       HAL_RCC_GetPCLK1Freq().                                            */
  /*       Alternate possibility, depending on prescaler settings:            */
  /*       use variable "SystemCoreClock" holding HCLK frequency, updated by  */
  /*       function HAL_RCC_ClockConfig().                                    */
  TimHandle.Init.Period = ((HAL_RCC_GetPCLK1Freq() / (489 * TIMER_FREQUENCY_HZ)) - 1);
  TimHandle.Init.Prescaler = (489 - 1);
  TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;

  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK) {
    /* Timer initialization Error */
    Error_Handler(2);
  }

  /* Timer TRGO selection */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig) != HAL_OK) {
    /* Timer TRGO selection Error */
    Error_Handler(3);
  }
}

/**
  * @brief  ADC configuration
  * @param  None
  * @retval None
  */
static void ADC_Config(void)
{
  ADC_ChannelConfTypeDef sConfig;

  /* Configuration of AdcHandle init structure: ADC parameters and regular group */
  AdcHandle.Instance = ADCx;

  if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK) {
    /* ADC initialization error */
    Error_Handler(4);
  }

  AdcHandle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
  AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.ScanConvMode = ADC_SCAN_ENABLE;               /* Sequencer enabled (ADC conversion on several channels, successively, following settings below) */
  AdcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  AdcHandle.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  AdcHandle.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  AdcHandle.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  AdcHandle.Init.ContinuousConvMode = DISABLE;                     /* Continuous mode disabled to have only 1 rank converted at each conversion trig, and because discontinuous mode is enabled */
  AdcHandle.Init.NbrOfConversion = ADC_CHANNELS; /* Sequencer of regular group will convert the 4 first ranks: rank1, rank2, rank3, rank4 */
  AdcHandle.Init.DiscontinuousConvMode = ENABLE;                   /* Sequencer of regular group will convert the sequence in several sub-divided sequences */
  AdcHandle.Init.NbrOfDiscConversion = 1;                          /* Sequencer of regular group will convert ranks one by one, at each conversion trig */
  AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_Tx_TRGO;  /* Trig of conversion start done by external event */
  AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  AdcHandle.Init.DMAContinuousRequests = ENABLE;

//    AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV4;
//    AdcHandle.Init.Resolution            = ADC_RESOLUTION_12B;
//    AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
//    AdcHandle.Init.ScanConvMode          = ADC_SCAN_DISABLE;                       /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
//    AdcHandle.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
//    AdcHandle.Init.LowPowerAutoWait      = ADC_AUTOWAIT_DISABLE;
//    AdcHandle.Init.LowPowerAutoPowerOff  = ADC_AUTOPOWEROFF_DISABLE;
//    AdcHandle.Init.ChannelsBank          = ADC_CHANNELS_BANK_A;
//    AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
//    AdcHandle.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
//    AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
//    AdcHandle.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
//    AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_Tx_TRGO;  /* Trig of conversion start done by external event */
//    AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
//    AdcHandle.Init.DMAContinuousRequests = ENABLE;

  if (HAL_ADC_Init(&AdcHandle) != HAL_OK) {
    /* ADC initialization error */
    Error_Handler(5);
  }

  /* Configuration of channel on ADCx regular group on sequencer rank 1 */
  /* Note: Considering IT occurring after each ADC conversion (IT by DMA end  */
  /*       of transfer), select sampling time and ADC clock with sufficient   */
  /*       duration to not create an overhead situation in IRQHandler.        */
  /* Note: Set long sampling time due to internal channels (VrefInt,          */
  /*       temperature sensor) constraints.                                   */
  /*       For example, sampling time of temperature sensor must be higher    */
  /*       than 4us. Refer to device datasheet for min/typ/max values.        */
  sConfig.Channel = ADCx_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_96CYCLES;

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
    /* Channel Configuration Error */
    Error_Handler(6);
  }

  /* Configuration of channel on ADCx regular group on sequencer rank 2 */
  /* Replicate previous rank settings, change only channel and rank */
  sConfig.Channel = ADCx_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
    /* Channel Configuration Error */
    Error_Handler(7);
  }

  /* Configuration of channel on ADCx regular group on sequencer rank 3 */
  /* Replicate previous rank settings, change only channel and rank */
  sConfig.Channel = ADCx_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
    /* Channel Configuration Error */
    Error_Handler(8);
  }

  sConfig.Channel = ADCx_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
    /* Channel Configuration Error */
    Error_Handler(9);
  }
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
//  BSP_LED_Toggle(LED3);
  /* Report to main program that ADC sequencer has reached its end */
  ubSequenceCompleted = SET;
}

/**
  * @brief  ADC error callback in non blocking mode
  *        (ADC conversion with interruption or transfer by DMA)
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  uart_print(snprintf(buffer, STRING_LENGTH, "ADC error = 0x%08lx\r\n", hadc->ErrorCode));
  /* In case of ADC error, call main error handler */
  Error_Handler(10);
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//  BSP_LED_Toggle(LED4);
//}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(uint32_t code)
{
  /* User may add here some code to deal with a potential error */
  uart_print(snprintf(buffer, STRING_LENGTH, "Error code = %ld\r\n", code));

  /* In case of error, LED3 is toggling at a frequency of 1Hz */
  for(;;) {
    /* Toggle LED3 */
    BSP_LED_Toggle(LED4);
    BSP_LED_Toggle(LED3);

    HAL_Delay(200);
  }
}
