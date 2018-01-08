#include "main.h"

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef   GPIO_InitStruct = {0};
  static DMA_HandleTypeDef  DmaHandle = {0};
  RCC_OscInitTypeDef RCC_OscInitStructure = {0};
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable clock of GPIO associated to the peripheral channels */
  ADCx_CHANNELa_GPIO_CLK_ENABLE();
  
  /* Enable clock of ADCx peripheral */
  ADCx_CLK_ENABLE();

  /* Note: STM32L1xx ADC is using a dedicated asynchronous clock derived        */
  /*       from HSI RC oscillator 16MHz.                                      */
  /*       The clock source has to be enabled at RCC top level using function */
  /*       "HAL_RCC_OscConfig()" (see comments in stm32l1xx_hal_adc.c header)   */

  /* Enable asynchronous clock source of ADCx */
  HAL_RCC_GetOscConfig(&RCC_OscInitStructure);
  RCC_OscInitStructure.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStructure.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStructure.HSIState = RCC_HSI_ON;
  HAL_RCC_OscConfig(&RCC_OscInitStructure);
  
  /* Enable clock of DMA associated to the peripheral */
  ADCx_DMA_CLK_ENABLE();
  
  /*##-2- Configure peripheral GPIO ##########################################*/
  /* Configure GPIO pin of the selected ADC channel */
  GPIO_InitStruct.Pin = ADCx_CHANNEL_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADCx_CHANNELa_GPIO_PORT, &GPIO_InitStruct);

  /* Configure GPIO pin of the selected ADC channel */
  GPIO_InitStruct.Pin = ADCx_CHANNEL_PIN_2;
  HAL_GPIO_Init(ADCx_CHANNELa_GPIO_PORT, &GPIO_InitStruct);

  /* Configure GPIO pin of the selected ADC channel */
  GPIO_InitStruct.Pin = ADCx_CHANNEL_PIN_3;
  HAL_GPIO_Init(ADCx_CHANNELa_GPIO_PORT, &GPIO_InitStruct);

  /* Configure GPIO pin of the selected ADC channel */
  GPIO_InitStruct.Pin = ADCx_CHANNEL_PIN_4;
  HAL_GPIO_Init(ADCx_CHANNELa_GPIO_PORT, &GPIO_InitStruct);

  /*##-3- Configure the DMA ##################################################*/
  /* Configure DMA parameters */
  DmaHandle.Instance = ADCx_DMA;

  DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
  DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;   /* Transfer from ADC by half-word to match with ADC configuration: ADC resolution 10 or 12 bits */
  DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;   /* Transfer to memory by half-word to match with buffer variable type: half-word */
  DmaHandle.Init.Mode                = DMA_CIRCULAR;              /* DMA in circular mode to match with ADC configuration: DMA continuous requests */
  DmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;
  
  /* Deinitialize  & Initialize the DMA for new transfer */
  HAL_DMA_DeInit(&DmaHandle);
  HAL_DMA_Init(&DmaHandle);

  /* Associate the initialized DMA handle to the ADC handle */
  __HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);
  
  /*##-4- Configure the NVIC #################################################*/

  /* NVIC configuration for DMA interrupt (transfer completion or error) */
  /* Priority: high-priority */
  HAL_NVIC_SetPriority(ADCx_DMA_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(ADCx_DMA_IRQn);
  

  /* NVIC configuration for ADC interrupt */
  /* Priority: high-priority */
  HAL_NVIC_SetPriority(ADCx_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADCx_IRQn);
}

/**
  * @brief ADC MSP de-initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable clock of ADC peripheral
  *          - Revert GPIO associated to the peripheral channels to their default state
  *          - Revert DMA associated to the peripheral to its default state
  *          - Revert NVIC associated to the peripheral interruptions to its default state
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  /*##-1- Reset peripherals ##################################################*/
  ADCx_FORCE_RESET();
  ADCx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks ################################*/
  /* De-initialize GPIO pin of the selected ADC channel */
  HAL_GPIO_DeInit(ADCx_CHANNELa_GPIO_PORT, ADCx_CHANNEL_PIN_1);
  HAL_GPIO_DeInit(ADCx_CHANNELa_GPIO_PORT, ADCx_CHANNEL_PIN_2);
  HAL_GPIO_DeInit(ADCx_CHANNELa_GPIO_PORT, ADCx_CHANNEL_PIN_3);
  HAL_GPIO_DeInit(ADCx_CHANNELa_GPIO_PORT, ADCx_CHANNEL_PIN_4);

  /*##-3- Disable the DMA ####################################################*/
  /* De-Initialize the DMA associated to the peripheral */
  if (hadc->DMA_Handle != NULL)
  {
    HAL_DMA_DeInit(hadc->DMA_Handle);
  }

  /*##-4- Disable the NVIC ###################################################*/
  /* Disable the NVIC configuration for DMA interrupt */
  HAL_NVIC_DisableIRQ(ADCx_DMA_IRQn);
  
  /* Disable the NVIC configuration for ADC interrupt */
  HAL_NVIC_DisableIRQ(ADCx_IRQn);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  /* TIM peripheral clock enable */
  TIMx_CLK_ENABLE();

//  HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
//
//  /* Enable the TIMx global Interrupt */
//  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
  * @brief TIM MSP de-initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable clock of peripheral
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim)
{
  /*##-1- Reset peripherals ##################################################*/
  TIMx_FORCE_RESET();
  TIMx_RELEASE_RESET();
//  HAL_NVIC_DisableIRQ(TIM2_IRQn);
}