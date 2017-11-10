/**
 ******************************************************************************
 * @file    stm32l1xx_hal_msp.c
 * @author  CLABS
 * @version V1.0.0
 * @date    8-Nov-2017
 * @brief   HAL MSP file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32cube_hal_init.h"
#include "stm32l1xx_hal.h"
#include "hw-config.h"
#include "dev/button-sensor.h"
/** @addtogroup STM32L1xx_HAL_Examples
 * @{
 */

/** @addtogroup Templates
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint8_t UART_RxBuffer[UART_RxBufferSize];
volatile uint32_t Usart_BaudRate = 115200;
extern UART_HandleTypeDef pUartPlmHandle;
UART_HandleTypeDef UartHandle;
extern const struct sensors_sensor button_sensor;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
 * @{
 */

/**
 * @brief  Initializes the Global MSP.
 * @param  None
 * @retval None
 */
void
HAL_MspInit(void)
{
  /* NOTE : This function is generated automatically by MicroXplorer and eventually
            modified by the user
   */
}
/**
 * @brief  DeInitializes the Global MSP.
 * @param  None
 * @retval None
 */
void
HAL_MspDeInit(void)
{
}
void
HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{

  if(htim_base->Instance == TIM2) {
    /* Peripheral clock enable */
    __TIM2_CLK_ENABLE();

    /**TIM2 GPIO Configuration
       PA0-WKUP     ------> TIM2_CH1
     */
    /* Peripheral interrupt init*/

    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  }
}
void
HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base)
{

  if(htim_base->Instance == TIM2) {
    /* Peripheral clock disable */
    __TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
       PA0-WKUP     ------> TIM2_CH1
     */
    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  }
}
/**
 * @brief UART MSP Initialization
 *        This function configures the hardware resources used in this example:
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration
 *           - NVIC configuration for UART interrupt request enable
 * @param huart: UART handle pointer
 * @retval None
 */
void
HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USARTx) {
    GPIO_InitTypeDef GPIO_InitStruct;

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    USARTx_TX_GPIO_CLK_ENABLE();
    USARTx_RX_GPIO_CLK_ENABLE();
    /* Enable USARTx clock */
    USARTx_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin = USARTx_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = USARTx_TX_AF;

    HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = USARTx_RX_PIN;
    GPIO_InitStruct.Alternate = USARTx_RX_AF;

    HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

    /*##-3- Configure the NVIC for UART ########################################*/
    /* NVIC for USART */
    HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(USARTx_IRQn);
  }

  if(huart->Instance == PLM_USART) {
    GPIO_InitTypeDef GPIO_InitStruct;

    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock */
    PLM_USART_TX_GPIO_CLK_ENABLE();
    PLM_USART_RX_GPIO_CLK_ENABLE();
    /* Enable USART1 clock */
    PLM_USART_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* UART TX GPIO pin configuration  */
    GPIO_InitStruct.Pin = PLM_USART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = PLM_USART_TX_AF;

    HAL_GPIO_Init(PLM_USART_TX_GPIO_PORT, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration  */
    GPIO_InitStruct.Pin = PLM_USART_RX_PIN;
    GPIO_InitStruct.Alternate = PLM_USART_RX_AF;

    HAL_GPIO_Init(PLM_USART_RX_GPIO_PORT, &GPIO_InitStruct);

    /*##-3- Configure the NVIC for UART ########################################*/
    /* NVIC for USART */
    HAL_NVIC_SetPriority(PLM_USART_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(PLM_USART_IRQn);
  }
}
/**
 * @brief UART MSP De-Initialization
 *        This function frees the hardware resources used in this example:
 *          - Disable the Peripheral's clock
 *          - Revert GPIO and NVIC configuration to their default state
 * @param huart: UART handle pointer
 * @retval None
 */
void
HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  /*##-1- Reset peripherals ##################################################*/
  USARTx_FORCE_RESET();
  USARTx_RELEASE_RESET();

  /*##-2- Disable peripherals and GPIO Clocks #################################*/
  /* Configure UART Tx as alternate function  */
  HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
  /* Configure UART Rx as alternate function  */
  HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);

  /*##-3- Disable the NVIC for UART ##########################################*/
  HAL_NVIC_DisableIRQ(USARTx_IRQn);
}
/**
 * @brief  Configure the USART
 * @param  None
 * @retval None
 */
void
USARTConfig(void)
{
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  UartHandle.Instance = USARTx;
  UartHandle.Init.BaudRate = Usart_BaudRate;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits = UART_STOPBITS_1;
  UartHandle.Init.Parity = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode = UART_MODE_TX_RX;

  if(HAL_UART_Init(&UartHandle) != HAL_OK) {
    while(1) ;
  }

  UartHandle.pRxBuffPtr = (uint8_t *)UART_RxBuffer;
  UartHandle.RxXferSize = UART_RxBufferSize;
  UartHandle.ErrorCode = HAL_UART_ERROR_NONE;
}
/**
 * @brief GPIO EXTI callback
 * @param uint16_t GPIO_Pin
 * @retval None
 */
void
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
#if defined(MCU_STOP_MODE) /*if MCU is in stop mode*/

  /* Clear Wake Up Flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

  /* Configures system clock after wake-up from STOP: enable HSE, PLL and select
     PLL as system clock source (HSE and PLL are disabled in STOP mode) */
  SystemClockConfig_STOP();
#endif
#if defined(MCU_SLEEP_MODE)
  /* Resume Tick interrupt if disabled prior to sleep mode entry*/
  HAL_ResumeTick();
#endif

  if(GPIO_Pin == USER_BUTTON_PIN) {
    sensors_changed(&button_sensor);
  }

  /* (GPIO_Pin == PLM_PL_RX_ON_PIN)
     {
     st7580_interrupt_callback();
     }*/
}
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
