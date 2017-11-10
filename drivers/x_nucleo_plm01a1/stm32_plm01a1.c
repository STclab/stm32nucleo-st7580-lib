/**
 ******************************************************************************
 * @file    stm32_plm01a1.c
 * @author  CLABS
 * @version V1.0.0
 * @date    8-Nov-2017
 * @brief   HAL related functionality of X-CUBE-PLM1
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#include "stm32_plm01a1.h"
#include "plm_gpio.h"
#include "hw-config.h"

/** @addtogroup BSP
 * @{
 */

/** @defgroup  NUCLEO_PLM_DRIVER
 * @brief PLM_driver modules
 * @{
 */

/** @defgroup NUCLEO_PLM_DRIVER_Private_Defines
 * @{
 */
/* Private typedef -----------------------------------------------------------*/

/**
 * @brief PlmDriver_t structure fitting
 */
PlmDriver_t st7580_cb =
{
  .Init = ST7580InterfaceInit,
  .Reset = ST7580Reset,
  .MibRead = ST7580MibRead,
  .MibWrite = ST7580MibWrite,
  .MibErase = ST7580MibErase,
  .Ping = ST7580Ping,
  .PhyData = ST7580PhyData,
  .DlData = ST7580DlData,
  .SsData = ST7580SsData,
  .NextIndicationFrame = ST7580NextIndicationFrame
};

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/**
 * @}
 */

/** @addtogroup NUCLEO_PLM_DRIVER_Private_Variables
 * @{
 */
/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef pUartPlmHandle;

PlmDriver_t *pPlmDriver;

/**
 * @}
 */

/** @defgroup NUCLEO_PLM_DRIVER_Private_Functions
 * @{
 */

/**
 * @brief  Initializes the PLM module.
 * @param  None
 * @retval None.
 */
void
BSP_PLM_Init(void)
{
  pPlmDriver = &st7580_cb;
  pPlmDriver->Init();
  return;
}
/**
 * @brief  Reset the PLM module.
 * @param  None
 * @retval None.
 */
int
BSP_PLM_Reset(void)
{
  return pPlmDriver->Reset();
}
/**
 * @brief  Write a PLM MIB parameter.
 * @param  indexMib	Selects MIB parameter to be written
 * @param  bufMib   Pointer to buffer containing value to be written
 * @param  lenBuf		Length of buffer pointed by buf
 * @retval 0 command confirmed, error code otherwise
 */
int
BSP_PLM_Mib_Write(uint8_t indexMib, const uint8_t *bufMib, uint8_t lenBuf)
{
  return pPlmDriver->MibWrite(indexMib, bufMib, lenBuf);
}
/**
 * @brief  Read a PLM MIB parameter.
 * @param  indexMib	Selects MIB parameter to be read
 * @param  bufMib   Pointer to buffer to store read value
 * @param  lenBuf		Length of buffer pointed by buf
 * @retval 0 command confirmed, error code otherwise
 */
int
BSP_PLM_Mib_Read(uint8_t indexMib, uint8_t *bufMib, uint8_t lenBuf)
{
  return pPlmDriver->MibRead(indexMib, bufMib, lenBuf);
}
/**
 * @brief  Erase a PLM MIB parameter.
 * @param  indexMib	Selects MIB parameter to be erased
 * @retval 0 command confirmed, error code otherwise
 */
int
BSP_PLM_Mib_Erase(uint8_t indexMib)
{
  return pPlmDriver->MibErase(indexMib);
}
/**
 * @brief  Ping the PLM module.
 * @param  pingBuf   Pointer to buffer containing ping test data to be sent.
          If ping is success ST7580 PLC Modem will reply with the same data
 * @param  pingLen		Length of buffer pointed by buf
 * @retval 0 command confirmed, error code otherwise
 */
int
BSP_PLM_Ping(const uint8_t *pingBuf, uint8_t pingLen)
{
  return pPlmDriver->Ping(pingBuf, pingLen);
}
/**
 * @brief  Send data via PLM communication.
 * @param  plmOpts  Transmission options
 * @param  dataBuf		Pointer to buffer containing data to be sent.
 * @param  dataLen		Length of buffer pointed by buf
 * @param	confData	Pointer to buffer to store transmission confirmation data
          from ST7580 PLC Modem, if requested
 * @retval 0 command confirmed, error code otherwise
 */
int
BSP_PLM_Send_Data(uint8_t plmOpts, const uint8_t *dataBuf, uint8_t dataLen, uint8_t *confData)
{
#ifdef USE_PHY_DATA
  return pPlmDriver->PhyData(plmOpts, dataBuf, dataLen, confData);
#endif

#ifdef USE_DL_DATA
  return pPlmDriver->DlData(plmOpts, dataBuf, dataLen, confData);
#endif
}
/**
 * @brief  Send encrypted data via PLM communication.
 * @param  plmOpts Transmission options
 * @param  dataBuf	Pointer to buffer containing data to be sent.
 * @param  clrLen Buf portion that has to be sent in clear
 * @param  encLen Buf portion that has to be encypted
 * @param	retData	Pointer to buffer to store transmission confirmation data
          from ST7580 PLC Modem, if requested
 * @retval 0 command confirmed, error code otherwise
 */
int
BSP_PLM_Send_Secure_data(uint8_t plmOpts, const uint8_t *dataBuf, uint8_t clrLen, uint8_t encLen, uint8_t *retData)
{
  return pPlmDriver->SsData(plmOpts, dataBuf, clrLen, encLen, retData);
}
/**
 * @brief  Receive data via PLM communication.
 * @param  None
 * @retval NULL if no indication frame is available, pointer no next indication
          frame otherwise
 */
ST7580Frame *
BSP_PLM_Receive_Frame(void)
{
  return pPlmDriver->NextIndicationFrame();
}
/**
 * @brief  Initializes PLM GPIOs.
 * @param  None
 * @retval None
 */
void
GPIO_PLM_Configuration()
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Clocks enable */
  PLM_GPIO_T_REQ_CLOCK_ENABLE();
  PLM_GPIO_RESETN_CLOCK_ENABLE();
  PLM_PL_TX_ON_CLOCK_ENABLE();
  PLM_PL_RX_ON_CLOCK_ENABLE();

  /* TREQ */
  GPIO_InitStruct.Pin = PLM_GPIO_T_REQ_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = PLM_GPIO_T_REQ_PUPD;
  GPIO_InitStruct.Speed = PLM_GPIO_T_REQ_SPEED;
  HAL_GPIO_Init(PLM_GPIO_T_REQ_PORT, &GPIO_InitStruct);

  /* RESETN */
  GPIO_InitStruct.Pin = PLM_GPIO_RESETN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = PLM_GPIO_RESETN_PUPD;
  GPIO_InitStruct.Speed = PLM_GPIO_RESETN_SPEED;
  HAL_GPIO_Init(PLM_GPIO_RESETN_PORT, &GPIO_InitStruct);

  /* TX_ON */
  GPIO_InitStruct.Pin = PLM_PL_TX_ON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = PLM_PL_TX_ON_PUPD;
  GPIO_InitStruct.Speed = PLM_PL_TX_ON_SPEED;
  HAL_GPIO_Init(PLM_PL_TX_ON_PORT, &GPIO_InitStruct);

  /* RX_ON */
  GPIO_InitStruct.Pin = PLM_PL_RX_ON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = PLM_PL_RX_ON_PUPD;
  GPIO_InitStruct.Speed = PLM_PL_RX_ON_SPEED;
  HAL_GPIO_Init(PLM_PL_RX_ON_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(PLM_GPIO_T_REQ_PORT, PLM_GPIO_T_REQ_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PLM_GPIO_RESETN_PORT, PLM_GPIO_RESETN_PIN, GPIO_PIN_SET);
}
/**
 * @brief  Initializes PLM UART HAL.
 * @param  None
 * @retval None
 */

void
UART_PLM_Configuration(void)
{
  pUartPlmHandle.Instance = PLM_USART;
  pUartPlmHandle.Init.BaudRate = PLM_USART_BAUDRATE;
  pUartPlmHandle.Init.WordLength = UART_WORDLENGTH_8B;
  pUartPlmHandle.Init.StopBits = UART_STOPBITS_1;
  pUartPlmHandle.Init.Parity = UART_PARITY_NONE;
  pUartPlmHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  pUartPlmHandle.Init.Mode = UART_MODE_TX_RX;

  if(HAL_UART_Init(&pUartPlmHandle) != HAL_OK) {
    while(1) ;
  }
  __HAL_UART_ENABLE_IT(&pUartPlmHandle, UART_IT_RXNE);
}
/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
