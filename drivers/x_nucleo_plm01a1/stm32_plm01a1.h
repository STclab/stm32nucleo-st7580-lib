/**
 ******************************************************************************
 * @file    stm32_plm01a1.h
 * @author  CLABS
 * @version V1.0.0
 * @date    8-Nov-2017
 * @brief   Header file for HAL related functionality of X-CUBE-PLM1
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLM_MODULE_CONF_H
#define __PLM_MODULE_CONF_H

/* Includes ------------------------------------------------------------------*/

#include "ST7580_Serial.h"
/* Exported types ------------------------------------------------------------*/

typedef struct sPlmDriver {
  void (*Init)();
  int (*Reset)();
  int (*MibRead)(uint8_t, uint8_t *, uint8_t);
  int (*MibWrite)(uint8_t, const uint8_t *, uint8_t);
  int (*MibErase)(uint8_t index);
  int (*Ping)(const uint8_t *, uint8_t);
  int (*PhyData)(uint8_t, const uint8_t *, uint8_t, uint8_t *);
  int (*DlData)(uint8_t, const uint8_t *, uint8_t, uint8_t *);
  int (*SsData)(uint8_t, const uint8_t *, uint8_t, uint8_t, uint8_t *);
  ST7580Frame * (*NextIndicationFrame)();
}PlmDriver_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported Variables --------------------------------------------------------*/
extern UART_HandleTypeDef pUartPlmHandle;

extern PlmDriver_t *pPlmDriver;

void GPIO_PLM_Configuration(void);
void UART_PLM_Configuration(void);
void USART_PRINT_MSG_Configuration(void);

void BSP_PLM_Init(void);
int BSP_PLM_Reset(void);
int BSP_PLM_Mib_Write(uint8_t indexMib, const uint8_t *bufMib, uint8_t lenBuf);
int BSP_PLM_Mib_Read(uint8_t indexMib, uint8_t *bufMib, uint8_t lenBuf);
int BSP_PLM_Mib_Erase(uint8_t indexMib);
int BSP_PLM_Ping(const uint8_t *pingBuf, uint8_t pingLen);
int BSP_PLM_Send_Data(uint8_t plmOpts, const uint8_t *dataBuf, uint8_t dataLen, uint8_t *confData);
int BSP_PLM_Send_Secure_data(uint8_t plmOpts, const uint8_t *dataBuf, uint8_t clrLen, uint8_t encLen, uint8_t *retData);
ST7580Frame *BSP_PLM_Receive_Frame(void);
#endif /* __PLM_MODULE_CONF_H */
