/**
 ******************************************************************************
 * @file    plm_gpio.h
 * @author  CLABS
 * @version V1.0.0
 * @date    8-Nov-2017
 * @brief   This file contains all the functions prototypes for the gpio
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
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported Variables ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLM_GPIO_H
#define __PLM_GPIO_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

/**
 * @addtogroup BSP
 * @{
 */

/* Exported types ------------------------------------------------------------*/
/* MCU GPIO pin working mode for GPIO */
typedef enum {
  PLM_MODE_GPIO_IN = 0x00,      /*!< Work as GPIO input */
  PLM_MODE_EXTI_IN,             /*!< Work as EXTI */
  PLM_MODE_GPIO_OUT,            /*!< Work as GPIO output */
}PlmGpioMode;

/* MCU GPIO pin enumeration for GPIO */
typedef enum {
  PLM_GPIO_T_REQ = 0x00,     /*!< PLM_GPIO_T_REQ selected */
  PLM_GPIO_RESETN = 0x01,    /*!< PLM_GPIO_RESETN1 selected */
  PLM_PL_TX_ON = 0x02,       /*!< PLM_PL_TX_ON selected */
  PLM_PL_RX_ON = 0x03,       /*!< PLM_PL_RX_ON selected */
}PlmGpioPin;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* MCU GPIO pin working mode for GPIO */
#define IS_PLM_GPIO_MODE(MODE) (((MODE) == PLM_MODE_GPIO_IN) || \
                                ((MODE) == PLM_MODE_EXTI_IN) || \
                                ((MODE) == PLM_MODE_GPIO_OUT))

/* Number of Arduino pins used for RADIO GPIO interface */
#define PLM_GPIO_NUMBER    ((uint8_t)4)

/* MCU GPIO pin enumeration for GPIO */
#define IS_PLM_GPIO_PIN(PIN)   (((PIN) == PLM_GPIO_T_REQ) || \
                                ((PIN) == PLM_GPIO_RESETN) || \
                                ((PIN) == PLM_PL_TX_ON) || \
                                ((PIN) == PLM_PL_RX_ON))

/* @defgroup Radio_Gpio_config_Define */

#define PLM_GPIO_T_REQ_PORT                          GPIOA
#define PLM_GPIO_T_REQ_PIN                           GPIO_PIN_5
#define PLM_GPIO_T_REQ_CLOCK_ENABLE()                __GPIOA_CLK_ENABLE()
#define PLM_GPIO_T_REQ_CLOCK_DISABLE()               __GPIOA_CLK_DISABLE()
#define PLM_GPIO_T_REQ_SPEED                         GPIO_SPEED_HIGH
#define PLM_GPIO_T_REQ_PUPD                          GPIO_NOPULL

#define PLM_GPIO_RESETN_PORT                          GPIOA
#define PLM_GPIO_RESETN_PIN                           GPIO_PIN_8
#define PLM_GPIO_RESETN_CLOCK_ENABLE()                __GPIOA_CLK_ENABLE()
#define PLM_GPIO_RESETN_CLOCK_DISABLE()               __GPIOA_CLK_DISABLE()
#define PLM_GPIO_RESETN_SPEED                         GPIO_SPEED_HIGH
#define PLM_GPIO_RESETN_PUPD                          GPIO_NOPULL

#define PLM_PL_TX_ON_PORT                          GPIOC
#define PLM_PL_TX_ON_PIN                           GPIO_PIN_0
#define PLM_PL_TX_ON_CLOCK_ENABLE()                __GPIOC_CLK_ENABLE()
#define PLM_PL_TX_ON_CLOCK_DISABLE()               __GPIOC_CLK_DISABLE()
#define PLM_PL_TX_ON_SPEED                         GPIO_SPEED_HIGH
#define PLM_PL_TX_ON_PUPD                          GPIO_NOPULL
#define PLM_PL_TX_ON_EXTI_LINE                     GPIO_PIN_0
#define PLM_PL_TX_ON_EXTI_MODE                     GPIO_MODE_IT_RISING_FALLING
#define PLM_PL_TX_ON_EXTI_IRQN                     EXTI0_IRQn
#define PLM_PL_TX_ON_EXTI_PREEMPTION_PRIORITY      2
#define PLM_PL_TX_ON_EXTI_SUB_PRIORITY             2
#define PLM_PL_TX_ON_EXTI_IRQ_HANDLER              EXTI0_IRQHandler

#define PLM_PL_RX_ON_PORT                          GPIOC
#define PLM_PL_RX_ON_PIN                           GPIO_PIN_1
#define PLM_PL_RX_ON_CLOCK_ENABLE()                __GPIOC_CLK_ENABLE()
#define PLM_PL_RX_ON_CLOCK_DISABLE()               __GPIOC_CLK_DISABLE()
#define PLM_PL_RX_ON_SPEED                         GPIO_SPEED_HIGH
#define PLM_PL_RX_ON_PUPD                          GPIO_NOPULL
#define PLM_PL_RX_ON_EXTI_LINE                     GPIO_PIN_1
#define PLM_PL_RX_ON_EXTI_MODE                     GPIO_MODE_IT_RISING_FALLING
#define PLM_PL_RX_ON_EXTI_IRQN                     EXTI1_IRQn
#define PLM_PL_RX_ON_EXTI_PREEMPTION_PRIORITY      2
#define PLM_PL_RX_ON_EXTI_SUB_PRIORITY             2
#define PLM_PL_RX_ON_EXTI_IRQ_HANDLER              EXTI1_IRQHandler

/* Exported Variables ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
FlagStatus PlmGpioGetLevel(PlmGpioPin xGpio);
void PlmGpioSetLevel(PlmGpioPin xGpio, GPIO_PinState xState);
void PlmGpioInit(PlmGpioPin xGpio, PlmGpioMode xGpioMode);
void PlmGpioInterruptCmd(PlmGpioPin xGpio, uint8_t nPreemption, uint8_t nSubpriority, FunctionalState xNewState);

#ifdef __cplusplus
}
#endif
#endif /*__PLM_GPIO_H */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
