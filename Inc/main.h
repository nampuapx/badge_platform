/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define ACS_INT_Pin GPIO_PIN_13
#define ACS_INT_GPIO_Port GPIOC
#define ACS_INT_EXTI_IRQn EXTI15_10_IRQn
#define VPWR_EN_Pin GPIO_PIN_2
#define VPWR_EN_GPIO_Port GPIOC
#define GPIO_ON_Pin GPIO_PIN_3
#define GPIO_ON_GPIO_Port GPIOC
#define VBAT_4_Pin GPIO_PIN_4
#define VBAT_4_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_5
#define LED_G_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_0
#define LED_B_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_1
#define LED_R_GPIO_Port GPIOB
#define LIS3DH_Pin GPIO_PIN_10
#define LIS3DH_GPIO_Port GPIOB
#define LIS3DHB11_Pin GPIO_PIN_11
#define LIS3DHB11_GPIO_Port GPIOB
#define EPD_RST_Pin GPIO_PIN_6
#define EPD_RST_GPIO_Port GPIOC
#define EPD_IRQ_Pin GPIO_PIN_7
#define EPD_IRQ_GPIO_Port GPIOC
#define EPD_CLK_EN_Pin GPIO_PIN_8
#define EPD_CLK_EN_GPIO_Port GPIOC
#define BAT_ST1_Pin GPIO_PIN_9
#define BAT_ST1_GPIO_Port GPIOC
#define BAT_ST2_Pin GPIO_PIN_8
#define BAT_ST2_GPIO_Port GPIOA
#define DRV_WKUP_Pin GPIO_PIN_9
#define DRV_WKUP_GPIO_Port GPIOA
#define DRV_VCOM_EN_Pin GPIO_PIN_10
#define DRV_VCOM_EN_GPIO_Port GPIOA
#define WLAN_SPI_CC_Pin GPIO_PIN_15
#define WLAN_SPI_CC_GPIO_Port GPIOA
#define WLAN_SPI_SCK_Pin GPIO_PIN_10
#define WLAN_SPI_SCK_GPIO_Port GPIOC
#define WLAN_SPI_MISO_Pin GPIO_PIN_11
#define WLAN_SPI_MISO_GPIO_Port GPIOC
#define WLAN_SPI_MOSI_Pin GPIO_PIN_12
#define WLAN_SPI_MOSI_GPIO_Port GPIOC
#define WLAN_EN_Pin GPIO_PIN_2
#define WLAN_EN_GPIO_Port GPIOD
#define WLAN_IRQ_Pin GPIO_PIN_3
#define WLAN_IRQ_GPIO_Port GPIOB
#define WLAN_IRQ_EXTI_IRQn EXTI3_IRQn
#define DRV_INT_Pin GPIO_PIN_4
#define DRV_INT_GPIO_Port GPIOB
#define DRV_VCOM_SW_Pin GPIO_PIN_5
#define DRV_VCOM_SW_GPIO_Port GPIOB
#define DRV_PWUP_Pin GPIO_PIN_6
#define DRV_PWUP_GPIO_Port GPIOB
#define DRV_PG_Pin GPIO_PIN_7
#define DRV_PG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
