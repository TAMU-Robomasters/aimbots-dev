/*
 * Copyright (c) 2009, Martin Rosekeit
 * Copyright (c) 2009, Thorsten Lajewski
 * Copyright (c) 2009-2010, 2016, Fabian Greif
 * Copyright (c) 2012-2013, 2016, 2018 Niklas Hauser
 * Copyright (c) 2013, Kevin Laeufer
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_DEVICE_HPP
#define MODM_DEVICE_HPP

#define STM32F427xx 1
#include <stdint.h>
// Defines for example the modm_always_inline macro
#include <modm/architecture/utils.hpp>

// Include external device headers:
#include <stm32f427xx.h>
#include <system_stm32f4xx.h>
/// @cond
// This is a hack to make the *_Typedef's known to GDB, so that you can debug
// the peripherals directly in GDB in any context.
// Otherwise GDB would throw a "no symbol 'GPIO_TypeDef' in current context".
extern ADC_TypeDef				___ADC1			;
extern ADC_Common_TypeDef		___ADC123_COMMON	;
extern ADC_TypeDef				___ADC2			;
extern ADC_TypeDef				___ADC3			;
extern CAN_TypeDef				___CAN1			;
extern CAN_TypeDef				___CAN2			;
extern CRC_TypeDef				___CRC				;
extern CoreDebug_Type			___CoreDebug		;
extern DAC_TypeDef				___DAC				;
extern DAC_TypeDef				___DAC1			;
extern DBGMCU_TypeDef			___DBGMCU			;
extern DCMI_TypeDef			___DCMI			;
extern DMA_TypeDef				___DMA1			;
extern DMA_Stream_TypeDef		___DMA1_Stream0	;
extern DMA_Stream_TypeDef		___DMA1_Stream1	;
extern DMA_Stream_TypeDef		___DMA1_Stream2	;
extern DMA_Stream_TypeDef		___DMA1_Stream3	;
extern DMA_Stream_TypeDef		___DMA1_Stream4	;
extern DMA_Stream_TypeDef		___DMA1_Stream5	;
extern DMA_Stream_TypeDef		___DMA1_Stream6	;
extern DMA_Stream_TypeDef		___DMA1_Stream7	;
extern DMA_TypeDef				___DMA2			;
extern DMA2D_TypeDef			___DMA2D			;
extern DMA_Stream_TypeDef		___DMA2_Stream0	;
extern DMA_Stream_TypeDef		___DMA2_Stream1	;
extern DMA_Stream_TypeDef		___DMA2_Stream2	;
extern DMA_Stream_TypeDef		___DMA2_Stream3	;
extern DMA_Stream_TypeDef		___DMA2_Stream4	;
extern DMA_Stream_TypeDef		___DMA2_Stream5	;
extern DMA_Stream_TypeDef		___DMA2_Stream6	;
extern DMA_Stream_TypeDef		___DMA2_Stream7	;
extern DWT_Type				___DWT				;
extern ETH_TypeDef				___ETH				;
extern EXTI_TypeDef			___EXTI			;
extern FLASH_TypeDef			___FLASH			;
extern FMC_Bank1_TypeDef		___FMC_Bank1		;
extern FMC_Bank1E_TypeDef		___FMC_Bank1E		;
extern FMC_Bank2_3_TypeDef		___FMC_Bank2_3		;
extern FMC_Bank4_TypeDef		___FMC_Bank4		;
extern FMC_Bank5_6_TypeDef		___FMC_Bank5_6		;
extern FPU_Type				___FPU				;
extern GPIO_TypeDef			___GPIOA			;
extern GPIO_TypeDef			___GPIOB			;
extern GPIO_TypeDef			___GPIOC			;
extern GPIO_TypeDef			___GPIOD			;
extern GPIO_TypeDef			___GPIOE			;
extern GPIO_TypeDef			___GPIOF			;
extern GPIO_TypeDef			___GPIOG			;
extern GPIO_TypeDef			___GPIOH			;
extern GPIO_TypeDef			___GPIOI			;
extern GPIO_TypeDef			___GPIOJ			;
extern GPIO_TypeDef			___GPIOK			;
extern I2C_TypeDef				___I2C1			;
extern I2C_TypeDef				___I2C2			;
extern I2C_TypeDef				___I2C3			;
extern SPI_TypeDef				___I2S2ext			;
extern SPI_TypeDef				___I2S3ext			;
extern ITM_Type				___ITM				;
extern IWDG_TypeDef			___IWDG			;
extern MPU_Type				___MPU				;
extern NVIC_Type				___NVIC			;
extern PWR_TypeDef				___PWR				;
extern RCC_TypeDef				___RCC				;
extern RNG_TypeDef				___RNG				;
extern RTC_TypeDef				___RTC				;
extern SAI_TypeDef				___SAI1			;
extern SAI_Block_TypeDef		___SAI1_Block_A	;
extern SAI_Block_TypeDef		___SAI1_Block_B	;
extern SCB_Type				___SCB				;
extern SCnSCB_Type				___SCnSCB			;
extern SDIO_TypeDef			___SDIO			;
extern SPI_TypeDef				___SPI1			;
extern SPI_TypeDef				___SPI2			;
extern SPI_TypeDef				___SPI3			;
extern SPI_TypeDef				___SPI4			;
extern SPI_TypeDef				___SPI5			;
extern SPI_TypeDef				___SPI6			;
extern SYSCFG_TypeDef			___SYSCFG			;
extern SysTick_Type			___SysTick			;
extern TIM_TypeDef				___TIM1			;
extern TIM_TypeDef				___TIM10			;
extern TIM_TypeDef				___TIM11			;
extern TIM_TypeDef				___TIM12			;
extern TIM_TypeDef				___TIM13			;
extern TIM_TypeDef				___TIM14			;
extern TIM_TypeDef				___TIM2			;
extern TIM_TypeDef				___TIM3			;
extern TIM_TypeDef				___TIM4			;
extern TIM_TypeDef				___TIM5			;
extern TIM_TypeDef				___TIM6			;
extern TIM_TypeDef				___TIM7			;
extern TIM_TypeDef				___TIM8			;
extern TIM_TypeDef				___TIM9			;
extern TPI_Type				___TPI				;
extern USART_TypeDef			___UART4			;
extern USART_TypeDef			___UART5			;
extern USART_TypeDef			___UART7			;
extern USART_TypeDef			___UART8			;
extern USART_TypeDef			___USART1			;
extern USART_TypeDef			___USART2			;
extern USART_TypeDef			___USART3			;
extern USART_TypeDef			___USART6			;
extern WWDG_TypeDef			___WWDG			;
/// @endcond

#endif  // MODM_DEVICE_HPP