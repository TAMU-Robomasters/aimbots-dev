/*
 * Copyright (c) 2021, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include <stdint.h>
#include <modm/architecture/utils.hpp>
#include <string_view>

extern "C"
{

void Reset_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void WWDG_IRQHandler(void);
void PVD_AVD_IRQHandler(void);
void TAMP_STAMP_IRQHandler(void);
void RTC_WKUP_IRQHandler(void);
void FLASH_IRQHandler(void);
void RCC_IRQHandler(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void DMA1_Stream0_IRQHandler(void);
void DMA1_Stream1_IRQHandler(void);
void DMA1_Stream2_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);
void DMA1_Stream4_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);
void ADC_IRQHandler(void);
void FDCAN1_IT0_IRQHandler(void);
void FDCAN2_IT0_IRQHandler(void);
void FDCAN1_IT1_IRQHandler(void);
void FDCAN2_IT1_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void TIM1_BRK_IRQHandler(void);
void TIM1_UP_IRQHandler(void);
void TIM1_TRG_COM_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void I2C2_EV_IRQHandler(void);
void I2C2_ER_IRQHandler(void);
void SPI1_IRQHandler(void);
void SPI2_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void RTC_Alarm_IRQHandler(void);
void TIM8_BRK_TIM12_IRQHandler(void);
void TIM8_UP_TIM13_IRQHandler(void);
void TIM8_TRG_COM_TIM14_IRQHandler(void);
void TIM8_CC_IRQHandler(void);
void DMA1_Stream7_IRQHandler(void);
void FMC_IRQHandler(void);
void SDMMC1_IRQHandler(void);
void TIM5_IRQHandler(void);
void SPI3_IRQHandler(void);
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);
void TIM7_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
void DMA2_Stream3_IRQHandler(void);
void DMA2_Stream4_IRQHandler(void);
void ETH_IRQHandler(void);
void ETH_WKUP_IRQHandler(void);
void FDCAN_CAL_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);
void DMA2_Stream7_IRQHandler(void);
void USART6_IRQHandler(void);
void I2C3_EV_IRQHandler(void);
void I2C3_ER_IRQHandler(void);
void OTG_HS_EP1_OUT_IRQHandler(void);
void OTG_HS_EP1_IN_IRQHandler(void);
void OTG_HS_WKUP_IRQHandler(void);
void OTG_HS_IRQHandler(void);
void DCMI_PSSI_IRQHandler(void);
void RNG_IRQHandler(void);
void FPU_IRQHandler(void);
void UART7_IRQHandler(void);
void UART8_IRQHandler(void);
void SPI4_IRQHandler(void);
void SPI5_IRQHandler(void);
void SPI6_IRQHandler(void);
void SAI1_IRQHandler(void);
void LTDC_IRQHandler(void);
void LTDC_ER_IRQHandler(void);
void DMA2D_IRQHandler(void);
void OCTOSPI1_IRQHandler(void);
void LPTIM1_IRQHandler(void);
void CEC_IRQHandler(void);
void I2C4_EV_IRQHandler(void);
void I2C4_ER_IRQHandler(void);
void SPDIF_RX_IRQHandler(void);
void DMAMUX1_OVR_IRQHandler(void);
void DFSDM1_FLT0_IRQHandler(void);
void DFSDM1_FLT1_IRQHandler(void);
void DFSDM1_FLT2_IRQHandler(void);
void DFSDM1_FLT3_IRQHandler(void);
void SWPMI1_IRQHandler(void);
void TIM15_IRQHandler(void);
void TIM16_IRQHandler(void);
void TIM17_IRQHandler(void);
void MDIOS_WKUP_IRQHandler(void);
void MDIOS_IRQHandler(void);
void MDMA_IRQHandler(void);
void SDMMC2_IRQHandler(void);
void HSEM1_IRQHandler(void);
void ADC3_IRQHandler(void);
void DMAMUX2_OVR_IRQHandler(void);
void BDMA_Channel0_IRQHandler(void);
void BDMA_Channel1_IRQHandler(void);
void BDMA_Channel2_IRQHandler(void);
void BDMA_Channel3_IRQHandler(void);
void BDMA_Channel4_IRQHandler(void);
void BDMA_Channel5_IRQHandler(void);
void BDMA_Channel6_IRQHandler(void);
void BDMA_Channel7_IRQHandler(void);
void COMP_IRQHandler(void);
void LPTIM2_IRQHandler(void);
void LPTIM3_IRQHandler(void);
void LPTIM4_IRQHandler(void);
void LPTIM5_IRQHandler(void);
void LPUART1_IRQHandler(void);
void CRS_IRQHandler(void);
void ECC_IRQHandler(void);
void SAI4_IRQHandler(void);
void DTS_IRQHandler(void);
void WAKEUP_PIN_IRQHandler(void);
void OCTOSPI2_IRQHandler(void);
void FMAC_IRQHandler(void);
void CORDIC_IRQHandler(void);
void UART9_IRQHandler(void);
void USART10_IRQHandler(void);
void I2C5_EV_IRQHandler(void);
void I2C5_ER_IRQHandler(void);
void FDCAN3_IT0_IRQHandler(void);
void FDCAN3_IT1_IRQHandler(void);
void TIM23_IRQHandler(void);
void TIM24_IRQHandler(void);
}

namespace modm::platform::detail
{

constexpr std::string_view vectorNames[] =
{
	"__main_stack_top",
	"Reset",
	"NMI",
	"HardFault",
	"MemManage",
	"BusFault",
	"UsageFault",
	"Undefined",
	"Undefined",
	"Undefined",
	"Undefined",
	"SVC",
	"DebugMon",
	"Undefined",
	"PendSV",
	"SysTick",
	"WWDG",
	"PVD_AVD",
	"TAMP_STAMP",
	"RTC_WKUP",
	"FLASH",
	"RCC",
	"EXTI0",
	"EXTI1",
	"EXTI2",
	"EXTI3",
	"EXTI4",
	"DMA1_Stream0",
	"DMA1_Stream1",
	"DMA1_Stream2",
	"DMA1_Stream3",
	"DMA1_Stream4",
	"DMA1_Stream5",
	"DMA1_Stream6",
	"ADC",
	"FDCAN1_IT0",
	"FDCAN2_IT0",
	"FDCAN1_IT1",
	"FDCAN2_IT1",
	"EXTI9_5",
	"TIM1_BRK",
	"TIM1_UP",
	"TIM1_TRG_COM",
	"TIM1_CC",
	"TIM2",
	"TIM3",
	"TIM4",
	"I2C1_EV",
	"I2C1_ER",
	"I2C2_EV",
	"I2C2_ER",
	"SPI1",
	"SPI2",
	"USART1",
	"USART2",
	"USART3",
	"EXTI15_10",
	"RTC_Alarm",
	"Undefined",
	"TIM8_BRK_TIM12",
	"TIM8_UP_TIM13",
	"TIM8_TRG_COM_TIM14",
	"TIM8_CC",
	"DMA1_Stream7",
	"FMC",
	"SDMMC1",
	"TIM5",
	"SPI3",
	"UART4",
	"UART5",
	"TIM6_DAC",
	"TIM7",
	"DMA2_Stream0",
	"DMA2_Stream1",
	"DMA2_Stream2",
	"DMA2_Stream3",
	"DMA2_Stream4",
	"ETH",
	"ETH_WKUP",
	"FDCAN_CAL",
	"Undefined",
	"Undefined",
	"Undefined",
	"Undefined",
	"DMA2_Stream5",
	"DMA2_Stream6",
	"DMA2_Stream7",
	"USART6",
	"I2C3_EV",
	"I2C3_ER",
	"OTG_HS_EP1_OUT",
	"OTG_HS_EP1_IN",
	"OTG_HS_WKUP",
	"OTG_HS",
	"DCMI_PSSI",
	"Undefined",
	"RNG",
	"FPU",
	"UART7",
	"UART8",
	"SPI4",
	"SPI5",
	"SPI6",
	"SAI1",
	"LTDC",
	"LTDC_ER",
	"DMA2D",
	"Undefined",
	"OCTOSPI1",
	"LPTIM1",
	"CEC",
	"I2C4_EV",
	"I2C4_ER",
	"SPDIF_RX",
	"Undefined",
	"Undefined",
	"Undefined",
	"Undefined",
	"DMAMUX1_OVR",
	"Undefined",
	"Undefined",
	"Undefined",
	"Undefined",
	"Undefined",
	"Undefined",
	"Undefined",
	"DFSDM1_FLT0",
	"DFSDM1_FLT1",
	"DFSDM1_FLT2",
	"DFSDM1_FLT3",
	"Undefined",
	"SWPMI1",
	"TIM15",
	"TIM16",
	"TIM17",
	"MDIOS_WKUP",
	"MDIOS",
	"Undefined",
	"MDMA",
	"Undefined",
	"SDMMC2",
	"HSEM1",
	"Undefined",
	"ADC3",
	"DMAMUX2_OVR",
	"BDMA_Channel0",
	"BDMA_Channel1",
	"BDMA_Channel2",
	"BDMA_Channel3",
	"BDMA_Channel4",
	"BDMA_Channel5",
	"BDMA_Channel6",
	"BDMA_Channel7",
	"COMP",
	"LPTIM2",
	"LPTIM3",
	"LPTIM4",
	"LPTIM5",
	"LPUART1",
	"Undefined",
	"CRS",
	"ECC",
	"SAI4",
	"DTS",
	"Undefined",
	"WAKEUP_PIN",
	"OCTOSPI2",
	"Undefined",
	"Undefined",
	"FMAC",
	"CORDIC",
	"UART9",
	"USART10",
	"I2C5_EV",
	"I2C5_ER",
	"FDCAN3_IT0",
	"FDCAN3_IT1",
	"TIM23",
	"TIM24",
};


#ifndef MODM_ISR_DISABLE_VALIDATION
#define MODM_ISR_VALIDATE(vector_str, vector) \
	static_assert(::modm::platform::detail::validateIrqName(vector_str), \
			"'" vector_str "' is not a valid IRQ name!\n" \
			"  Hint: You do not need to add '_IRQHandler' to the name.\n" \
			"  Hint: Here are all the IRQs on this device:\n" \
			"    - WWDG\n" \
			"    - PVD_AVD\n" \
			"    - TAMP_STAMP\n" \
			"    - RTC_WKUP\n" \
			"    - FLASH\n" \
			"    - RCC\n" \
			"    - EXTI0\n" \
			"    - EXTI1\n" \
			"    - EXTI2\n" \
			"    - EXTI3\n" \
			"    - EXTI4\n" \
			"    - DMA1_Stream0\n" \
			"    - DMA1_Stream1\n" \
			"    - DMA1_Stream2\n" \
			"    - DMA1_Stream3\n" \
			"    - DMA1_Stream4\n" \
			"    - DMA1_Stream5\n" \
			"    - DMA1_Stream6\n" \
			"    - ADC\n" \
			"    - FDCAN1_IT0\n" \
			"    - FDCAN2_IT0\n" \
			"    - FDCAN1_IT1\n" \
			"    - FDCAN2_IT1\n" \
			"    - EXTI9_5\n" \
			"    - TIM1_BRK\n" \
			"    - TIM1_UP\n" \
			"    - TIM1_TRG_COM\n" \
			"    - TIM1_CC\n" \
			"    - TIM2\n" \
			"    - TIM3\n" \
			"    - TIM4\n" \
			"    - I2C1_EV\n" \
			"    - I2C1_ER\n" \
			"    - I2C2_EV\n" \
			"    - I2C2_ER\n" \
			"    - SPI1\n" \
			"    - SPI2\n" \
			"    - USART1\n" \
			"    - USART2\n" \
			"    - USART3\n" \
			"    - EXTI15_10\n" \
			"    - RTC_Alarm\n" \
			"    - TIM8_BRK_TIM12\n" \
			"    - TIM8_UP_TIM13\n" \
			"    - TIM8_TRG_COM_TIM14\n" \
			"    - TIM8_CC\n" \
			"    - DMA1_Stream7\n" \
			"    - FMC\n" \
			"    - SDMMC1\n" \
			"    - TIM5\n" \
			"    - SPI3\n" \
			"    - UART4\n" \
			"    - UART5\n" \
			"    - TIM6_DAC\n" \
			"    - TIM7\n" \
			"    - DMA2_Stream0\n" \
			"    - DMA2_Stream1\n" \
			"    - DMA2_Stream2\n" \
			"    - DMA2_Stream3\n" \
			"    - DMA2_Stream4\n" \
			"    - ETH\n" \
			"    - ETH_WKUP\n" \
			"    - FDCAN_CAL\n" \
			"    - DMA2_Stream5\n" \
			"    - DMA2_Stream6\n" \
			"    - DMA2_Stream7\n" \
			"    - USART6\n" \
			"    - I2C3_EV\n" \
			"    - I2C3_ER\n" \
			"    - OTG_HS_EP1_OUT\n" \
			"    - OTG_HS_EP1_IN\n" \
			"    - OTG_HS_WKUP\n" \
			"    - OTG_HS\n" \
			"    - DCMI_PSSI\n" \
			"    - RNG\n" \
			"    - FPU\n" \
			"    - UART7\n" \
			"    - UART8\n" \
			"    - SPI4\n" \
			"    - SPI5\n" \
			"    - SPI6\n" \
			"    - SAI1\n" \
			"    - LTDC\n" \
			"    - LTDC_ER\n" \
			"    - DMA2D\n" \
			"    - OCTOSPI1\n" \
			"    - LPTIM1\n" \
			"    - CEC\n" \
			"    - I2C4_EV\n" \
			"    - I2C4_ER\n" \
			"    - SPDIF_RX\n" \
			"    - DMAMUX1_OVR\n" \
			"    - DFSDM1_FLT0\n" \
			"    - DFSDM1_FLT1\n" \
			"    - DFSDM1_FLT2\n" \
			"    - DFSDM1_FLT3\n" \
			"    - SWPMI1\n" \
			"    - TIM15\n" \
			"    - TIM16\n" \
			"    - TIM17\n" \
			"    - MDIOS_WKUP\n" \
			"    - MDIOS\n" \
			"    - MDMA\n" \
			"    - SDMMC2\n" \
			"    - HSEM1\n" \
			"    - ADC3\n" \
			"    - DMAMUX2_OVR\n" \
			"    - BDMA_Channel0\n" \
			"    - BDMA_Channel1\n" \
			"    - BDMA_Channel2\n" \
			"    - BDMA_Channel3\n" \
			"    - BDMA_Channel4\n" \
			"    - BDMA_Channel5\n" \
			"    - BDMA_Channel6\n" \
			"    - BDMA_Channel7\n" \
			"    - COMP\n" \
			"    - LPTIM2\n" \
			"    - LPTIM3\n" \
			"    - LPTIM4\n" \
			"    - LPTIM5\n" \
			"    - LPUART1\n" \
			"    - CRS\n" \
			"    - ECC\n" \
			"    - SAI4\n" \
			"    - DTS\n" \
			"    - WAKEUP_PIN\n" \
			"    - OCTOSPI2\n" \
			"    - FMAC\n" \
			"    - CORDIC\n" \
			"    - UART9\n" \
			"    - USART10\n" \
			"    - I2C5_EV\n" \
			"    - I2C5_ER\n" \
			"    - FDCAN3_IT0\n" \
			"    - FDCAN3_IT1\n" \
			"    - TIM23\n" \
			"    - TIM24\n" \
	)
#else
#define MODM_ISR_VALIDATE(...)
#endif

constexpr int getIrqPosition(std::string_view name)
{
	for (int pos = 0; pos < 179; pos++)
		if (vectorNames[pos] == name) return pos;
	return -1;
}

constexpr bool validateIrqName(std::string_view name)
{
	return getIrqPosition(name) != -1;
}

}	// namespace modm::platform::detail