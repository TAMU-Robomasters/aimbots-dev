/*
 * Copyright (c) 2018, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <stdint.h>
#include <modm/architecture/utils.hpp>

// ----------------------------------------------------------------------------
void Undefined_Handler(void);
/* Provide weak aliases for each Exception handler to Undefined_Handler.
 * As they are weak aliases, any function with the same name will override
 * this definition. */
void Reset_Handler(void);
void NMI_Handler(void)							__attribute__((weak, alias("Undefined_Handler")));
void HardFault_Handler(void)					__attribute__((weak, alias("Undefined_Handler")));
void MemManage_Handler(void)					__attribute__((weak, alias("Undefined_Handler")));
void BusFault_Handler(void)						__attribute__((weak, alias("Undefined_Handler")));
void UsageFault_Handler(void)					__attribute__((weak, alias("Undefined_Handler")));
void SVC_Handler(void)							__attribute__((weak, alias("Undefined_Handler")));
void DebugMon_Handler(void)						__attribute__((weak, alias("Undefined_Handler")));
void PendSV_Handler(void)						__attribute__((weak, alias("Undefined_Handler")));
void SysTick_Handler(void)						__attribute__((weak, alias("Undefined_Handler")));
void WWDG_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void PVD_AVD_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void TAMP_STAMP_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void RTC_WKUP_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void FLASH_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void RCC_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void EXTI0_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void EXTI1_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void EXTI2_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void EXTI3_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void EXTI4_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void DMA1_Stream0_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void DMA1_Stream1_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void DMA1_Stream2_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void DMA1_Stream3_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void DMA1_Stream4_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void DMA1_Stream5_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void DMA1_Stream6_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void ADC_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void FDCAN1_IT0_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void FDCAN2_IT0_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void FDCAN1_IT1_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void FDCAN2_IT1_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void EXTI9_5_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void TIM1_BRK_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void TIM1_UP_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void TIM1_TRG_COM_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void TIM1_CC_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void TIM2_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void TIM3_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void TIM4_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void I2C1_EV_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void I2C1_ER_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void I2C2_EV_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void I2C2_ER_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void SPI1_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void SPI2_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void USART1_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void USART2_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void USART3_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void EXTI15_10_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void RTC_Alarm_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void TIM8_BRK_TIM12_IRQHandler(void)			__attribute__((weak, alias("Undefined_Handler")));
void TIM8_UP_TIM13_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void TIM8_TRG_COM_TIM14_IRQHandler(void)		__attribute__((weak, alias("Undefined_Handler")));
void TIM8_CC_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void DMA1_Stream7_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void FMC_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void SDMMC1_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void TIM5_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void SPI3_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void UART4_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void UART5_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void TIM6_DAC_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void TIM7_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void DMA2_Stream0_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void DMA2_Stream1_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void DMA2_Stream2_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void DMA2_Stream3_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void DMA2_Stream4_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void ETH_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void ETH_WKUP_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void FDCAN_CAL_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void DMA2_Stream5_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void DMA2_Stream6_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void DMA2_Stream7_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void USART6_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void I2C3_EV_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void I2C3_ER_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void OTG_HS_EP1_OUT_IRQHandler(void)			__attribute__((weak, alias("Undefined_Handler")));
void OTG_HS_EP1_IN_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void OTG_HS_WKUP_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void OTG_HS_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void DCMI_PSSI_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void RNG_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void FPU_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void UART7_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void UART8_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void SPI4_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void SPI5_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void SPI6_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void SAI1_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void LTDC_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void LTDC_ER_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void DMA2D_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void OCTOSPI1_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void LPTIM1_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void CEC_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void I2C4_EV_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void I2C4_ER_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void SPDIF_RX_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void DMAMUX1_OVR_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void DFSDM1_FLT0_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void DFSDM1_FLT1_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void DFSDM1_FLT2_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void DFSDM1_FLT3_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void SWPMI1_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void TIM15_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void TIM16_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void TIM17_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void MDIOS_WKUP_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void MDIOS_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void MDMA_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void SDMMC2_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void HSEM1_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void ADC3_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void DMAMUX2_OVR_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void BDMA_Channel0_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void BDMA_Channel1_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void BDMA_Channel2_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void BDMA_Channel3_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void BDMA_Channel4_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void BDMA_Channel5_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void BDMA_Channel6_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void BDMA_Channel7_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void COMP_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void LPTIM2_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void LPTIM3_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void LPTIM4_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void LPTIM5_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void LPUART1_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void CRS_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void ECC_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void SAI4_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void DTS_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void WAKEUP_PIN_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void OCTOSPI2_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void FMAC_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void CORDIC_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void UART9_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void USART10_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void I2C5_EV_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void I2C5_ER_IRQHandler(void)					__attribute__((weak, alias("Undefined_Handler")));
void FDCAN3_IT0_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void FDCAN3_IT1_IRQHandler(void)				__attribute__((weak, alias("Undefined_Handler")));
void TIM23_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));
void TIM24_IRQHandler(void)						__attribute__((weak, alias("Undefined_Handler")));

// ----------------------------------------------------------------------------
typedef void (* const FunctionPointer)(void);

extern uint32_t __main_stack_top[]; // from linkerscript

// Define the vector table
modm_section(".vector_rom")
FunctionPointer vectorsRom[] =
{
	(FunctionPointer)__main_stack_top,		// -16: stack pointer
	Reset_Handler,							// -15: code entry point
	NMI_Handler,							// -14: Non Maskable Interrupt handler
	HardFault_Handler,						// -13: hard fault handler
	MemManage_Handler,						// -12
	BusFault_Handler,						// -11
	UsageFault_Handler,						// -10
	Undefined_Handler,						//  -9
	Undefined_Handler,						//  -8
	Undefined_Handler,						//  -7
	Undefined_Handler,						//  -6
	SVC_Handler,							//  -5
	DebugMon_Handler,						//  -4
	Undefined_Handler,						//  -3
	PendSV_Handler,							//  -2
	SysTick_Handler,						//  -1
	WWDG_IRQHandler,						//   0
	PVD_AVD_IRQHandler,						//   1
	TAMP_STAMP_IRQHandler,					//   2
	RTC_WKUP_IRQHandler,					//   3
	FLASH_IRQHandler,						//   4
	RCC_IRQHandler,							//   5
	EXTI0_IRQHandler,						//   6
	EXTI1_IRQHandler,						//   7
	EXTI2_IRQHandler,						//   8
	EXTI3_IRQHandler,						//   9
	EXTI4_IRQHandler,						//  10
	DMA1_Stream0_IRQHandler,				//  11
	DMA1_Stream1_IRQHandler,				//  12
	DMA1_Stream2_IRQHandler,				//  13
	DMA1_Stream3_IRQHandler,				//  14
	DMA1_Stream4_IRQHandler,				//  15
	DMA1_Stream5_IRQHandler,				//  16
	DMA1_Stream6_IRQHandler,				//  17
	ADC_IRQHandler,							//  18
	FDCAN1_IT0_IRQHandler,					//  19
	FDCAN2_IT0_IRQHandler,					//  20
	FDCAN1_IT1_IRQHandler,					//  21
	FDCAN2_IT1_IRQHandler,					//  22
	EXTI9_5_IRQHandler,						//  23
	TIM1_BRK_IRQHandler,					//  24
	TIM1_UP_IRQHandler,						//  25
	TIM1_TRG_COM_IRQHandler,				//  26
	TIM1_CC_IRQHandler,						//  27
	TIM2_IRQHandler,						//  28
	TIM3_IRQHandler,						//  29
	TIM4_IRQHandler,						//  30
	I2C1_EV_IRQHandler,						//  31
	I2C1_ER_IRQHandler,						//  32
	I2C2_EV_IRQHandler,						//  33
	I2C2_ER_IRQHandler,						//  34
	SPI1_IRQHandler,						//  35
	SPI2_IRQHandler,						//  36
	USART1_IRQHandler,						//  37
	USART2_IRQHandler,						//  38
	USART3_IRQHandler,						//  39
	EXTI15_10_IRQHandler,					//  40
	RTC_Alarm_IRQHandler,					//  41
	Undefined_Handler,						//  42
	TIM8_BRK_TIM12_IRQHandler,				//  43
	TIM8_UP_TIM13_IRQHandler,				//  44
	TIM8_TRG_COM_TIM14_IRQHandler,			//  45
	TIM8_CC_IRQHandler,						//  46
	DMA1_Stream7_IRQHandler,				//  47
	FMC_IRQHandler,							//  48
	SDMMC1_IRQHandler,						//  49
	TIM5_IRQHandler,						//  50
	SPI3_IRQHandler,						//  51
	UART4_IRQHandler,						//  52
	UART5_IRQHandler,						//  53
	TIM6_DAC_IRQHandler,					//  54
	TIM7_IRQHandler,						//  55
	DMA2_Stream0_IRQHandler,				//  56
	DMA2_Stream1_IRQHandler,				//  57
	DMA2_Stream2_IRQHandler,				//  58
	DMA2_Stream3_IRQHandler,				//  59
	DMA2_Stream4_IRQHandler,				//  60
	ETH_IRQHandler,							//  61
	ETH_WKUP_IRQHandler,					//  62
	FDCAN_CAL_IRQHandler,					//  63
	Undefined_Handler,						//  64
	Undefined_Handler,						//  65
	Undefined_Handler,						//  66
	Undefined_Handler,						//  67
	DMA2_Stream5_IRQHandler,				//  68
	DMA2_Stream6_IRQHandler,				//  69
	DMA2_Stream7_IRQHandler,				//  70
	USART6_IRQHandler,						//  71
	I2C3_EV_IRQHandler,						//  72
	I2C3_ER_IRQHandler,						//  73
	OTG_HS_EP1_OUT_IRQHandler,				//  74
	OTG_HS_EP1_IN_IRQHandler,				//  75
	OTG_HS_WKUP_IRQHandler,					//  76
	OTG_HS_IRQHandler,						//  77
	DCMI_PSSI_IRQHandler,					//  78
	Undefined_Handler,						//  79
	RNG_IRQHandler,							//  80
	FPU_IRQHandler,							//  81
	UART7_IRQHandler,						//  82
	UART8_IRQHandler,						//  83
	SPI4_IRQHandler,						//  84
	SPI5_IRQHandler,						//  85
	SPI6_IRQHandler,						//  86
	SAI1_IRQHandler,						//  87
	LTDC_IRQHandler,						//  88
	LTDC_ER_IRQHandler,						//  89
	DMA2D_IRQHandler,						//  90
	Undefined_Handler,						//  91
	OCTOSPI1_IRQHandler,					//  92
	LPTIM1_IRQHandler,						//  93
	CEC_IRQHandler,							//  94
	I2C4_EV_IRQHandler,						//  95
	I2C4_ER_IRQHandler,						//  96
	SPDIF_RX_IRQHandler,					//  97
	Undefined_Handler,						//  98
	Undefined_Handler,						//  99
	Undefined_Handler,						// 100
	Undefined_Handler,						// 101
	DMAMUX1_OVR_IRQHandler,					// 102
	Undefined_Handler,						// 103
	Undefined_Handler,						// 104
	Undefined_Handler,						// 105
	Undefined_Handler,						// 106
	Undefined_Handler,						// 107
	Undefined_Handler,						// 108
	Undefined_Handler,						// 109
	DFSDM1_FLT0_IRQHandler,					// 110
	DFSDM1_FLT1_IRQHandler,					// 111
	DFSDM1_FLT2_IRQHandler,					// 112
	DFSDM1_FLT3_IRQHandler,					// 113
	Undefined_Handler,						// 114
	SWPMI1_IRQHandler,						// 115
	TIM15_IRQHandler,						// 116
	TIM16_IRQHandler,						// 117
	TIM17_IRQHandler,						// 118
	MDIOS_WKUP_IRQHandler,					// 119
	MDIOS_IRQHandler,						// 120
	Undefined_Handler,						// 121
	MDMA_IRQHandler,						// 122
	Undefined_Handler,						// 123
	SDMMC2_IRQHandler,						// 124
	HSEM1_IRQHandler,						// 125
	Undefined_Handler,						// 126
	ADC3_IRQHandler,						// 127
	DMAMUX2_OVR_IRQHandler,					// 128
	BDMA_Channel0_IRQHandler,				// 129
	BDMA_Channel1_IRQHandler,				// 130
	BDMA_Channel2_IRQHandler,				// 131
	BDMA_Channel3_IRQHandler,				// 132
	BDMA_Channel4_IRQHandler,				// 133
	BDMA_Channel5_IRQHandler,				// 134
	BDMA_Channel6_IRQHandler,				// 135
	BDMA_Channel7_IRQHandler,				// 136
	COMP_IRQHandler,						// 137
	LPTIM2_IRQHandler,						// 138
	LPTIM3_IRQHandler,						// 139
	LPTIM4_IRQHandler,						// 140
	LPTIM5_IRQHandler,						// 141
	LPUART1_IRQHandler,						// 142
	Undefined_Handler,						// 143
	CRS_IRQHandler,							// 144
	ECC_IRQHandler,							// 145
	SAI4_IRQHandler,						// 146
	DTS_IRQHandler,							// 147
	Undefined_Handler,						// 148
	WAKEUP_PIN_IRQHandler,					// 149
	OCTOSPI2_IRQHandler,					// 150
	Undefined_Handler,						// 151
	Undefined_Handler,						// 152
	FMAC_IRQHandler,						// 153
	CORDIC_IRQHandler,						// 154
	UART9_IRQHandler,						// 155
	USART10_IRQHandler,						// 156
	I2C5_EV_IRQHandler,						// 157
	I2C5_ER_IRQHandler,						// 158
	FDCAN3_IT0_IRQHandler,					// 159
	FDCAN3_IT1_IRQHandler,					// 160
	TIM23_IRQHandler,						// 161
	TIM24_IRQHandler,						// 162
};
// reserve space for the remapped vector table in RAM
modm_section(".vector_ram")
FunctionPointer vectorsRam[sizeof(vectorsRom) / sizeof(FunctionPointer)];

// ----------------------------------------------------------------------------
// Ignore redeclaration of interrupt handlers in vendor headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wredundant-decls"
// Explicitly include this BELOW the vector table to *not deal* with potential
// re-#defines of interrupt vector names! Bad vendors!! BAD!!!
#include <modm/platform/device.hpp>
#include <modm/architecture/interface/assert.h>

void Undefined_Handler(void)
{
	const int32_t irqn = ((int32_t)__get_IPSR()) - 16;
	// Set the currently executing interrupt to the lowest priority to allow
	// reporting of the assertion failure and disable it from firing again.
	NVIC_SetPriority((IRQn_Type)irqn, (1ul << __NVIC_PRIO_BITS) - 1ul);
	NVIC_DisableIRQ((IRQn_Type)irqn);
	modm_assert_continue_fail(0, "nvic.undef",
			"An undefined NVIC interrupt was raised!", irqn);
}

#pragma GCC diagnostic pop