/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright (c) 2016, Toradex AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#if !defined(__BOARD_H__)
#define __BOARD_H__

#include "pin_mux.h"
#include "rdc.h"
#include "rdc_defs_imx7d.h"
#include "ccm_imx7d.h"
#include "clock_freq.h"

/* The board name */
#define BOARD_NAME                            "IMX7_COLIBRI_M4"
#define BOARD_DOMAIN_ID                       (1)

/* PWM information for this board */
#define BOARD_PWM1_RDC_PDAP				      rdcPdapPwm1
#define BOARD_PWM1_CCM_CCGR				      ccmCcgrGatePwm1
#define BOARD_PWM1_BASEADDR					  PWM1
#define BOARD_PWM1_IRQ_NUM					  PWM1_IRQn
#define	BOARD_PWM1_HANDLER					  PWM1_Handler

#define BOARD_PWM2_RDC_PDAP				      rdcPdapPwm2
#define BOARD_PWM2_CCM_CCGR				      ccmCcgrGatePwm2
#define BOARD_PWM2_BASEADDR					  PWM2
#define BOARD_PWM2_IRQ_NUM					  PWM2_IRQn
#define	BOARD_PWM2_HANDLER					  PWM2_Handler

#define BOARD_PWM3_RDC_PDAP				      rdcPdapPwm3
#define BOARD_PWM3_CCM_CCGR				      ccmCcgrGatePwm3
#define BOARD_PWM3_BASEADDR					  PWM3
#define BOARD_PWM3_IRQ_NUM					  PWM3_IRQn
#define	BOARD_PWM3_HANDLER					  PWM3_Handler

#define BOARD_PWM4_RDC_PDAP				      rdcPdapPwm4
#define BOARD_PWM4_CCM_CCGR				      ccmCcgrGatePwm4
#define BOARD_PWM4_BASEADDR					  PWM4
#define BOARD_PWM4_IRQ_NUM					  PWM4_IRQn
#define	BOARD_PWM4_HANDLER					  PWM4_Handler


/* ADC information for this board */
#define BOARD_ADC_RDC_PDAP                    rdcPdapAdc1
#define BOARD_ADC_CCM_CCGR                    ccmCcgrGateAdc
#define BOARD_ADC_BASEADDR                    ADC1
#define BOARD_ADC_IRQ_NUM                     ADC1_IRQn
#define BOARD_ADC_HANDLER                     ADC1_Handler
#define BOARD_ADC_INPUT_CHANNEL               (3)

/* WDOG information for this board */
#define BOARD_WDOG_RDC_PDAP                   rdcPdapWdog3
#define BOARD_WDOG_CCM_ROOT                   ccmRootWdog
#define BOARD_WDOG_CCM_CCGR                   ccmCcgrGateWdog3
#define BOARD_WDOG_BASEADDR                   WDOG3
#define BOARD_WDOG_IRQ_NUM                    WDOG3_IRQn
#define BOARD_WDOG_HANDLER                    WDOG3_Handler

/* SEMA4 information for this board */
#define BOARD_SEMA4_RDC_PDAP                  rdcPdapSemaphoreHs
#define BOARD_SEMA4_CCM_CCGR                  ccmCcgrGateSemaHs
#define BOARD_SEMA4_BASEADDR                  SEMA4
#define BOARD_SEMA4_IRQ_NUM                   SEMA4_HS_M4_IRQn
#define BOARD_SEMA4_HANDLER                   SEMA4_HS_M4_Handler

/* GPT instance A information for this board */
#define BOARD_GPTA_RDC_PDAP                   rdcPdapGpt3
#define BOARD_GPTA_CCM_ROOT                   ccmRootGpt3
#define BOARD_GPTA_CCM_CCGR                   ccmCcgrGateGpt3
#define BOARD_GPTA_BASEADDR                   GPT3
#define BOARD_GPTA_IRQ_NUM                    GPT3_IRQn
#define BOARD_GPTA_HANDLER                    GPT3_Handler
/* GPT instance B information for this board */
#define BOARD_GPTB_RDC_PDAP                   rdcPdapGpt4
#define BOARD_GPTB_CCM_ROOT                   ccmRootGpt4
#define BOARD_GPTB_CCM_CCGR                   ccmCcgrGateGpt4
#define BOARD_GPTB_BASEADDR                   GPT4
#define BOARD_GPTB_IRQ_NUM                    GPT4_IRQn
#define BOARD_GPTB_HANDLER                    GPT4_Handler

/* GPIO information for this board */
#define BOARD_GPIO_KEY_CCM_CCGR               ccmCcgrGateGpio2
#define BOARD_GPIO_LED_RDC_PDAP               rdcPdapGpio1
#define BOARD_GPIO_LED_CONFIG                 (&gpioLed)
#define BOARD_GPIO_KEY_RDC_PDAP               rdcPdapGpio2
#define BOARD_GPIO_KEY_CONFIG                 (&gpioKeyFunc1)
#define BOARD_GPIO_KEY_IRQ_NUM                GPIO2_INT31_16_IRQn
#define BOARD_GPIO_KEY_HANDLER                GPIO2_INT31_16_Handler

/* Debug UART information for this board */
#define BOARD_DEBUG_UART_RDC_PDAP             rdcPdapUart2
#define BOARD_DEBUG_UART_CCM_ROOT             ccmRootUart2
#define BOARD_DEBUG_UART_CCM_CCGR             ccmCcgrGateUart2
#define BOARD_DEBUG_UART_BASEADDR             UART2
#define BOARD_DEBUG_UART_IRQ_NUM              UART2_IRQn
#define BOARD_DEBUG_UART_HANDLER              UART2_Handler

/* MU information for this board*/
#define BOARD_MU_HANDLER                      MU_M4_Handler
#define BOARD_MU_IRQ_NUM                      MU_M4_IRQn
#define BOARD_MU_BASE_ADDR                    MUB
#define BOARD_MU_CCM_CCGR                     ccmCcgrGateMu
#define BOARD_MU_RDC_PDAP                     rdcPdapMuB

/* I2C information for this board */
#define BOARD_I2C_RDC_PDAP                    rdcPdapI2c4
#define BOARD_I2C_CCM_ROOT                    ccmRootI2c4
#define BOARD_I2C_CCM_CCGR                    ccmCcgrGateI2c4
#define BOARD_I2C_BASEADDR                    I2C4
#define BOARD_I2C_IRQ_NUM                     I2C4_IRQn
#define BOARD_I2C_HANDLER                     I2C4_Handler
#define BOARD_I2C_FXAS21002_ADDR              (0x20)
#define BOARD_I2C_FXOS8700_ADDR               (0x1E)

/* FlexCAN information for this board */
#define BOARD_FLEXCAN_RDC_PDAP                rdcPdapFlexCan2
#define BOARD_FLEXCAN_CCM_ROOT                ccmRootCan2
#define BOARD_FLEXCAN_CCM_CCGR                ccmCcgrGateCan2
#define BOARD_FLEXCAN_BASEADDR                CAN2
#define BOARD_FLEXCAN_IRQ_NUM                 FLEXCAN2_IRQn
#define BOARD_FLEXCAN_HANDLER                 FLEXCAN2_Handler

/* GPC information for this board*/
#define BOARD_GPC_BASEADDR                    GPC

/* SIM_WAKEUP CG information*/
#define BOARD_SIM_WAKEUP_CCGR                 ccmCcgrGateSimWakeup

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

void hardware_init(void);
void dbg_uart_init(void);
/* Function to initialize clock base on board configuration. */
void BOARD_ClockInit(void);
void BOARD_RdcInit(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* __BOARD_H__ */
