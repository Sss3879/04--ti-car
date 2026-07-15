/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000




/* Defines for OLED */
#define OLED_INST                                                           I2C1
#define OLED_INST_IRQHandler                                     I2C1_IRQHandler
#define OLED_INST_INT_IRQN                                         I2C1_INT_IRQn
#define OLED_BUS_SPEED_HZ                                                 100000
#define GPIO_OLED_SDA_PORT                                                 GPIOB
#define GPIO_OLED_SDA_PIN                                          DL_GPIO_PIN_3
#define GPIO_OLED_IOMUX_SDA                                      (IOMUX_PINCM16)
#define GPIO_OLED_IOMUX_SDA_FUNC                       IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_OLED_SCL_PORT                                                 GPIOB
#define GPIO_OLED_SCL_PIN                                          DL_GPIO_PIN_2
#define GPIO_OLED_IOMUX_SCL                                      (IOMUX_PINCM15)
#define GPIO_OLED_IOMUX_SCL_FUNC                       IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for Serial */
#define Serial_INST                                                        UART0
#define Serial_INST_FREQUENCY                                           32000000
#define Serial_INST_IRQHandler                                  UART0_IRQHandler
#define Serial_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_Serial_RX_PORT                                                GPIOA
#define GPIO_Serial_TX_PORT                                                GPIOA
#define GPIO_Serial_RX_PIN                                        DL_GPIO_PIN_31
#define GPIO_Serial_TX_PIN                                        DL_GPIO_PIN_28
#define GPIO_Serial_IOMUX_RX                                      (IOMUX_PINCM6)
#define GPIO_Serial_IOMUX_TX                                      (IOMUX_PINCM3)
#define GPIO_Serial_IOMUX_RX_FUNC                       IOMUX_PINCM6_PF_UART0_RX
#define GPIO_Serial_IOMUX_TX_FUNC                       IOMUX_PINCM3_PF_UART0_TX
#define Serial_BAUD_RATE                                                (115200)
#define Serial_IBRD_32_MHZ_115200_BAUD                                      (17)
#define Serial_FBRD_32_MHZ_115200_BAUD                                      (23)





/* Defines for POT */
#define POT_INST                                                            ADC1
#define POT_INST_IRQHandler                                      ADC1_IRQHandler
#define POT_INST_INT_IRQN                                        (ADC1_INT_IRQn)
#define POT_ADCMEM_0                                          DL_ADC12_MEM_IDX_0
#define POT_ADCMEM_0_REF                       DL_ADC12_REFERENCE_VOLTAGE_INTREF
#define POT_ADCMEM_0_REF_VOLTAGE_V                                          1.40
#define GPIO_POT_C1_PORT                                                   GPIOA
#define GPIO_POT_C1_PIN                                           DL_GPIO_PIN_16
#define GPIO_POT_IOMUX_C1                                        (IOMUX_PINCM38)
#define GPIO_POT_IOMUX_C1_FUNC                    (IOMUX_PINCM38_PF_UNCONNECTED)


/* Defines for VREF */
#define VREF_VOLTAGE_MV                                                     1400




/* Port definition for Pin Group Key */
#define Key_PORT                                                         (GPIOA)

/* Defines for Key1: GPIOA.24 with pinCMx 54 on package pin 25 */
// pins affected by this interrupt request:["Key1"]
#define Key_INT_IRQN                                            (GPIOA_INT_IRQn)
#define Key_INT_IIDX                            (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define Key_Key1_IIDX                                       (DL_GPIO_IIDX_DIO24)
#define Key_Key1_PIN                                            (DL_GPIO_PIN_24)
#define Key_Key1_IOMUX                                           (IOMUX_PINCM54)
/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOA)

/* Defines for LED1: GPIOA.15 with pinCMx 37 on package pin 8 */
#define LED_LED1_PIN                                            (DL_GPIO_PIN_15)
#define LED_LED1_IOMUX                                           (IOMUX_PINCM37)
/* Defines for LED0: GPIOA.14 with pinCMx 36 on package pin 7 */
#define LED_LED0_PIN                                            (DL_GPIO_PIN_14)
#define LED_LED0_IOMUX                                           (IOMUX_PINCM36)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_OLED_init(void);
void SYSCFG_DL_Serial_init(void);
void SYSCFG_DL_POT_init(void);
void SYSCFG_DL_VREF_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
