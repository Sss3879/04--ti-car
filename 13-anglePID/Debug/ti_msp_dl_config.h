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



#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2500
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for PWM1 */
#define PWM1_INST                                                          TIMG0
#define PWM1_INST_IRQHandler                                    TIMG0_IRQHandler
#define PWM1_INST_INT_IRQN                                      (TIMG0_INT_IRQn)
#define PWM1_INST_CLK_FREQ                                              40000000
/* GPIO defines for channel 0 */
#define GPIO_PWM1_C0_PORT                                                  GPIOA
#define GPIO_PWM1_C0_PIN                                          DL_GPIO_PIN_12
#define GPIO_PWM1_C0_IOMUX                                       (IOMUX_PINCM34)
#define GPIO_PWM1_C0_IOMUX_FUNC                      IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_PWM1_C0_IDX                                     DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM1_C1_PORT                                                  GPIOA
#define GPIO_PWM1_C1_PIN                                          DL_GPIO_PIN_13
#define GPIO_PWM1_C1_IOMUX                                       (IOMUX_PINCM35)
#define GPIO_PWM1_C1_IOMUX_FUNC                      IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_PWM1_C1_IDX                                     DL_TIMER_CC_1_INDEX



/* Defines for MOTOR_PID */
#define MOTOR_PID_INST                                                   (TIMA0)
#define MOTOR_PID_INST_IRQHandler                               TIMA0_IRQHandler
#define MOTOR_PID_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define MOTOR_PID_INST_LOAD_VALUE                                       (39999U)
/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG6)
#define TIMER_0_INST_IRQHandler                                 TIMG6_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG6_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                          (7999U)




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

/* Defines for I2C_0 */
#define I2C_0_INST                                                          I2C0
#define I2C_0_INST_IRQHandler                                    I2C0_IRQHandler
#define I2C_0_INST_INT_IRQN                                        I2C0_INT_IRQn
#define I2C_0_BUS_SPEED_HZ                                                100000
#define GPIO_I2C_0_SDA_PORT                                                GPIOA
#define GPIO_I2C_0_SDA_PIN                                         DL_GPIO_PIN_0
#define GPIO_I2C_0_IOMUX_SDA                                      (IOMUX_PINCM1)
#define GPIO_I2C_0_IOMUX_SDA_FUNC                       IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_I2C_0_SCL_PORT                                                GPIOA
#define GPIO_I2C_0_SCL_PIN                                         DL_GPIO_PIN_1
#define GPIO_I2C_0_IOMUX_SCL                                      (IOMUX_PINCM2)
#define GPIO_I2C_0_IOMUX_SCL_FUNC                       IOMUX_PINCM2_PF_I2C0_SCL


/* Defines for Serial */
#define Serial_INST                                                        UART0
#define Serial_INST_FREQUENCY                                           40000000
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
#define Serial_IBRD_40_MHZ_115200_BAUD                                      (21)
#define Serial_FBRD_40_MHZ_115200_BAUD                                      (45)





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

/* Defines for ADC12_0 */
#define ADC12_0_INST                                                        ADC0
#define ADC12_0_INST_IRQHandler                                  ADC0_IRQHandler
#define ADC12_0_INST_INT_IRQN                                    (ADC0_INT_IRQn)
#define ADC12_0_ADCMEM_ADC_CH0                                DL_ADC12_MEM_IDX_0
#define ADC12_0_ADCMEM_ADC_CH0_REF               DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC12_0_ADCMEM_ADC_CH0_REF_VOLTAGE_V                                     3.3
#define GPIO_ADC12_0_C0_PORT                                               GPIOA
#define GPIO_ADC12_0_C0_PIN                                       DL_GPIO_PIN_27
#define GPIO_ADC12_0_IOMUX_C0                                    (IOMUX_PINCM60)
#define GPIO_ADC12_0_IOMUX_C0_FUNC                (IOMUX_PINCM60_PF_UNCONNECTED)


/* Defines for VREF */
#define VREF_VOLTAGE_MV                                                     1400




/* Defines for DMA_CH0 */
#define DMA_CH0_CHAN_ID                                                      (0)
#define ADC12_0_INST_DMA_TRIGGER                      (DMA_ADC0_EVT_GEN_BD_TRIG)


/* Port definition for Pin Group Gray_Addr_A */
#define Gray_Addr_A_PORT                                                 (GPIOA)

/* Defines for ADDR2: GPIOA.17 with pinCMx 39 on package pin 10 */
#define Gray_Addr_A_ADDR2_PIN                                   (DL_GPIO_PIN_17)
#define Gray_Addr_A_ADDR2_IOMUX                                  (IOMUX_PINCM39)
/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOA)

/* Defines for LED1: GPIOA.15 with pinCMx 37 on package pin 8 */
#define LED_LED1_PIN                                            (DL_GPIO_PIN_15)
#define LED_LED1_IOMUX                                           (IOMUX_PINCM37)
/* Defines for LED0: GPIOA.14 with pinCMx 36 on package pin 7 */
#define LED_LED0_PIN                                            (DL_GPIO_PIN_14)
#define LED_LED0_IOMUX                                           (IOMUX_PINCM36)
/* Port definition for Pin Group Key */
#define Key_PORT                                                         (GPIOB)

/* Defines for Key1: GPIOB.6 with pinCMx 23 on package pin 58 */
// groups represented: ["Motor","Key"]
// pins affected: ["E2A","Key1","Key2"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define Key_Key1_IIDX                                        (DL_GPIO_IIDX_DIO6)
#define Key_Key1_PIN                                             (DL_GPIO_PIN_6)
#define Key_Key1_IOMUX                                           (IOMUX_PINCM23)
/* Defines for Key2: GPIOB.7 with pinCMx 24 on package pin 59 */
#define Key_Key2_IIDX                                        (DL_GPIO_IIDX_DIO7)
#define Key_Key2_PIN                                             (DL_GPIO_PIN_7)
#define Key_Key2_IOMUX                                           (IOMUX_PINCM24)
/* Defines for AIN1: GPIOA.8 with pinCMx 19 on package pin 54 */
#define Motor_AIN1_PORT                                                  (GPIOA)
#define Motor_AIN1_PIN                                           (DL_GPIO_PIN_8)
#define Motor_AIN1_IOMUX                                         (IOMUX_PINCM19)
/* Defines for AIN2: GPIOA.9 with pinCMx 20 on package pin 55 */
#define Motor_AIN2_PORT                                                  (GPIOA)
#define Motor_AIN2_PIN                                           (DL_GPIO_PIN_9)
#define Motor_AIN2_IOMUX                                         (IOMUX_PINCM20)
/* Defines for STBY: GPIOB.24 with pinCMx 52 on package pin 23 */
#define Motor_STBY_PORT                                                  (GPIOB)
#define Motor_STBY_PIN                                          (DL_GPIO_PIN_24)
#define Motor_STBY_IOMUX                                         (IOMUX_PINCM52)
/* Defines for BIN1: GPIOB.18 with pinCMx 44 on package pin 15 */
#define Motor_BIN1_PORT                                                  (GPIOB)
#define Motor_BIN1_PIN                                          (DL_GPIO_PIN_18)
#define Motor_BIN1_IOMUX                                         (IOMUX_PINCM44)
/* Defines for BIN2: GPIOA.7 with pinCMx 14 on package pin 49 */
#define Motor_BIN2_PORT                                                  (GPIOA)
#define Motor_BIN2_PIN                                           (DL_GPIO_PIN_7)
#define Motor_BIN2_IOMUX                                         (IOMUX_PINCM14)
/* Defines for E1A: GPIOA.21 with pinCMx 46 on package pin 17 */
#define Motor_E1A_PORT                                                   (GPIOA)
// pins affected by this interrupt request:["E1A"]
#define Motor_GPIOA_INT_IRQN                                    (GPIOA_INT_IRQn)
#define Motor_GPIOA_INT_IIDX                    (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define Motor_E1A_IIDX                                      (DL_GPIO_IIDX_DIO21)
#define Motor_E1A_PIN                                           (DL_GPIO_PIN_21)
#define Motor_E1A_IOMUX                                          (IOMUX_PINCM46)
/* Defines for E1B: GPIOA.22 with pinCMx 47 on package pin 18 */
#define Motor_E1B_PORT                                                   (GPIOA)
#define Motor_E1B_PIN                                           (DL_GPIO_PIN_22)
#define Motor_E1B_IOMUX                                          (IOMUX_PINCM47)
/* Defines for E2A: GPIOB.19 with pinCMx 45 on package pin 16 */
#define Motor_E2A_PORT                                                   (GPIOB)
#define Motor_E2A_IIDX                                      (DL_GPIO_IIDX_DIO19)
#define Motor_E2A_PIN                                           (DL_GPIO_PIN_19)
#define Motor_E2A_IOMUX                                          (IOMUX_PINCM45)
/* Defines for E2B: GPIOB.20 with pinCMx 48 on package pin 19 */
#define Motor_E2B_PORT                                                   (GPIOB)
#define Motor_E2B_PIN                                           (DL_GPIO_PIN_20)
#define Motor_E2B_IOMUX                                          (IOMUX_PINCM48)
/* Port definition for Pin Group Gray_Addr_B */
#define Gray_Addr_B_PORT                                                 (GPIOB)

/* Defines for ADDR0: GPIOB.8 with pinCMx 25 on package pin 60 */
#define Gray_Addr_B_ADDR0_PIN                                    (DL_GPIO_PIN_8)
#define Gray_Addr_B_ADDR0_IOMUX                                  (IOMUX_PINCM25)
/* Defines for ADDR1: GPIOB.9 with pinCMx 26 on package pin 61 */
#define Gray_Addr_B_ADDR1_PIN                                    (DL_GPIO_PIN_9)
#define Gray_Addr_B_ADDR1_IOMUX                                  (IOMUX_PINCM26)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_PWM1_init(void);
void SYSCFG_DL_MOTOR_PID_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_OLED_init(void);
void SYSCFG_DL_I2C_0_init(void);
void SYSCFG_DL_Serial_init(void);
void SYSCFG_DL_POT_init(void);
void SYSCFG_DL_ADC12_0_init(void);
void SYSCFG_DL_VREF_init(void);
void SYSCFG_DL_DMA_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
