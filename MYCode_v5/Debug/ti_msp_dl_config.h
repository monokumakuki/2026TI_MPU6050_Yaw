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


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     80000000
/* Defines for SYSPLL_ERR_01 Workaround */
/* Represent 1.000 as 1000 */
#define FLOAT_TO_INT_SCALE                                               (1000U)
#define FCC_EXPECTED_RATIO                                                  2000
#define FCC_UPPER_BOUND                       (FCC_EXPECTED_RATIO * (1 + 0.003))
#define FCC_LOWER_BOUND                       (FCC_EXPECTED_RATIO * (1 - 0.003))

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);


/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMG7
#define PWM_0_INST_IRQHandler                                   TIMG7_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMG7_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             10000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOB
#define GPIO_PWM_0_C0_PIN                                         DL_GPIO_PIN_15
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM32)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM32_PF_TIMG7_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOB
#define GPIO_PWM_0_C1_PIN                                         DL_GPIO_PIN_16
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM33)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM33_PF_TIMG7_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX

/* Defines for PWM_Servo */
#define PWM_Servo_INST                                                     TIMA0
#define PWM_Servo_INST_IRQHandler                               TIMA0_IRQHandler
#define PWM_Servo_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define PWM_Servo_INST_CLK_FREQ                                          1250000
/* GPIO defines for channel 0 */
#define GPIO_PWM_Servo_C0_PORT                                             GPIOB
#define GPIO_PWM_Servo_C0_PIN                                      DL_GPIO_PIN_8
#define GPIO_PWM_Servo_C0_IOMUX                                  (IOMUX_PINCM25)
#define GPIO_PWM_Servo_C0_IOMUX_FUNC                 IOMUX_PINCM25_PF_TIMA0_CCP0
#define GPIO_PWM_Servo_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_Servo_C1_PORT                                             GPIOB
#define GPIO_PWM_Servo_C1_PIN                                      DL_GPIO_PIN_9
#define GPIO_PWM_Servo_C1_IOMUX                                  (IOMUX_PINCM26)
#define GPIO_PWM_Servo_C1_IOMUX_FUNC                 IOMUX_PINCM26_PF_TIMA0_CCP1
#define GPIO_PWM_Servo_C1_IDX                                DL_TIMER_CC_1_INDEX



/* Defines for PID_TIMER */
#define PID_TIMER_INST                                                   (TIMG0)
#define PID_TIMER_INST_IRQHandler                               TIMG0_IRQHandler
#define PID_TIMER_INST_INT_IRQN                                 (TIMG0_INT_IRQn)
#define PID_TIMER_INST_LOAD_VALUE                                       (24999U)




/* Defines for OLED */
#define OLED_INST                                                           I2C0
#define OLED_INST_IRQHandler                                     I2C0_IRQHandler
#define OLED_INST_INT_IRQN                                         I2C0_INT_IRQn
#define OLED_BUS_SPEED_HZ                                                 100000
#define GPIO_OLED_SDA_PORT                                                 GPIOA
#define GPIO_OLED_SDA_PIN                                         DL_GPIO_PIN_28
#define GPIO_OLED_IOMUX_SDA                                       (IOMUX_PINCM3)
#define GPIO_OLED_IOMUX_SDA_FUNC                        IOMUX_PINCM3_PF_I2C0_SDA
#define GPIO_OLED_SCL_PORT                                                 GPIOA
#define GPIO_OLED_SCL_PIN                                         DL_GPIO_PIN_31
#define GPIO_OLED_IOMUX_SCL                                       (IOMUX_PINCM6)
#define GPIO_OLED_IOMUX_SCL_FUNC                        IOMUX_PINCM6_PF_I2C0_SCL

/* Defines for MPU6050 */
#define MPU6050_INST                                                        I2C1
#define MPU6050_INST_IRQHandler                                  I2C1_IRQHandler
#define MPU6050_INST_INT_IRQN                                      I2C1_INT_IRQn
#define MPU6050_BUS_SPEED_HZ                                              100000
#define GPIO_MPU6050_SDA_PORT                                              GPIOA
#define GPIO_MPU6050_SDA_PIN                                      DL_GPIO_PIN_10
#define GPIO_MPU6050_IOMUX_SDA                                   (IOMUX_PINCM21)
#define GPIO_MPU6050_IOMUX_SDA_FUNC                    IOMUX_PINCM21_PF_I2C1_SDA
#define GPIO_MPU6050_SCL_PORT                                              GPIOA
#define GPIO_MPU6050_SCL_PIN                                      DL_GPIO_PIN_11
#define GPIO_MPU6050_IOMUX_SCL                                   (IOMUX_PINCM22)
#define GPIO_MPU6050_IOMUX_SCL_FUNC                    IOMUX_PINCM22_PF_I2C1_SCL


/* Defines for UART_Cam */
#define UART_Cam_INST                                                      UART3
#define UART_Cam_INST_FREQUENCY                                         80000000
#define UART_Cam_INST_IRQHandler                                UART3_IRQHandler
#define UART_Cam_INST_INT_IRQN                                    UART3_INT_IRQn
#define GPIO_UART_Cam_RX_PORT                                              GPIOB
#define GPIO_UART_Cam_TX_PORT                                              GPIOB
#define GPIO_UART_Cam_RX_PIN                                       DL_GPIO_PIN_3
#define GPIO_UART_Cam_TX_PIN                                       DL_GPIO_PIN_2
#define GPIO_UART_Cam_IOMUX_RX                                   (IOMUX_PINCM16)
#define GPIO_UART_Cam_IOMUX_TX                                   (IOMUX_PINCM15)
#define GPIO_UART_Cam_IOMUX_RX_FUNC                    IOMUX_PINCM16_PF_UART3_RX
#define GPIO_UART_Cam_IOMUX_TX_FUNC                    IOMUX_PINCM15_PF_UART3_TX
#define UART_Cam_BAUD_RATE                                              (115200)
#define UART_Cam_IBRD_80_MHZ_115200_BAUD                                    (43)
#define UART_Cam_FBRD_80_MHZ_115200_BAUD                                    (26)
/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           40000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                         DL_GPIO_PIN_1
#define GPIO_UART_0_TX_PIN                                         DL_GPIO_PIN_0
#define GPIO_UART_0_IOMUX_RX                                      (IOMUX_PINCM2)
#define GPIO_UART_0_IOMUX_TX                                      (IOMUX_PINCM1)
#define GPIO_UART_0_IOMUX_RX_FUNC                       IOMUX_PINCM2_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                       IOMUX_PINCM1_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_40_MHZ_115200_BAUD                                      (21)
#define UART_0_FBRD_40_MHZ_115200_BAUD                                      (45)





/* Defines for AIN1: GPIOA.13 with pinCMx 35 on package pin 6 */
#define TB6612_AIN1_PORT                                                 (GPIOA)
#define TB6612_AIN1_PIN                                         (DL_GPIO_PIN_13)
#define TB6612_AIN1_IOMUX                                        (IOMUX_PINCM35)
/* Defines for AIN2: GPIOA.12 with pinCMx 34 on package pin 5 */
#define TB6612_AIN2_PORT                                                 (GPIOA)
#define TB6612_AIN2_PIN                                         (DL_GPIO_PIN_12)
#define TB6612_AIN2_IOMUX                                        (IOMUX_PINCM34)
/* Defines for BIN1: GPIOB.0 with pinCMx 12 on package pin 47 */
#define TB6612_BIN1_PORT                                                 (GPIOB)
#define TB6612_BIN1_PIN                                          (DL_GPIO_PIN_0)
#define TB6612_BIN1_IOMUX                                        (IOMUX_PINCM12)
/* Defines for BIN2: GPIOB.1 with pinCMx 13 on package pin 48 */
#define TB6612_BIN2_PORT                                                 (GPIOB)
#define TB6612_BIN2_PIN                                          (DL_GPIO_PIN_1)
#define TB6612_BIN2_IOMUX                                        (IOMUX_PINCM13)
/* Defines for LEFT1: GPIOB.25 with pinCMx 56 on package pin 27 */
#define LINE_LEFT1_PORT                                                  (GPIOB)
#define LINE_LEFT1_PIN                                          (DL_GPIO_PIN_25)
#define LINE_LEFT1_IOMUX                                         (IOMUX_PINCM56)
/* Defines for LEFT2: GPIOB.24 with pinCMx 52 on package pin 23 */
#define LINE_LEFT2_PORT                                                  (GPIOB)
#define LINE_LEFT2_PIN                                          (DL_GPIO_PIN_24)
#define LINE_LEFT2_IOMUX                                         (IOMUX_PINCM52)
/* Defines for LEFT3: GPIOB.20 with pinCMx 48 on package pin 19 */
#define LINE_LEFT3_PORT                                                  (GPIOB)
#define LINE_LEFT3_PIN                                          (DL_GPIO_PIN_20)
#define LINE_LEFT3_IOMUX                                         (IOMUX_PINCM48)
/* Defines for LEFT4: GPIOA.14 with pinCMx 36 on package pin 7 */
#define LINE_LEFT4_PORT                                                  (GPIOA)
#define LINE_LEFT4_PIN                                          (DL_GPIO_PIN_14)
#define LINE_LEFT4_IOMUX                                         (IOMUX_PINCM36)
/* Defines for RIGHT1: GPIOB.18 with pinCMx 44 on package pin 15 */
#define LINE_RIGHT1_PORT                                                 (GPIOB)
#define LINE_RIGHT1_PIN                                         (DL_GPIO_PIN_18)
#define LINE_RIGHT1_IOMUX                                        (IOMUX_PINCM44)
/* Defines for RIGHT2: GPIOB.19 with pinCMx 45 on package pin 16 */
#define LINE_RIGHT2_PORT                                                 (GPIOB)
#define LINE_RIGHT2_PIN                                         (DL_GPIO_PIN_19)
#define LINE_RIGHT2_IOMUX                                        (IOMUX_PINCM45)
/* Defines for RIGHT3: GPIOB.10 with pinCMx 27 on package pin 62 */
#define LINE_RIGHT3_PORT                                                 (GPIOB)
#define LINE_RIGHT3_PIN                                         (DL_GPIO_PIN_10)
#define LINE_RIGHT3_IOMUX                                        (IOMUX_PINCM27)
/* Defines for RIGHT4: GPIOA.7 with pinCMx 14 on package pin 49 */
#define LINE_RIGHT4_PORT                                                 (GPIOA)
#define LINE_RIGHT4_PIN                                          (DL_GPIO_PIN_7)
#define LINE_RIGHT4_IOMUX                                        (IOMUX_PINCM14)
/* Defines for KEY1: GPIOA.30 with pinCMx 5 on package pin 37 */
#define KEY_KEY1_PORT                                                    (GPIOA)
#define KEY_KEY1_PIN                                            (DL_GPIO_PIN_30)
#define KEY_KEY1_IOMUX                                            (IOMUX_PINCM5)
/* Defines for KEY2: GPIOB.11 with pinCMx 28 on package pin 63 */
#define KEY_KEY2_PORT                                                    (GPIOB)
#define KEY_KEY2_PIN                                            (DL_GPIO_PIN_11)
#define KEY_KEY2_IOMUX                                           (IOMUX_PINCM28)
/* Defines for KEY3: GPIOB.14 with pinCMx 31 on package pin 2 */
#define KEY_KEY3_PORT                                                    (GPIOB)
#define KEY_KEY3_PIN                                            (DL_GPIO_PIN_14)
#define KEY_KEY3_IOMUX                                           (IOMUX_PINCM31)
/* Defines for KEY4: GPIOA.18 with pinCMx 40 on package pin 11 */
#define KEY_KEY4_PORT                                                    (GPIOA)
#define KEY_KEY4_PIN                                            (DL_GPIO_PIN_18)
#define KEY_KEY4_IOMUX                                           (IOMUX_PINCM40)
/* Port definition for Pin Group Encoder */
#define Encoder_PORT                                                     (GPIOA)

/* Defines for Left_Encode_A: GPIOA.17 with pinCMx 39 on package pin 10 */
// pins affected by this interrupt request:["Left_Encode_A","Right_Encode_A"]
#define Encoder_INT_IRQN                                        (GPIOA_INT_IRQn)
#define Encoder_INT_IIDX                        (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define Encoder_Left_Encode_A_IIDX                          (DL_GPIO_IIDX_DIO17)
#define Encoder_Left_Encode_A_PIN                               (DL_GPIO_PIN_17)
#define Encoder_Left_Encode_A_IOMUX                              (IOMUX_PINCM39)
/* Defines for Left_Encode_B: GPIOA.24 with pinCMx 54 on package pin 25 */
#define Encoder_Left_Encode_B_PIN                               (DL_GPIO_PIN_24)
#define Encoder_Left_Encode_B_IOMUX                              (IOMUX_PINCM54)
/* Defines for Right_Encode_A: GPIOA.15 with pinCMx 37 on package pin 8 */
#define Encoder_Right_Encode_A_IIDX                         (DL_GPIO_IIDX_DIO15)
#define Encoder_Right_Encode_A_PIN                              (DL_GPIO_PIN_15)
#define Encoder_Right_Encode_A_IOMUX                             (IOMUX_PINCM37)
/* Defines for Right_Encode_B: GPIOA.16 with pinCMx 38 on package pin 9 */
#define Encoder_Right_Encode_B_PIN                              (DL_GPIO_PIN_16)
#define Encoder_Right_Encode_B_IOMUX                             (IOMUX_PINCM38)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_SYSCTL_CLK_init(void);

bool SYSCFG_DL_SYSCTL_SYSPLL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_PWM_Servo_init(void);
void SYSCFG_DL_PID_TIMER_init(void);
void SYSCFG_DL_OLED_init(void);
void SYSCFG_DL_MPU6050_init(void);
void SYSCFG_DL_UART_Cam_init(void);
void SYSCFG_DL_UART_0_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
