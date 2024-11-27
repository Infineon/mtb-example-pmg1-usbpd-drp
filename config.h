/******************************************************************************
* File Name: config.h
*
* Description: This header file defines the application configuration for the PMG1
*              MCU USBPD DRP Example for ModusToolBox.
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#ifndef _CONFIG_H_
#define _CONFIG_H_


/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include "cybsp.h"
#include "cy_pdutils_sw_timer.h"

/*
 * Macro defines additional delay in milliseconds before the PD stack starts sending
 * SRC_CAP message. This may be required to work with some non-compliant sink devices
 * which require more start up time for PD.
 */
#define DELAY_SRC_CAP_START_MS                  (100u)

/*******************************************************************************
 * USB-PD SAR ADC Configurations
 ******************************************************************************/

#define APP_VBUS_POLL_ADC_ID                    (CY_USBPD_ADC_ID_0)
#if defined(CY_DEVICE_CCG3)
#define APP_VBUS_POLL_ADC_INPUT                 (CY_USBPD_ADC_INPUT_AMUX_A)
#else
#define APP_VBUS_POLL_ADC_INPUT                 (CY_USBPD_ADC_INPUT_AMUX_B)
#endif /* defined(CY_DEVICE_CCG3) */

#if (defined(CY_DEVICE_CCG3))
/* Function/Macro to set P1 source voltage to 5V. */
#define APP_VBUS_SET_5V_P1()                        \
{                                                   \
   Cy_GPIO_Write(VSEL1_PORT, VSEL1_PIN, 0);         \
   Cy_GPIO_Write(VSEL2_PORT, VSEL2_PIN, 0);         \
}

/* Function/Macro to set P1 source voltage to 9V. */
#define APP_VBUS_SET_9V_P1()                        \
{                                                   \
   Cy_GPIO_Write(VSEL1_PORT, VSEL1_PIN, 1);         \
   Cy_GPIO_Write(VSEL2_PORT, VSEL2_PIN, 0);         \
}

/* Function/Macro to set P1 source voltage to 12V. Not supported on CY4531. */
#define APP_VBUS_SET_12V_P1()                       ((void)0)

/* Function/Macro to set P1 source voltage to 13V. Not supported on CY4531. */
#define APP_VBUS_SET_13V_P1()                       ((void)0)

/* Function/Macro to set P1 source voltage to 15V. */
#define APP_VBUS_SET_15V_P1()                       \
{                                                   \
    Cy_GPIO_Write(VSEL1_PORT, VSEL1_PIN, 0);        \
    Cy_GPIO_Write(VSEL2_PORT, VSEL2_PIN, 1);        \
}

/* Function/Macro to set P1 source voltage to 19V. Not supported on CY4531. */
#define APP_VBUS_SET_19V_P1()                       ((void)0)

/* Function/Macro to set P1 source voltage to 20V. */
#define APP_VBUS_SET_20V_P1()                       \
{                                                   \
    Cy_GPIO_Write(VSEL1_PORT, VSEL1_PIN, 1);        \
    Cy_GPIO_Write(VSEL2_PORT, VSEL2_PIN, 1);        \
}
#endif /* (defined(CY_DEVICE_PMG1S3) || defined(CY_DEVICE_CCG6)) */

#ifndef APP_FW_LED_ENABLE
#define APP_FW_LED_ENABLE                          (0u)
#endif /* APP_FW_LED_ENABLE */

#if APP_FW_LED_ENABLE
/*
 * Port0 activity indicator LED timer.
 */
#define LED1_TIMER_ID                               (CY_PDUTILS_TIMER_USER_START_ID)

/*
 * Port1 activity indicator LED timer.
 */
#define LED2_TIMER_ID                               (CY_PDUTILS_TIMER_USER_START_ID + 1u)

/*
 * The LED toggle period (ms).
 */
#define LED_TIMER_PERIOD                            (500u)
#endif /* APP_FW_LED_ENABLE */

/* Enable dual regulator support for TYPE-A VBUS. */
#define TYPE_A_DUAL_REG_ENABLE                      (1u)

/* TYPE-A port PWM step change timer. */
#define TYPE_A_PWM_STEP_TIMER_ID                    (CY_PDUTILS_TIMER_USER_START_ID + 2u)

/* TYPE-A current sense timer. */
#define TYPE_A_CUR_SENSE_TIMER_ID                   (CY_PDUTILS_TIMER_USER_START_ID + 3u)

/* TYPE-A regulator switch timer. */
#define TYPE_A_REG_SWITCH_TIMER_ID                  (CY_PDUTILS_TIMER_USER_START_ID + 4u)

/* TYPE-A port PWM step change time in ms. */
#define TYPE_A_PWM_STEP_TIMER_PERIOD                (1)

/* TYPE-A current sense timer period. */
#define TYPE_A_CUR_SENSE_TIMER_PERIOD               (30000)

/* TYPE-A regulator switch timer period in ms. */
#define TYPE_A_REG_SWITCH_TIMER_PERIOD              (10)

#endif /* _CONFIG_H_ */

/* End of file [] */
