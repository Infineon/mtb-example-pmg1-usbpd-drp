/******************************************************************************
* File Name: type_a.c
*
* Description: TYPE-A port source file
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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

#include <stdbool.h>
#include "cybsp.h"
#include "config.h"
#include "cy_gpio.h"
#include "cy_tcpwm_pwm.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_app_status.h"
#include "cy_app_battery_charging.h"
#include "cy_pdutils_sw_timer.h"
#include "type_a.h"

#if (CCG_TYPE_A_PORT_ENABLE == 1)

/* Default PWM period. */
#define PWM_A_PWM_PERIOD_VALUE                  (480)
/* Number of counts configured as PWM period. */
#define VBUS_A_PWM_PERIOD                       (PWM_A_PWM_PERIOD_VALUE)
/* Maximum change in percentage in PWM duty cycle allowed in one step. */
#define TYPE_A_PWM_STEP_MAX_PER                 (10)
/* Maximum change in PWM duty cycle in terms of PWM counts. */
#define TYPE_A_PWM_STEP_MAX_SIZE                ((int)(VBUS_A_PWM_PERIOD) * \
        (TYPE_A_PWM_STEP_MAX_PER) / 100)
/* TYPE-A Disconnect Vsense in mV. */
/*
 * TYPE-A Rsense is 20m Ohm and gain is 200. By default we are looking for
 * 200mA as detach threshold. This translates to 800mV Vsense.
 */
#define TYPE_A_DISCON_VSENSE                    (800)

/* Minimum voltage supported on Type-A port in mV. */
#define TYPE_A_VBUS_MIN_VOLT                    (3300)

/* Maximum voltage supported on Type-A port in mV. */
#define TYPE_A_VBUS_MAX_VOLT                    (20000)

extern cy_stc_pdutils_sw_timer_t gl_TimerCtx;
extern cy_stc_usbpd_context_t   gl_UsbPdPort0Ctx;
extern cy_stc_usbpd_context_t   gl_TypeAportCtx;

/* TYPE-A port status. */
type_a_status_t gl_type_a_status;

type_a_status_t* type_a_get_status(void)
{
    return &gl_type_a_status;
}

bool type_a_is_idle(void)
{
    /*
     * Return true if TYPE-A port is not connected so that system can enter low
     * power mode. If TYPE-A port is connected, then PWM is being used for supplying
     * VBUS and hence system can't enter low power mode.
     * Note: It is expected that legacy charging state machine checks for TYPE-A port
     * are done outside of this function for determining IDLE status.
     */
    return !gl_type_a_status.type_a_connected;
}

static void type_a_enable_boost(void)
{
    Cy_GPIO_Write(BUCK_BOOST_EN_A_PORT, BUCK_BOOST_EN_A_PIN, 0u);
}

static void type_a_disable_boost(void)
{
    Cy_GPIO_Write(BUCK_BOOST_EN_A_PORT, BUCK_BOOST_EN_A_PIN, 1u);
    /* Turn off PWM. */
    Cy_TCPWM_PWM_SetCompare0(PWM_A_HW, PWM_A_NUM, 1u);
    Cy_TCPWM_PWM_Disable(PWM_A_HW, PWM_A_NUM);
    Cy_GPIO_SetDrivemode(PWM_OUT_A_PORT, PWM_OUT_A_PIN, CY_GPIO_DM_ANALOG);
    Cy_PdUtils_SwTimer_Stop(&gl_TimerCtx, TYPE_A_PWM_STEP_TIMER_ID);
    gl_type_a_status.pwm_duty = 0;
}

void type_a_reg_switch_timer_cb(cy_timer_id_t id, void *cbContext)
{
    (void)id;
    (void)cbContext;
    /* Disable low power regulator here. */
    Cy_GPIO_SetDrivemode(TYPE_A_VBUS_EN_PORT, TYPE_A_VBUS_EN_PIN, CY_GPIO_DM_ANALOG);
}

void type_a_update_status(bool is_connect, bool detach_det)
{
    if (is_connect == true)
    {
#ifdef TYPE_A_DUAL_REG_ENABLE
        /* TYPE-A port is in connect state. Enable buck-boost regulator with 5V. */
        type_a_set_voltage(CY_PD_VSAFE_5V);
        type_a_enable_boost();

        /*
         * We have to give some time before turning off low power regulator.
         * This gives time to buck boost regulator to sustain loaded conditions.
         * Using a timer for this.
         */
        Cy_PdUtils_SwTimer_Start(&gl_TimerCtx, NULL, TYPE_A_REG_SWITCH_TIMER_ID, TYPE_A_REG_SWITCH_TIMER_PERIOD,
            type_a_reg_switch_timer_cb);
#endif /* TYPE_A_DUAL_REG_ENABLE */
    }
    else
    {
#ifdef TYPE_A_DUAL_REG_ENABLE
        /*
         * TYPE-A port is no longer connected. Disable buck-boost regulator and switch
         * to 5V VBUS from LDO.
         */
        type_a_disable_boost();

        Cy_PdUtils_SwTimer_Stop(&gl_TimerCtx, TYPE_A_REG_SWITCH_TIMER_ID);
        /* Enable VBUS from low power regulator. */
        Cy_GPIO_Write(TYPE_A_VBUS_EN_PORT, TYPE_A_VBUS_EN_PIN, 0u);
        Cy_GPIO_SetDrivemode(TYPE_A_VBUS_EN_PORT, TYPE_A_VBUS_EN_PIN, CY_GPIO_DM_STRONG_IN_OFF);
#endif /* TYPE_A_DUAL_REG_ENABLE */
    }
    
    /* Ensure TYPE-A current sense debounce timer is not running. */
    gl_type_a_status.cur_sense_debounce_active = false;
    Cy_PdUtils_SwTimer_Stop(&gl_TimerCtx, TYPE_A_CUR_SENSE_TIMER_ID);

    gl_type_a_status.type_a_connected = is_connect;
    gl_type_a_status.detach_det = detach_det;
}

void type_a_set_volt_timer_cbk(cy_timer_id_t id, void *cbContext)
{
    int32_t tmp;

    (void)id;
    (void)cbContext;

    tmp = (gl_type_a_status.new_pwm_duty - gl_type_a_status.pwm_duty);

    /* Adjust to the maximum step size allowed. */
    if (tmp > 0)
    {
        tmp = CY_USBPD_GET_MIN(tmp, TYPE_A_PWM_STEP_MAX_SIZE);
    }
    else
    {
        tmp = CY_USBPD_GET_MAX(tmp, (-1 * (int32_t)(TYPE_A_PWM_STEP_MAX_SIZE)));
    }

    gl_type_a_status.pwm_duty += tmp;
    Cy_TCPWM_PWM_SetCompare0(PWM_A_HW, PWM_A_NUM, gl_type_a_status.pwm_duty);

    if (gl_type_a_status.pwm_duty != gl_type_a_status.new_pwm_duty)
    {
        /* Start the timer if the PWM needs further adjustment. */
        Cy_PdUtils_SwTimer_Start(&gl_TimerCtx, NULL, TYPE_A_PWM_STEP_TIMER_ID, TYPE_A_PWM_STEP_TIMER_PERIOD,
                type_a_set_volt_timer_cbk);
    }
}

static uint16_t pwm_get_duty_cycle(uint16_t vbus, uint16_t min_volt, uint16_t max_volt, uint16_t pwm_period)
{
    uint32_t pwm_duty = 0;

    /*
     * Formula for Vout based on PWM Duty cycle is:
     * Vout = VBUS_MAX * (1/6 + 5/6 * PWM_DUTY_CYCLE)
     * Substituting PWM Period value and compare value (pwm_duty) in this formula
     * Vout = VBUS_MAX * (1/6 + 5/6 * pwm_duty/PWM_PERIOD)
     * Solving for pwm_duty
     * pwm_duty = PWM_PERIOD * ((Vout * 6 - VBUS_MAX)/VBUS_MAX * 5)
     */

    /* Ensure VBUS requested is not less than VBUS_MIN and more than VBUS_MAX. */
    if((vbus >= min_volt) && (vbus <= max_volt))
    {
        /* Applying formula as above. */
        pwm_duty = (vbus * 6) - max_volt;
        pwm_duty *= pwm_period;
        pwm_duty /=  max_volt * 5;
    }

    if(pwm_duty == 0)
    {
        pwm_duty = 1;
    }

    return pwm_duty;
}

void type_a_set_voltage(uint16_t volt_mV)
{
    Cy_PdUtils_SwTimer_Stop(&gl_TimerCtx, TYPE_A_PWM_STEP_TIMER_ID);

    /* Set up PWM for TYPE-A VBUS. */
    gl_type_a_status.new_pwm_duty = pwm_get_duty_cycle(volt_mV, 
            TYPE_A_VBUS_MIN_VOLT, TYPE_A_VBUS_MAX_VOLT, VBUS_A_PWM_PERIOD);

    Cy_TCPWM_PWM_Enable(PWM_A_HW, PWM_A_NUM);
    Cy_TCPWM_TriggerStart(PWM_A_HW, PWM_A_MASK);
    /* Invoke the callback directly to avoid losing time. */
    type_a_set_volt_timer_cbk(TYPE_A_PWM_STEP_TIMER_ID, NULL);
    Cy_GPIO_SetDrivemode(PWM_OUT_A_PORT, PWM_OUT_A_PIN, CY_GPIO_DM_STRONG_IN_OFF);

    gl_type_a_status.cur_vbus_a = volt_mV;
}

void type_a_enable_disable_vbus(bool on_off)
{
    if(on_off == true)
    {
#ifndef TYPE_A_DUAL_REG_ENABLE
        /* Enable VBUS_A high-voltage buck-boost VBUS regulator. */
        type_a_enable_boost();
#else
        /*
         * Do nothing because either buck-boost is already enabled or low power
         * regulator is providing default 5V VBUS.
         */
#endif /* TYPE_A_DUAL_REG_ENABLE */
    }
    else
    {
        /* Disable VBUS_A buck-boost VBUS regulator. */
        type_a_disable_boost();

#ifdef TYPE_A_DUAL_REG_ENABLE
        /* Disable VBUS from low-power regulator as well. */
        Cy_GPIO_SetDrivemode(TYPE_A_VBUS_EN_PORT, TYPE_A_VBUS_EN_PIN, CY_GPIO_DM_ANALOG);
#endif /* TYPE_A_DUAL_REG_ENABLE */

        gl_type_a_status.cur_vbus_a = 0u; 
    }
}

void type_a_port_enable_disable(bool en_dis)
{
    cy_stc_usbpd_context_t *context = &gl_TypeAportCtx;
    if(en_dis == true)
    {
        /* Turn on VBUS. */
#ifndef TYPE_A_DUAL_REG_ENABLE
        /* Enabling Buck Boost here for 5V VBUS as 5V from LDO is not available. */
        type_a_set_voltage(CY_PD_VSAFE_5V);
        type_a_enable_disable_vbus(true);
#endif /* TYPE_A_DUAL_REG_ENABLE */

        /* Initialize legacy charging. */
#if BATTERY_CHARGING_ENABLE
        Cy_App_Bc_Init(context, &gl_TimerCtx);
        Cy_App_Bc_Start(context);
#endif /* BATTERY_CHARGING_ENABLE */
    
        gl_type_a_status.type_a_enabled = true;
        gl_type_a_status.detach_det = false;
        gl_type_a_status.cur_sense_debounce_active = false;
        gl_type_a_status.sdp_mode_enabled = false;
    }
    else
    {
        /* Disable TYPE-A port. Don't provide any VBUS. */
        type_a_enable_disable_vbus(false);
#if BATTERY_CHARGING_ENABLE
        /* Disable Legacy charging. */
        Cy_App_Bc_Stop(context);
#endif /* BATTERY_CHARGING_ENABLE */

        gl_type_a_status.type_a_enabled = false;
        gl_type_a_status.type_a_connected = false;
        
        /* Ensure TYPE-A current sense de-bounce timer is not running. */
        gl_type_a_status.cur_sense_debounce_active = false;
        gl_type_a_status.sdp_mode_enabled = false;
        Cy_PdUtils_SwTimer_Stop(&gl_TimerCtx, TYPE_A_CUR_SENSE_TIMER_ID);        
    }
}

void type_a_detect_disconnect(void)
{
    /*
     * Run through detach detection if TYPE-A port is already connected to a
     * BC 1.2 or Apple device. These devices require detach detection based on
     * current drawn from the port.
     */
    if ((gl_type_a_status.type_a_connected == true) &&
        (gl_type_a_status.detach_det == true))
    {
        bool comp_out;

        uint8_t dac_level;
        Cy_GPIO_SetHSIOM(TYPE_A_CUR_SENSE_PORT, TYPE_A_CUR_SENSE_PIN, HSIOM_SEL_AMUXA);

        /* Convert Vsense to DAC level. */
        dac_level = Cy_USBPD_Adc_VoltToLevel(&gl_UsbPdPort0Ctx, CY_USBPD_ADC_ID_1, TYPE_A_DISCON_VSENSE);

        /* Get comparator status. */
        comp_out = Cy_USBPD_Adc_CompSample(&gl_UsbPdPort0Ctx, CY_USBPD_ADC_ID_1,
                CY_USBPD_ADC_INPUT_AMUX_A, dac_level);
        Cy_GPIO_SetHSIOM(TYPE_A_CUR_SENSE_PORT, TYPE_A_CUR_SENSE_PIN, HSIOM_SEL_GPIO);

        /*
         * If comparator output is high, TYPE-A port is in attached state.
         * Otherwise it is disconnected state.
         */
        if (comp_out == false)
        {
            /*
             * We need to debounce this condition for 30 seconds before
             * switching to low power regulator.
             */
            if (gl_type_a_status.cur_sense_debounce_active == false)
            {
                /* Start debounce timer. */
                Cy_PdUtils_SwTimer_Start(&gl_TimerCtx, NULL, TYPE_A_CUR_SENSE_TIMER_ID,
                        TYPE_A_CUR_SENSE_TIMER_PERIOD, NULL);
                gl_type_a_status.cur_sense_debounce_active = true;
            }
            else
            {
                /* Debounce active. Check if debounce time has expired. */
                if (!(Cy_PdUtils_SwTimer_IsRunning(&gl_TimerCtx, TYPE_A_CUR_SENSE_TIMER_ID)))
                {
                    gl_type_a_status.cur_sense_debounce_active = false;
                    /* Indicate TYPE-A disconnect. */
                    type_a_update_status(false, false);
                }
            }
        }
        else
        {
            /* Don't debounce disconnect condition. */
            gl_type_a_status.cur_sense_debounce_active = false;
            Cy_PdUtils_SwTimer_Stop(&gl_TimerCtx, TYPE_A_CUR_SENSE_TIMER_ID);
        }
    }
}
#endif /* CCG_TYPE_A_PORT_ENABLE */

/* End of File */
