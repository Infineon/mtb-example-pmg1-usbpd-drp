/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PMG1 MCU: USBPD DRP Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
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

#include "cy_pdl.h"
#include "cybsp.h"
#include "config.h"

#include "cy_pdutils_sw_timer.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_usbpd_typec.h"
#include "cy_pdstack_dpm.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_phy.h"
#include "cy_app_instrumentation.h"
#include "cy_app.h"
#include "cy_app_pdo.h"
#include "cy_app_sink.h"
#include "cy_app_source.h"
#include "cy_app_swap.h"
#include "cy_app_vdm.h"
#include "ncp_81239.h"
#include "mtbcfg_ezpd.h"
#include "cy_app_fault_handlers.h"
#if CCG_TYPE_A_PORT_ENABLE
#include "type_a.h"
#endif /* CCG_TYPE_A_PORT_ENABLE */

#if APP_FW_LED_ENABLE
/* Structure to hold the user LED configuration. */
typedef struct
{
    GPIO_PRT_Type* gpioPort;    /* User LED port base address */
    uint32_t gpioPin;           /* User LED pin GPIO number */
    uint16_t timerId;           /* User LED Timer Id */
}cy_stc_user_led_config_t;

/* Variable to store the user LED configuration. */
static cy_stc_user_led_config_t gl_userLedConfig[NO_OF_TYPEC_PORTS] =
{
    {CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_PIN, LED1_TIMER_ID},
#if PMG1_PD_DUALPORT_ENABLE
    {CYBSP_USER_LED2_PORT, CYBSP_USER_LED2_PIN, LED2_TIMER_ID},
#endif /* PMG1_PD_DUALPORT_ENABLE */
};
#endif /* APP_FW_LED_ENABLE */

/* Software Timer context variable. */
cy_stc_pdutils_sw_timer_t gl_TimerCtx;

cy_stc_usbpd_context_t   gl_UsbPdPort0Ctx;
cy_stc_pdstack_context_t gl_PdStackPort0Ctx;

#if PMG1_PD_DUALPORT_ENABLE
cy_stc_usbpd_context_t gl_UsbPdPort1Ctx;
cy_stc_pdstack_context_t gl_PdStackPort1Ctx;
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if CCG_TYPE_A_PORT_ENABLE
cy_stc_usbpd_context_t gl_TypeAportCtx;
#endif /* CCG_TYPE_A_PORT_ENABLE */

cy_stc_scb_i2c_context_t gl_i2cm_context;

static bool glSrcCapChngDpmCmdPending[NO_OF_TYPEC_PORTS] =
{
    false,
#if PMG1_PD_DUALPORT_ENABLE
    false
#endif /* PMG1_PD_DUALPORT_ENABLE */
};

#if (defined(CY_DEVICE_PMG1S3) || defined(CY_DEVICE_CCG6))
ncp_81239_context_t gl_ncp81239ContextPort1 =
{
    .i2cAddr = NCP_PORT_1_SLAVE_ADDR,
    .scbBase = I2CM_HW,
    .i2cContext = &gl_i2cm_context,
    .enableGpioPort = NCP_ENABLE_P0_PORT,
    .enableGpioPin = NCP_ENABLE_P0_PIN
};

#endif /* (defined(CY_DEVICE_PMG1S3) || defined(CY_DEVICE_CCG6)) */

#if PMG1_PD_DUALPORT_ENABLE
ncp_81239_context_t gl_ncp81239ContextPort2 =
{
    .i2cAddr = NCP_PORT_2_SLAVE_ADDR,
    .scbBase = I2CM_HW,
    .i2cContext = &gl_i2cm_context,
    .enableGpioPort = NCP_ENABLE_P1_PORT,
    .enableGpioPin = NCP_ENABLE_P1_PIN
};
#endif /* PMG1_PD_DUALPORT_ENABLE */

uint32_t gl_discIdRespPort0[7] = {0xFF00A841, 0x19C004B4, 0x00000000, 0xF5010000, 0x40000000};
#if PMG1_PD_DUALPORT_ENABLE
uint32_t gl_discIdRespPort1[7] = {0xFF00A841, 0x19C004B4, 0x00000000, 0xF5010000, 0x40000001};
#endif /* PMG1_PD_DUALPORT_ENABLE */

const cy_stc_pdstack_dpm_params_t pdstack_port0_dpm_params =
{
    .dpmSnkWaitCapPeriod = 400,
    .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
    .dpmDefCableCap = 300,
    .muxEnableDelayPeriod = 0,
    .typeCSnkWaitCapPeriod = 0,
    .defCur = 90
};

#if PMG1_PD_DUALPORT_ENABLE
/* PD Stack DPM Parameters for Port 1 */
const cy_stc_pdstack_dpm_params_t pdstack_port1_dpm_params =
{
    .dpmSnkWaitCapPeriod = 400,
    .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
    .dpmDefCableCap = 300,
    .muxEnableDelayPeriod = 0,
    .typeCSnkWaitCapPeriod = 0,
    .defCur = 90
};
#endif /* PMG1_PD_DUALPORT_ENABLE */

const cy_stc_app_params_t port0_app_params =
{
    .appVbusPollAdcId = APP_VBUS_POLL_ADC_ID,
    .appVbusPollAdcInput = APP_VBUS_POLL_ADC_INPUT,
    .prefPowerRole = 2u, /* 0-Sink, 1-Source, 2-No Preference */
    .prefDataRole = 2u, /* 0-UFP, 1- DFP, 2-No Preference */
    .discIdResp = (cy_pd_pd_do_t *)&gl_discIdRespPort0[0],
    .discIdLen = 0x18,
    .swapResponse = 0x00
};

#if PMG1_PD_DUALPORT_ENABLE
const cy_stc_app_params_t port1_app_params =
{
    .appVbusPollAdcId = APP_VBUS_POLL_ADC_ID,
    .appVbusPollAdcInput = APP_VBUS_POLL_ADC_INPUT,
    .prefPowerRole = 2u, /* 0-Sink, 1-Source, 2-No Preference */
    .prefDataRole = 2u, /* 0-UFP, 1- DFP, 2-No Preference */
    .discIdResp = (cy_pd_pd_do_t *)&gl_discIdRespPort1[0],
    .discIdLen = 0x18,
    .swapResponse = 0x00
};
#endif /* PMG1_PD_DUALPORT_ENABLE */

cy_stc_pdstack_context_t * gl_PdStackContexts[NO_OF_TYPEC_PORTS] =
{
        &gl_PdStackPort0Ctx,
#if PMG1_PD_DUALPORT_ENABLE
        &gl_PdStackPort1Ctx
#endif /* PMG1_PD_DUALPORT_ENABLE */
};

const cy_stc_sysint_t wdt_interrupt_config =
{
    .intrSrc = (IRQn_Type)srss_interrupt_wdt_IRQn,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port0_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_IRQ,
    .intrPriority = 0U,
};


const cy_stc_sysint_t usbpd_port0_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_DS_IRQ,
    .intrPriority = 0U,
};

#if PMG1_PD_DUALPORT_ENABLE
/* USB PD Port 1 Interrupt 0 Configuration */
const cy_stc_sysint_t usbpd_port1_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_IRQ,
    .intrPriority = 0U,
};

/* USB PD Port 1 Interrupt 1 Configuration */
const cy_stc_sysint_t usbpd_port1_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_DS_IRQ,
    .intrPriority = 0U,
};
#endif /* PMG1_PD_DUALPORT_ENABLE */

/*******************************************************************************
* Function Name: get_pdstack_context
********************************************************************************
* Summary:
*   Returns the respective port PD Stack Context
*
* Parameters:
*  portIdx - Port Index
*
* Return:
*  cy_stc_pdstack_context_t
*
*******************************************************************************/
cy_stc_pdstack_context_t *get_pdstack_context(uint8_t portIdx)
{
    return (gl_PdStackContexts[portIdx]);
}

#if APP_FW_LED_ENABLE
/*******************************************************************************
* Function Name: led_timer_cb
********************************************************************************
* Summary:
*  Toggles the LED
*
* Parameters:
*  id - Timer ID
*  callbackContext - Context
*
* Return:
*  None
*
*******************************************************************************/
void led_timer_cb (
        cy_timer_id_t id,            /**< Timer ID for which callback is being generated. */
        void *callbackContext)       /**< Timer module Context. */
{
    cy_stc_pdstack_context_t *stack_ctx = (cy_stc_pdstack_context_t *)callbackContext;
    cy_stc_user_led_config_t *user_led = &gl_userLedConfig[stack_ctx->port];

    /* Toggle the User LED and re-start timer to schedule the next toggle event. */
    Cy_GPIO_Inv(user_led->gpioPort, user_led->gpioPin);

    Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, callbackContext, id, LED_TIMER_PERIOD, led_timer_cb);
}
#endif /* APP_FW_LED_ENABLE */

/*******************************************************************************
* Function Name: sln_pd_event_handler
********************************************************************************
* Summary:
*   Solution PD Event Handler
*
* Parameters:
*  ctx - PD Stack Context
*  evt - App Event
*  data - Data
*
* Return:
*  None
*
*******************************************************************************/
void sln_pd_event_handler(cy_stc_pdstack_context_t* ctx, cy_en_pdstack_app_evt_t evt, const void *data)
{
#if APP_FW_LED_ENABLE
    cy_stc_user_led_config_t *user_led = &gl_userLedConfig[ctx->port];
#endif /* APP_FW_LED_ENABLE */
    const cy_stc_pdstack_pd_contract_info_t* contract_status;

    switch(evt)
    {
        case APP_EVT_CONNECT:
        case APP_EVT_PR_SWAP_COMPLETE:
#if APP_FW_LED_ENABLE
            if(ctx->dpmConfig.curPortRole == CY_PD_PRT_ROLE_SINK)
            {
                /* Start a timer to toggle the LED */
                Cy_PdUtils_SwTimer_Start (&gl_TimerCtx, ctx, user_led->timerId, LED_TIMER_PERIOD, led_timer_cb);
            }
            else
            {
                /* Turn ON LED */
                Cy_GPIO_Clr(user_led->gpioPort, user_led->gpioPin);
            }
#endif /* APP_FW_LED_ENABLE */
            break;
        case APP_EVT_DISCONNECT:
#if APP_FW_LED_ENABLE
            /* Stop the LED timer */
            Cy_PdUtils_SwTimer_Stop (&gl_TimerCtx, user_led->timerId);
            /* Turn OFF LED */
            Cy_GPIO_Set(user_led->gpioPort, user_led->gpioPin);
#endif /* APP_FW_LED_ENABLE */
        case APP_EVT_HARD_RESET_COMPLETE:
        case APP_EVT_HR_SENT_RCVD_DEFERRED:
        case APP_EVT_HARD_RESET_SENT:
        case APP_EVT_PE_DISABLED:
        case APP_EVT_HARD_RESET_RCVD:
        case APP_EVT_VBUS_PORT_DISABLE:
        case APP_EVT_TYPE_C_ERROR_RECOVERY:
            glSrcCapChngDpmCmdPending[ctx->port] = false;
            break;

        case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:
            contract_status = (cy_stc_pdstack_pd_contract_info_t*)data;

            if (contract_status->status == CY_PDSTACK_CONTRACT_REJECT_NO_CONTRACT)
            {
                /* The solution is expected to update the source capabilities
                 * and trigger the negotiation for a new contract if port
                 * partner's request has been rejected. */
                glSrcCapChngDpmCmdPending[ctx->port] = true;
            }
            break;

        default:
            break;
    }
}

/*******************************************************************************
* Function Name: soln_sleep
********************************************************************************
* Summary:
*   Function checks if the solution layer is idle
*
* Parameters:
*  None
*
* Return:
*  True if the solution is idle otherwise false
*
*******************************************************************************/
bool soln_sleep ()
{
    bool retVal = true;

    if (glSrcCapChngDpmCmdPending[0])
    {
        retVal = false;
    }

#if PMG1_PD_DUALPORT_ENABLE
    if (glSrcCapChngDpmCmdPending[1])
    {
        retVal = false;
    }
#endif /* PMG1_PD_DUALPORT_ENABLE */

    return retVal;
}

/*******************************************************************************
* Function Name: instrumentation_cb
********************************************************************************
* Summary:
*  Callback function for handling instrumentation faults
*
* Parameters:
*  port - Port
*  evt - Event
*
* Return:
*  None
*
*******************************************************************************/
void instrumentation_cb(uint8_t port, uint8_t evt)
{
    uint8_t evt_offset = APP_TOTAL_EVENTS;
    evt += evt_offset;
    sln_pd_event_handler(&gl_PdStackPort0Ctx, (cy_en_pdstack_app_evt_t)evt, NULL);
}

/*******************************************************************************
* Function Name: wdt_interrupt_handler
********************************************************************************
* Summary:
*  Interrupt Handler for Watch Dog Timer
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void wdt_interrupt_handler(void)
{
    /* Clear WDT pending interrupt */
    Cy_WDT_ClearInterrupt();

#if (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0)
    /* Load the timer match register. */
    Cy_WDT_SetMatch((Cy_WDT_GetCount() + gl_TimerCtx.multiplier));
#endif /* (CY_PDUTILS_TIMER_TICKLESS_ENABLE == 0) */

    /* Invoke the timer handler. */
    Cy_PdUtils_SwTimer_InterruptHandler(&(gl_TimerCtx));
}

/*******************************************************************************
* Function Name: cy_usbpd0_intr0_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD0 Interrupt 0
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd0_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort0Ctx);
}

/*******************************************************************************
* Function Name: cy_usbpd0_intr1_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD0 Interrupt 1
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd0_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort0Ctx);
}

#if PMG1_PD_DUALPORT_ENABLE
/*******************************************************************************
* Function Name: cy_usbpd1_intr0_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD1 Interrupt 0
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd1_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&gl_UsbPdPort1Ctx);
}

/*******************************************************************************
* Function Name: cy_usbpd1_intr1_handler
********************************************************************************
* Summary:
*  Interrupt Handler for USBPD1 Interrupt 1
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void cy_usbpd1_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&gl_UsbPdPort1Ctx);
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

/*******************************************************************************
* Function Name: get_dpm_connect_stat
********************************************************************************
* Summary:
*  Gets the DPM configuration for Port 0
*
* Parameters:
*  None
*
* Return:
*  cy_stc_pd_dpm_config_t
*
*******************************************************************************/
cy_stc_pd_dpm_config_t* get_dpm_connect_stat(void)
{
    return &(gl_PdStackPort0Ctx.dpmConfig);
}

#if PMG1_PD_DUALPORT_ENABLE
/*******************************************************************************
* Function Name: get_dpm_port1_connect_stat
********************************************************************************
* Summary:
*  Gets the DPM configuration for Port 1
*
* Parameters:
*  None
*
* Return:
*  cy_stc_pd_dpm_config_t
*
*******************************************************************************/
cy_stc_pd_dpm_config_t* get_dpm_port1_connect_stat()
{
    return &(gl_PdStackPort1Ctx.dpmConfig);
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

/*
 * Application callback functions for the DPM. Since this application
 * uses the functions provided by the stack, loading the stack defaults.
 */
const cy_stc_pdstack_app_cbk_t app_callback =
{
    .app_event_handler = Cy_App_EventHandler,
#if (!CY_PD_SINK_ONLY)
    .psrc_set_voltage = Cy_App_Source_SetVoltage,
    .psrc_set_current = Cy_App_Source_SetCurrent,
    .psrc_enable = Cy_App_Source_Enable,
    .psrc_disable = Cy_App_Source_Disable,
#endif  /* (!CY_PD_SINK_ONLY) */
    .vconn_enable = Cy_App_VconnEnable,
    .vconn_disable = Cy_App_VconnDisable,
    .vconn_is_present = Cy_App_VconnIsPresent,
    .vbus_is_present = Cy_App_VbusIsPresent,
    .vbus_discharge_on = Cy_App_VbusDischargeOn,
    .vbus_discharge_off = Cy_App_VbusDischargeOff,
#if (!(CY_PD_SOURCE_ONLY))
    .psnk_set_voltage = Cy_App_Sink_SetVoltage,
    .psnk_set_current = Cy_App_Sink_SetCurrent,
    .psnk_enable = Cy_App_Sink_Enable,
    .psnk_disable = Cy_App_Sink_Disable,
    .eval_src_cap = Cy_App_Pdo_EvalSrcCap,
#endif /* (!(CY_PD_SOURCE_ONLY)) */
#if (!CY_PD_SINK_ONLY)
    .eval_rdo = Cy_App_Pdo_EvalRdo,
#endif  /* (!CY_PD_SINK_ONLY) */
    .eval_dr_swap = Cy_App_Swap_EvalDrSwap,
    .eval_pr_swap = Cy_App_Swap_EvalPrSwap,
    .eval_vconn_swap = Cy_App_Swap_EvalVconnSwap,
#if (!CY_PD_VDM_DISABLE)
    .eval_vdm = Cy_App_Vdm_EvalVdmMsg,
#endif /* (!CY_PD_VDM_DISABLE) */
#if (CY_PD_REV3_ENABLE && CY_PD_FRS_TX_ENABLE)
#if ((!(CY_PD_SOURCE_ONLY)) && (!CY_PD_SINK_ONLY))
    .eval_fr_swap = Cy_App_Swap_EvalFrSwap,
#endif /* ((!(CY_PD_SOURCE_ONLY)) && (!CY_PD_SINK_ONLY))  */
#endif /* CY_PD_REV3_ENABLE && CY_PD_FRS_TX_ENABLE */
    .vbus_get_value = Cy_App_VbusGetValue,
#if (!CY_PD_SINK_ONLY)
    .psrc_get_voltage = Cy_App_Source_GetVoltage,
#endif  /* (!CY_PD_SINK_ONLY) */
#if CY_PD_USB4_SUPPORT_ENABLE
   NULL,
#endif /* CY_PD_USB4_SUPPORT_ENABLE */
#if (!CY_PD_SINK_ONLY)
    .send_src_info = Cy_App_SendSrcInfo
#endif /* (!CY_PD_SINK_ONLY) */
};

#if CCG_TYPE_A_PORT_ENABLE
/*
 * Solution level callback functions for the application layer
 */
const cy_app_sln_cbk_t sln_cbk =
{
    .type_a_is_idle = type_a_is_idle,
    .type_a_update_status = type_a_update_status,
    .type_a_set_voltage = type_a_set_voltage,
    .type_a_enable_disable_vbus = type_a_enable_disable_vbus,
    .type_a_port_enable_disable = type_a_port_enable_disable,
    .type_a_detect_disconnect = type_a_detect_disconnect
};
#endif /* CCG_TYPE_A_PORT_ENABLE */

/*******************************************************************************
* Function Name: app_get_callback_ptr
********************************************************************************
* Summary:
*  Returns pointer to the structure holding the application callback functions
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  cy_stc_pdstack_app_cbk_t
*
*******************************************************************************/
cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t * context)
{
    (void)context;
    /* Solution callback pointer is same for all ports */
    return ((cy_stc_pdstack_app_cbk_t *)(&app_callback));
}

/*******************************************************************************
* Function Name: soln_set_volt_port1
********************************************************************************
* Summary:
*  Sets the regulator output voltage on port 1
*
* Parameters:
*  vol_in_mv - Voltage in mv
*
* Return:
*  None
*
*******************************************************************************/
void soln_set_volt_port1 (uint16_t vol_in_mv)
{
#if (defined(CY_DEVICE_PMG1S3) || defined(CY_DEVICE_CCG6))
    ncp_81239_set_volt (&gl_ncp81239ContextPort1, vol_in_mv);
#elif defined(CY_DEVICE_CCG3)
    uint32_t intr_state = Cy_SysLib_EnterCriticalSection ();
    switch (vol_in_mv)
        {
            case CY_PD_VSAFE_9V:
                APP_VBUS_SET_9V_P1();
                break;
            case CY_PD_VSAFE_12V:
                APP_VBUS_SET_12V_P1();
                break;
            case CY_PD_VSAFE_13V:
                APP_VBUS_SET_13V_P1();
                break;
            case CY_PD_VSAFE_15V:
                APP_VBUS_SET_15V_P1();
                break;
            case CY_PD_VSAFE_19V:
                APP_VBUS_SET_19V_P1();
                break;
            case CY_PD_VSAFE_20V:
                APP_VBUS_SET_20V_P1();
                break;
            default:
                Cy_App_GetStatus(0)->psrc_volt = CY_PD_VSAFE_5V;
                APP_VBUS_SET_5V_P1();
                break;
        }
    Cy_SysLib_ExitCriticalSection(intr_state);
#elif defined(CY_DEVICE_CCG3PA)
    /* Do nothing */
#else
    CY_ASSERT(0);
#endif /* (defined(CY_DEVICE_PMG1S3) || defined(CY_DEVICE_CCG6)) */
}

/*******************************************************************************
* Function Name: soln_set_volt_port2
********************************************************************************
* Summary:
*  Sets the regulator output voltage on port 2
*
* Parameters:
*  vol_in_mv - Voltage in mv
*
* Return:
*  None
*
*******************************************************************************/
#if PMG1_PD_DUALPORT_ENABLE
void soln_set_volt_port2 (uint16_t vol_in_mv)
{
    ncp_81239_set_volt (&gl_ncp81239ContextPort2, vol_in_mv);
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

/*******************************************************************************
* Function Name: soln_sink_fet_off
********************************************************************************
* Summary:
*  Turns off the consumer FET
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  None
*
*******************************************************************************/
void soln_sink_fet_off(cy_stc_pdstack_context_t * context)
{
#if defined(CY_DEVICE_PMG1S3)
    if (context->port == 0u)
    {
        Cy_GPIO_Clr (PFET_SNK_CTRL_P0_PORT, PFET_SNK_CTRL_P0_PIN);
    }
#if PMG1_PD_DUALPORT_ENABLE
    else
    {
        Cy_GPIO_Clr (PFET_SNK_CTRL_P1_PORT, PFET_SNK_CTRL_P1_PIN);
    }
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* CY_DEVICE_PMG1S3 */
}

/*******************************************************************************
* Function Name: soln_sink_fet_on
********************************************************************************
* Summary:
*  Turns on the consumer FET
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  None
*
*******************************************************************************/
void soln_sink_fet_on(cy_stc_pdstack_context_t * context)
{
#if defined(CY_DEVICE_PMG1S3)
    if (context->port == 0u)
    {
        Cy_GPIO_Set (PFET_SNK_CTRL_P0_PORT, PFET_SNK_CTRL_P0_PIN);
    }
#if PMG1_PD_DUALPORT_ENABLE
    else
    {
        Cy_GPIO_Set (PFET_SNK_CTRL_P1_PORT, PFET_SNK_CTRL_P1_PIN);
    }
#endif /* PMG1_PD_DUALPORT_ENABLE */
#endif /* CY_DEVICE_PMG1S3 */
}

/*******************************************************************************
* Function Name: soln_vbus_fet_on
********************************************************************************
* Summary:
*  Turns on the provider FET
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  None
*
*******************************************************************************/
void soln_vbus_fet_on (cy_stc_pdstack_context_t *context)
{
    (void)context;
#if defined(CY_DEVICE_CCG3PA)
    Cy_GPIO_Write(REG_EN_PORT, REG_EN_PIN, 0u);
#endif /* defined(CY_DEVICE_CCG3PA) */
}

/*******************************************************************************
* Function Name: soln_vbus_fet_off
********************************************************************************
* Summary:
*  Turns off the provider FET
*
* Parameters:
*  context - PD Stack Context
*
* Return:
*  None
*
*******************************************************************************/
void soln_vbus_fet_off (cy_stc_pdstack_context_t *context)
{
    (void)context;
#if defined(CY_DEVICE_CCG3PA)
    Cy_GPIO_Write(REG_EN_PORT, REG_EN_PIN, 1u);
#endif /* defined(CY_DEVICE_CCG3PA) */
}

/*******************************************************************************
* Function Name: soln_task
********************************************************************************
* Summary:
*  Solution layer task
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void soln_task ()
{
    uint8_t i = 0;

    for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
    {
        if (glSrcCapChngDpmCmdPending[i] == true)
        {
            /* The solution is expected to update the source capabilities and
             * trigger the negotiation for a new contract if port partner's
             * request has been rejected. */
            if (Cy_PdStack_Dpm_SendPdCommand(get_pdstack_context(i), CY_PDSTACK_DPM_CMD_SRC_CAP_CHNG,  NULL, false, NULL) == CY_PDSTACK_STAT_SUCCESS)
            {
                glSrcCapChngDpmCmdPending[i] = false;
            }
        }
    }
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - Initial setup of device
*  - Enables Watchdog timer, USBPD interrupts
*  - Initializes USBPD block and PDStack
*  - Runs device policy tasks and application level tasks
*
* Parameters:
*  None
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cy_stc_pdutils_timer_config_t timerConfig;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    sln_pd_event_handler(&gl_PdStackPort0Ctx, APP_EVT_POWER_CYCLE, NULL);
    /*
     * Register the interrupt handler for the watchdog timer. This timer is used to
     * implement the soft timers required by the USB-PD Stack.
     */
    Cy_SysInt_Init(&wdt_interrupt_config, &wdt_interrupt_handler);
    NVIC_EnableIRQ(wdt_interrupt_config.intrSrc);

    timerConfig.sys_clk_freq = Cy_SysClk_ClkSysGetFrequency();
    timerConfig.hw_timer_ctx = NULL;

    /* Initialize the soft timer module. */
    Cy_PdUtils_SwTimer_Init(&gl_TimerCtx, &timerConfig);

    /* Enable global interrupts */
    __enable_irq();

#if (defined(CY_DEVICE_PMG1S3) || defined(CY_DEVICE_CCG6))
    /* Initialize the I2C interface to the NCP81239 controller */
    Cy_SCB_I2C_Init (I2CM_HW, &I2CM_config, &gl_i2cm_context);
    Cy_SCB_I2C_Enable(I2CM_HW, &gl_i2cm_context);

    /* Enable NCP81239 controller */
    ncp_81239_init (&gl_ncp81239ContextPort1);
#endif /* (defined(CY_DEVICE_PMG1S3) || defined(CY_DEVICE_CCG6)) */
#if PMG1_PD_DUALPORT_ENABLE
    ncp_81239_init (&gl_ncp81239ContextPort2);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Initialize the instrumentation related data structures. */
    Cy_App_Instrumentation_Init(&gl_TimerCtx);

    /* Register callback function to be executed when instrumentation fault occurs. */
    Cy_App_Instrumentation_RegisterCb((cy_app_instrumentation_cb_t )instrumentation_cb);

    /* Configure and enable the USBPD interrupts */
    Cy_SysInt_Init(&usbpd_port0_intr0_config, &cy_usbpd0_intr0_handler);
    NVIC_EnableIRQ(usbpd_port0_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port0_intr1_config, &cy_usbpd0_intr1_handler);
    NVIC_EnableIRQ(usbpd_port0_intr1_config.intrSrc);

#if PMG1_PD_DUALPORT_ENABLE
    /* Configure and enable the USBPD interrupts for Port #1. */
    Cy_SysInt_Init(&usbpd_port1_intr0_config, &cy_usbpd1_intr0_handler);
    NVIC_EnableIRQ(usbpd_port1_intr0_config.intrSrc);

    Cy_SysInt_Init(&usbpd_port1_intr1_config, &cy_usbpd1_intr1_handler);
    NVIC_EnableIRQ(usbpd_port1_intr1_config.intrSrc);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Initialize the USBPD driver */
#if defined(CY_DEVICE_CCG3)
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, NULL,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);
#else
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, mtb_usbpd_port0_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_connect_stat);
#endif

#if PMG1_PD_DUALPORT_ENABLE
    Cy_USBPD_Init(&gl_UsbPdPort1Ctx, 1, mtb_usbpd_port1_HW, mtb_usbpd_port1_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port1_config, get_dpm_port1_connect_stat);
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if CCG_TYPE_A_PORT_ENABLE
    /* Initialize the port, base and legacy charging configuration for type-A port. */
    gl_TypeAportCtx.port = 1u;
    gl_TypeAportCtx.base = mtb_usbpd_port0_HW;
    gl_TypeAportCtx.usbpdConfig->legacyChargingConfig = &mtb_usbpd_port0_type_A_config;
    /* Initialize the alternate port PD context in port-0 USBPD context structure. */
    gl_UsbPdPort0Ctx.altPortUsbPdCtx[TYPEC_PORT_0_IDX] = &gl_TypeAportCtx;
    /* Register solution callback */
    Cy_App_RegisterSlnCallback(NULL, (cy_app_sln_cbk_t *)&sln_cbk);
    /* Initialize the PWM for regulator control */
    Cy_TCPWM_PWM_Init(PWM_A_HW, PWM_A_NUM, &PWM_A_config);
#endif /* CCG_TYPE_A_PORT_ENABLE */

    /* Initialize the Device Policy Manager. */
    Cy_PdStack_Dpm_Init(&gl_PdStackPort0Ctx,
                       &gl_UsbPdPort0Ctx,
                       &mtb_usbpd_port0_pdstack_config,
                       app_get_callback_ptr(&gl_PdStackPort0Ctx),
                       &pdstack_port0_dpm_params,
                       &gl_TimerCtx);

#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdStack_Dpm_Init(&gl_PdStackPort1Ctx,
                       &gl_UsbPdPort1Ctx,
                       &mtb_usbpd_port1_pdstack_config,
                       app_get_callback_ptr(&gl_PdStackPort1Ctx),
                       &pdstack_port1_dpm_params,
                       &gl_TimerCtx);
    /* Send NOT_SUPPORTED for DATA RESET message. */
    Cy_PdStack_Dpm_SetDataReset (&gl_PdStackPort1Ctx, false);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Perform application level initialization. */
    Cy_App_Init(&gl_PdStackPort0Ctx, &port0_app_params);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_App_Init(&gl_PdStackPort1Ctx, &port1_app_params);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Initialize the fault configuration values */
    Cy_App_Fault_InitVars(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_App_Fault_InitVars(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

    /* Start any timers or tasks associated with application instrumentation. */
    Cy_App_Instrumentation_Start();

    /* Start the device policy manager operation. This will initialize the USB-PD block and enable connect detection. */
    Cy_PdStack_Dpm_Start(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    Cy_PdStack_Dpm_Start(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

#if defined(CY_DEVICE_CCG3PA)
    /* Not in dead battery condition, disable the internal regulator */
    if(gl_PdStackPort0Ctx.dpmStat.deadBat != true)
    {
        Cy_USBPD_DisableVsysReg(&gl_UsbPdPort0Ctx);
    }
#endif /* defined(CY_DEVICE_CCG3PA) */

    /*
     * After the initialization is complete, keep processing the USB-PD device policy manager task in a loop.
     * Since this application does not have any other function, the PMG1 device can be placed in "deep sleep"
     * mode for power saving whenever the PD stack and drivers are idle.
     */
#if !CY_PD_SINK_ONLY
    gl_PdStackPort0Ctx.dpmStat.srcCapStartDelay = DELAY_SRC_CAP_START_MS;
#endif /* !CY_PD_SINK_ONLY */

    for (;;)
    {
        /* Handle the device policy tasks for each PD port. */
        Cy_PdStack_Dpm_Task(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
        Cy_PdStack_Dpm_Task(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

        /* Perform any application level tasks. */
        Cy_App_Task(&gl_PdStackPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
        Cy_App_Task(&gl_PdStackPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE */

        /* Perform tasks associated with instrumentation. */
        Cy_App_Instrumentation_Task();

        /* Perform solution level tasks. */
        soln_task();

#if SYS_DEEPSLEEP_ENABLE
        /* If possible, enter deep sleep mode for power saving. */
        Cy_App_SystemSleep(&gl_PdStackPort0Ctx,
#if PMG1_PD_DUALPORT_ENABLE
                &gl_PdStackPort1Ctx
#else
                NULL
#endif /* PMG1_PD_DUALPORT_ENABLE */
                );
#endif /* SYS_DEEPSLEEP_ENABLE */
    }
}

/* [] END OF FILE */
