/***************************************************************************//**
* \file ncp_81239.c
* \version 1.0
*
* \brief
* Implements functions associated with NCP81239 buck-boost controller
*
********************************************************************************
* \copyright
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company)
* or an affiliate of Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "ncp_81239.h"
#include "cy_app_i2c_master.h"

/* Initialize NCP81239 buck-boost controller */
bool ncp_81239_init (ncp_81239_context_t *context)
{
    bool status = false;
    uint8_t reg_addr[3] = {PD_CTRL_SKEW_RATE_REG_ADDR, PD_CTRL_VPS_REG_ADDR, PD_CTRL_PWM_FREQ_ADDR};
    uint8_t reg_value[3] = {PD_CTRL_SKEW_RATE_4_9_MV_US, (PD_CTRL_VPS_5V + NCP_REG_EXCESS_VOLTAGE), PD_CTRL_PWM_FREQ_300KHZ};

    if (context->enableGpioPort)
    {
        Cy_GPIO_Write(context->enableGpioPort, context->enableGpioPin, 1U);
        Cy_SysLib_Delay(1u);
    }

    for (uint8_t itr = 0; itr < sizeof(reg_addr); itr++)
    {
        status = Cy_App_I2CMaster_RegWrite(context->scbBase, context->i2cAddr, \
                                           &reg_addr[itr], 1u, &reg_value[itr], 1u, context->i2cContext);
    }

    return status;
}

/* Set the output voltage */
bool ncp_81239_set_volt (ncp_81239_context_t *context, uint16_t volt_in_mv)
{
    bool status = false;
    uint8_t reg_addr;
    uint8_t reg_value;

    reg_addr = PD_CTRL_VPS_REG_ADDR;

    reg_value = (volt_in_mv / NCP_REG_VOLT_RESOLUTION) + NCP_REG_EXCESS_VOLTAGE;

    status = Cy_App_I2CMaster_RegWrite(context->scbBase, context->i2cAddr, \
                                &reg_addr, 1u, &reg_value, 1u, context->i2cContext);

    return status;
}

/* Enable NCP81239 */
bool ncp_81239_enable(ncp_81239_context_t *context)
{
    bool status = false;
    uint8_t reg_addr;
    uint8_t reg_value;

    reg_addr = PD_CTRL_EN_ADDR;
    reg_value = (1u << PD_CTRL_EN_MASK_POS) | (1u << PD_CTRL_EN_INT_POS);

    status = Cy_App_I2CMaster_RegWrite(context->scbBase, context->i2cAddr, \
                                    &reg_addr, 1u, &reg_value, 1u, context->i2cContext);
    return(status);
}

/* Disable NCP81239 */
bool ncp_81239_disable(ncp_81239_context_t *context)
{
    bool status = false;
    uint8_t reg_addr;
    uint8_t reg_value;

    reg_addr = PD_CTRL_EN_ADDR;

    status = Cy_App_I2CMaster_RegRead(context->scbBase, context->i2cAddr, \
                                    &reg_addr, 1u, &reg_value, 1u, context->i2cContext);
    if(status == true)
    {
        /* Clearing the mask and the interrupt bit */
        reg_value = reg_value & (~((1u << PD_CTRL_EN_MASK_POS) | (1u << PD_CTRL_EN_INT_POS)));

        status = Cy_App_I2CMaster_RegWrite(context->scbBase, context->i2cAddr, \
                                        &reg_addr, 1u, &reg_value, 1u, context->i2cContext);
    }

    return(status);
}

/* Sets the slew rate */
bool ncp_81239_set_slew_rate(ncp_81239_context_t *context, ncp_81239_slew_rates_t slew_rate)
{
    bool status = false;
    uint8_t reg_addr;
    uint8_t reg_value;

    reg_addr = PD_CTRL_SKEW_RATE_REG_ADDR;
    reg_value = slew_rate;

    status = Cy_App_I2CMaster_RegWrite(context->scbBase, context->i2cAddr, \
                                    &reg_addr, 1u, &reg_value, 1u, context->i2cContext);

    return(status);
}

/* [] END OF FILE */
