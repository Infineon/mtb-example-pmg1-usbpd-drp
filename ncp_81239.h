/***************************************************************************//**
* \file ncp_81239.h
* \version 1.0
*
* \brief
* Defines data structures and function prototypes associated with
* NCP81239 buck-boost controller.
*
********************************************************************************
* \copyright
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company)
* or an affiliate of Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef _NCP_81239_H_
#define _NCP_81239_H_

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_scb_i2c.h"

/*******************************************************************************
 * Macro declarations
 ******************************************************************************/
#define PD_CTRL_SLAVE_ADDR              (0x74)

#define PD_CTRL_EN_ADDR                 (0x00u)
#define PD_CTRL_EN_POL_POS              (0u)
#define PD_CTRL_EN_POL                  (0u<<PD_CTRL_EN_POL_POS)    /*default*/
#define PD_CTRL_EN_PUP_POS              (1u)
#define PD_CTRL_EN_PUP                  (0u<<PD_CTRL_EN_PUP_POS)    /*default*/
#define PD_CTRL_EN_MASK_POS             (2u)
#define PD_CTRL_EN_MASK                 (0u<<PD_CTRL_EN_MASK_POS)   /*default*/
#define PD_CTRL_EN_INT_POS              (3u)
#define PD_CTRL_EN_INT                  (0u<<PD_CTRL_EN_INT_POS)    /*default*/
#define PD_CTRL_EN_REACTION_POS         (4u)
#define PD_CTRL_EN_REACTION             (0u<<PD_CTRL_EN_REACTION_POS)   /*default*/

#define PD_CTRL_VPS_REG_ADDR            (0x01u)
#define PD_CTRL_VPS_0V                  (0x00u) /*default*/
#define PD_CTRL_VPS_5V                  (0x32u) /* 5 V */
#define PD_CTRL_VPS_20V                 (0xC8u) /* 20 V */

#define PD_CTRL_SKEW_RATE_REG_ADDR      (0x02u)

#define PD_CTRL_PWM_FREQ_ADDR           (0x03u)
#define PD_CRTL_PWM_FREQ_MASK           (0x07u)
#define PD_CTRL_PWM_FREQ_VPS_STEP_POS   (4u)
#define PD_CTRL_PWM_FREQ_VPS_STEP_10MV  (0<<PD_CTRL_PWM_FREQ_VPS_STEP_POS)  /*default*/
#define PD_CTRL_PWM_FREQ_VPS_STEP_5MV   (1<<PD_CTRL_PWM_FREQ_VPS_STEP_POS)

#define PD_CTRL_FET_ADDR                (0x04u)
#define PD_CTRL_FET_PFET_POS            (0u)
#define PD_CTRL_FET_PFET_DIS            (0u<<PD_CTRL_FET_PFET_POS)  /*default*/
#define PD_CTRL_FET_PFET_EN             (1u<<PD_CTRL_FET_PFET_POS)
#define PD_CTRL_FET_CFET_POS            (1u)
#define PD_CTRL_FET_CFET_DIS            (0u<<PD_CTRL_FET_CFET_POS)  /*default*/
#define PD_CTRL_FET_CFET_EN             (1u<<PD_CTRL_FET_CFET_POS)
#define PD_CTRL_FET_DB_POS              (2u)
#define PD_CTRL_FET_DB_DIS              (0u<<PD_CTRL_FET_DB_POS)    /*default*/
#define PD_CTRL_FET_DB_EN               (1u<<PD_CTRL_FET_DB_POS)
#define PD_CTRL_FET_CS1_DCHRG_POS       (4u)
#define PD_CTRL_FET_CS1_DCHRG_DIS       (0u<<PD_CTRL_FET_CS1_DCHRG_POS) /*default*/
#define PD_CTRL_FET_CS1_DCHRG_EN        (1u<<PD_CTRL_FET_CS1_DCHRG_POS)
#define PD_CTRL_FET_CS2_DCHRG_POS       (5u)
#define PD_CTRL_FET_CS2_DCHRG_DIS       (0u<<PD_CTRL_FET_CS2_DCHRG_POS) /*default*/
#define PD_CTRL_FET_CS2_DCHRG_EN        (1u<<PD_CTRL_FET_CS2_DCHRG_POS)

#define PD_CTRL_OCP_CLIM_ADDR           (0x05u)
#define PD_CTRL_OCP_CLIMP_POS           (0u)
#define PD_CTRL_OCP_CLIMP_7_6A          (0u<<PD_CTRL_OCP_CLIMP_POS) /*default*/
#define PD_CTRL_OCP_CLIMP_4_6A          (1u<<PD_CTRL_OCP_CLIMP_POS)
#define PD_CTRL_OCP_CLIMP_2_2A          (2u<<PD_CTRL_OCP_CLIMP_POS)
#define PD_CTRL_OCP_CLIMP_10A           (3u<<PD_CTRL_OCP_CLIMP_POS)
#define PD_CTRL_OCP_CLIMN_POS           (4u)
#define PD_CTRL_OCP_CLIMN_MINUS_8A      (0u<<PD_CTRL_OCP_CLIMN_POS) /*default*/
#define PD_CTRL_OCP_CLIMN_MINUS_5A      (1u<<PD_CTRL_OCP_CLIMN_POS)
#define PD_CTRL_OCP_CLIMN_MINUS_3A      (2u<<PD_CTRL_OCP_CLIMN_POS)
#define PD_CTRL_OCP_CLIMN_0A            (3u<<PD_CTRL_OCP_CLIMN_POS)

#define PD_CTRL_CS_CLIM_ADDR            (0x06u)
#define PD_CTRL_CS1_CLIM_POS            (0u)
#define PD_CTRL_CS1_CLIM_0_5A           (0u<<PD_CTRL_CS1_CLIM_POS)  /*default*/
#define PD_CTRL_CS1_CLIM_1_5A           (1u<<PD_CTRL_CS1_CLIM_POS)
#define PD_CTRL_CS1_CLIM_3A             (2u<<PD_CTRL_CS1_CLIM_POS)
#define PD_CTRL_CS1_CLIM_5A             (3u<<PD_CTRL_CS1_CLIM_POS)
#define PD_CTRL_CS2_CLIM_POS            (2u)
#define PD_CTRL_CS2_CLIM_0_5A           (0u<<PD_CTRL_CS2_CLIM_POS)  /*default*/
#define PD_CTRL_CS2_CLIM_1_5A           (1u<<PD_CTRL_CS2_CLIM_POS)
#define PD_CTRL_CS2_CLIM_3A             (2u<<PD_CTRL_CS2_CLIM_POS)
#define PD_CTRL_CS2_CLIM_5A             (3u<<PD_CTRL_CS2_CLIM_POS)
#define PD_CTRL_CS_CLIM_EN_POS          (4u)
#define PD_CTRL_CS_CLIM_EN              (0u<<PD_CTRL_CS_CLIM_EN_POS)    /*default */

#define PD_CTRL_GM_ADDR                 (0x07u)
#define PD_CTRL_GM_AMPLO_POS            (0u)
#define PD_CTRL_GM_AMPLO_MASK           (0x07u)
#define PD_CTRL_GM_AMPLO_87US           (0u<<PD_CTRL_GM_AMPLO_POS)
#define PD_CTRL_GM_AMPLO_100US          (1u<<PD_CTRL_GM_AMPLO_POS)  /*default*/
#define PD_CTRL_GM_AMPLO_117US          (2u<<PD_CTRL_GM_AMPLO_POS)
#define PD_CTRL_GM_AMPLO_333US          (3u<<PD_CTRL_GM_AMPLO_POS)
#define PD_CTRL_GM_AMPLO_400US          (4u<<PD_CTRL_GM_AMPLO_POS)
#define PD_CTRL_GM_AMPLO_500US          (5u<<PD_CTRL_GM_AMPLO_POS)
#define PD_CTRL_GM_AMPLO_667US          (6u<<PD_CTRL_GM_AMPLO_POS)
#define PD_CTRL_GM_AMPLO_1000US         (7u<<PD_CTRL_GM_AMPLO_POS)
#define PD_CTRL_GM_AMPLO_DIS            (0<<3u)
#define PD_CTRL_GM_AMPLO_EN             (1<<3u) /*default*/
#define PD_CTRL_GM_AMPHI_POS            (0u)
#define PD_CTRL_GM_AMPHI_MASK           (0x07u)
#define PD_CTRL_GM_AMPHI_87US           (0u<<PD_CTRL_GM_AMPHI_POS)
#define PD_CTRL_GM_AMPHI_100US          (1u<<PD_CTRL_GM_AMPHI_POS)
#define PD_CTRL_GM_AMPHI_117US          (2u<<PD_CTRL_GM_AMPHI_POS)
#define PD_CTRL_GM_AMPHI_333US          (3u<<PD_CTRL_GM_AMPHI_POS)
#define PD_CTRL_GM_AMPHI_400US          (4u<<PD_CTRL_GM_AMPHI_POS)
#define PD_CTRL_GM_AMPHI_500US          (5u<<PD_CTRL_GM_AMPHI_POS)  /*default*/
#define PD_CTRL_GM_AMPHI_667US          (6u<<PD_CTRL_GM_AMPHI_POS)
#define PD_CTRL_GM_AMPHI_1000US         (7u<<PD_CTRL_GM_AMPHI_POS)
#define PD_CTRL_GM_AMPHI_DIS            (0<<7u) /*default*/
#define PD_CTRL_GM_AMPHI_EN             (1<<7u)

#define PD_CTRL_AMUX_ADDR               (0x08u)
#define PD_CTRL_AMUX_TRIG_POS           (0u)
#define PD_CTRL_AMUX_TRIG_SINGLE_FAULT  (0u<<PD_CTRL_AMUX_TRIG_POS) /*default*/
#define PD_CTRL_AMUX_TRIG_SINGLE        (1u<<PD_CTRL_AMUX_TRIG_POS)
#define PD_CTRL_AMUX_TRIG_CONTINUOUS    (2u<<PD_CTRL_AMUX_TRIG_POS)
#define PD_CTRL_AMUX_TRIG_RESERVED      (3u<<PD_CTRL_AMUX_TRIG_POS)
#define PD_CTRL_AMUX_SEL_POS            (2u)
#define PD_CTRL_AMUX_SEL_V2             (0u<<PD_CTRL_AMUX_SEL_POS)  /*default*/
#define PD_CTRL_AMUX_SEL_V1             (1u<<PD_CTRL_AMUX_SEL_POS)
#define PD_CTRL_AMUX_SEL_CS2            (2u<<PD_CTRL_AMUX_SEL_POS)
#define PD_CTRL_AMUX_SEL_CS1            (3u<<PD_CTRL_AMUX_SEL_POS)
#define PD_CTRL_AMUX_SEL_ROTATING       (4u<<PD_CTRL_AMUX_SEL_POS)
#define PD_CTRL_AMUX_ADC_POS            (5u)
#define PD_CTRL_AMUX_ADC_EN             (0u<<PD_CTRL_AMUX_ADC_POS)  /*default*/
#define PD_CTRL_AMUX_ADC_DIS            (1u<<PD_CTRL_AMUX_ADC_POS)

#define PD_CTRL_INT_MASK_ADDR           (0x09u)
#define PD_CTRL_INT_MASK_CS_CLIND       (0b00000001u)
#define PD_CTRL_INT_MASK_OVP            (0b00000010u)
#define PD_CTRL_INT_MASK_OCP_P          (0b00000100u)
#define PD_CTRL_INT_MASK_PG_INT         (0b00001000u)
#define PD_CTRL_INT_MASK_TSD            (0b00010000u)
#define PD_CTRL_INT_MASK_UVP            (0b00100000u)
#define PD_CTRL_INT_MASK_VCHN           (0b01000000u)
#define PD_CTRL_INT_MASK_I2C_ACK        (0b10000000u)

#define PD_CTRL_INT2_MASK_ADDR          (0x0Au)
#define PD_CTRL_INT2_MASK_SHUTDOWN      (0b00000001u)

#define PD_CTRL_ADC_VOUT_ADDR           (0x10u)
#define PD_CTRL_ADC_VIN_ADDR            (0x11u)
#define PD_CTRL_ADC_CS2_ADDR            (0x12u)
#define PD_CTRL_ADC_CS1_ADDR            (0x13u)

#define PD_CTRL_INT_ADDR                (0x14u)
#define PD_CTRL_INT_CS_CLIND            (0b00000001u)
#define PD_CTRL_INT_OVP                 (0b00000010u)
#define PD_CTRL_INT_OCP_P               (0b00000100u)
#define PD_CTRL_INT_PG_INT              (0b00001000u)
#define PD_CTRL_INT_TSD                 (0b00010000u)
#define PD_CTRL_INT_UVP                 (0b00100000u)
#define PD_CTRL_INT_VCHN                (0b01000000u)
#define PD_CTRL_INT_I2C_ACK             (0b10000000u)

#define PD_CTRL_INT2_ADDR               (0x15u)
#define PD_CTRL_INT2_SHUTDOWN           (0b00000001u)

/* NCP regulator I2C slave addresses for each PD port */
#define NCP_PORT_1_SLAVE_ADDR           (0x74u)
#define NCP_PORT_2_SLAVE_ADDR           (0x75u)

/* Resolution of voltage changes (in mV) supported by the NCP regulator */
#define NCP_REG_VOLT_RESOLUTION         (100u)

/*
 * Excess voltage to be configured to make up for in-system drops. The unit used
 * is the resolution supported by the regulator itself.
 */
#define NCP_REG_EXCESS_VOLTAGE          (1u)

/** NCP81239 buck-boost regulator data structure */
typedef struct ncp81239_context
{
    /** I2C 7-bit address */
    uint8_t i2cAddr;

    /** I2C SCB base address */
    CySCB_Type *scbBase;

    /** I2C SCB context pointer */
    cy_stc_scb_i2c_context_t *i2cContext;

    /** Controller enable pin GPIO address*/
    GPIO_PRT_Type *enableGpioPort;

    /** Controller enable pin GPIO number */
    uint8_t enableGpioPin;
}ncp_81239_context_t;

/**
 * @typedef ncp_81239_slew_rates_t
 * @brief Slew Rates for NCP81239 buck-boost controller
 */
typedef enum
{
    PD_CTRL_SKEW_RATE_0_61_MV_US, /**< 0 - Slew rate 0.61 mV */
    PD_CTRL_SKEW_RATE_1_2_MV_US,  /**< 1 - Slew rate 1.2 mV */
    PD_CTRL_SKEW_RATE_2_4_MV_US,  /**< 2 - Slew rate 2.4 mV */
    PD_CTRL_SKEW_RATE_4_9_MV_US   /**< 3 - Slew rate 4.9 mV */
}ncp_81239_slew_rates_t;

/**
 * @typedef ncp_81239_freq_t
 * @brief PWM frequencies for NCP81239 buck-boost controller
 */
typedef enum
{
    PD_CTRL_PWM_FREQ_600KHZ, /**< 0 - PWM frequency 600 kHz */
    PD_CTRL_PWM_FREQ_150KHZ, /**< 1 - PWM frequency 150 kHz */
    PD_CTRL_PWM_FREQ_300KHZ, /**< 2 - PWM frequency 300 kHz */
    PD_CTRL_PWM_FREQ_450KHZ, /**< 3 - PWM frequency 450 kHz */
    PD_CTRL_PWM_FREQ_750KHZ, /**< 4 - PWM frequency 750 kHz */
    PD_CTRL_PWM_FREQ_900KHZ, /**< 5 - PWM frequency 900 kHz */
    PD_CTRL_PWM_FREQ_1200KHZ, /**< 6 - PWM frequency 1200 kHz */
    PD_CTRL_PWM_FREQ_RESERVED /**< 7 - PWM frequency Reserved */
}ncp_81239_freq_t;

/**
 * @brief NCP81239 Power Delivery controller initialize
 * I2C SCB block should be initialized and all the structure member of
 * ncp_81239_context_t should be initialized before calling this function.
 * @param context - Pointer to the NCP81239 data structure context
 * @return Return true if operation is success; false otherwise.
 */
bool ncp_81239_init (ncp_81239_context_t *context);

/**
 * @brief Selects the output voltage
 * @param context - Pointer to the NCP81239 data structure context
 * @param volt_in_mv - Voltage unit in PDO (mV)
 * @return Return true if operation is success; false otherwise.
 */
bool ncp_81239_set_volt (ncp_81239_context_t *context, uint16_t volt_in_mv);

/**
 * @brief Enables NCP81239
 * @param context - Pointer to the NCP81239 data structure context
 * @return Return true if operation is success; false otherwise.
 */
bool ncp_81239_enable (ncp_81239_context_t *context);

/**
 * @brief Disables NCP81239
 * @param context - Pointer to the NCP81239 data structure context
 * @return Return true if operation is success; false otherwise.
 */
bool ncp_81239_disable (ncp_81239_context_t *context);

/**
 * @brief Sets the slew rate
 * @param context - Pointer to the NCP81239 data structure context
 * @param slew_rate - Slew Rate in mV
 * @return Return true if operation is success; false otherwise.
 */
bool ncp_81239_set_slew_rate (ncp_81239_context_t *context, ncp_81239_slew_rates_t slew_rate);

#endif /* _NCP_81239_H_ */

/* [] END OF FILE */
