/*
 * Copyright 2022, Cypress Semiconductor Corporation (an Infineon company) or
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
*/

/**
* @file cy_eth_user_config.h
* @brief This file provides platform specific configuration macros for Ethernet.
*/
#ifndef CY_ETH_USER_CONFIG
#define CY_ETH_USER_CONFIG

#include "cy_ethif.h"
#include "cy_ephy.h"
#include "cy_ecm.h"

#define CY_GIG_ETH_TYPE                        ETH0
#define CY_GIG_ETH_INSTANCE_NUM                (1)

#define ETH_REG_BASE                           CY_GIG_ETH_TYPE

#define CY_GIG_ETH_TX_CLK_PORT                 GPIO_PRT18
#define CY_GIG_ETH_TX_CLK_PIN                  3
#define CY_GIG_ETH_TX_CLK_PIN_MUX              P18_3_ETH0_TX_CLK

#define CY_GIG_ETH_TX_CTL_PORT                 GPIO_PRT18
#define CY_GIG_ETH_TX_CTL_PIN                  1
#define CY_GIG_ETH_TX_CTL_PIN_MUX              P18_1_ETH0_TX_CTL

#define CY_GIG_ETH_TD0_PORT                    GPIO_PRT18
#define CY_GIG_ETH_TD0_PIN                     4
#define CY_GIG_ETH_TD0_PIN_MUX                 P18_4_ETH0_TXD0

#define CY_GIG_ETH_TD1_PORT                    GPIO_PRT18
#define CY_GIG_ETH_TD1_PIN                     5
#define CY_GIG_ETH_TD1_PIN_MUX                 P18_5_ETH0_TXD1

#define CY_GIG_ETH_TD2_PORT                    GPIO_PRT18
#define CY_GIG_ETH_TD2_PIN                     6
#define CY_GIG_ETH_TD2_PIN_MUX                 P18_6_ETH0_TXD2

#define CY_GIG_ETH_TD3_PORT                    GPIO_PRT18
#define CY_GIG_ETH_TD3_PIN                     7
#define CY_GIG_ETH_TD3_PIN_MUX                 P18_7_ETH0_TXD3

#define CY_GIG_ETH_RX_CLK_PORT                 GPIO_PRT23
#define CY_GIG_ETH_RX_CLK_PIN                  3
#define CY_GIG_ETH_RX_CLK_PIN_MUX              P23_3_ETH0_RX_CLK

#define CY_GIG_ETH_RX_CTL_PORT                 GPIO_PRT21
#define CY_GIG_ETH_RX_CTL_PIN                  5
#define CY_GIG_ETH_RX_CTL_PIN_MUX              P21_5_ETH0_RX_CTL

#define CY_GIG_ETH_RD0_PORT                    GPIO_PRT19
#define CY_GIG_ETH_RD0_PIN                     0
#define CY_GIG_ETH_RD0_PIN_MUX                 P19_0_ETH0_RXD0

#define CY_GIG_ETH_RD1_PORT                    GPIO_PRT19
#define CY_GIG_ETH_RD1_PIN                     1
#define CY_GIG_ETH_RD1_PIN_MUX                 P19_1_ETH0_RXD1

#define CY_GIG_ETH_RD2_PORT                    GPIO_PRT19
#define CY_GIG_ETH_RD2_PIN                     2
#define CY_GIG_ETH_RD2_PIN_MUX                 P19_2_ETH0_RXD2

#define CY_GIG_ETH_RD3_PORT                    GPIO_PRT19
#define CY_GIG_ETH_RD3_PIN                     3
#define CY_GIG_ETH_RD3_PIN_MUX                 P19_3_ETH0_RXD3

#define CY_GIG_ETH_MDC_PORT                    GPIO_PRT3
#define CY_GIG_ETH_MDC_PIN                     1
#define CY_GIG_ETH_MDC_PIN_MUX                 P3_1_ETH0_MDC

#define CY_GIG_ETH_MDIO_PORT                   GPIO_PRT3
#define CY_GIG_ETH_MDIO_PIN                    0
#define CY_GIG_ETH_MDIO_PIN_MUX                P3_0_ETH0_MDIO

#define CY_GIG_ETH_REF_CLK_PORT                GPIO_PRT18
#define CY_GIG_ETH_REF_CLK_PIN                 0
#define CY_GIG_ETH_REF_CLK_PIN_MUX             P18_0_ETH0_REF_CLK

/* Setup IRQ source for 0, 1, and 2 priority queue */
#define CY_GIG_ETH_IRQN0                       eth_0_interrupt_eth_0_IRQn
#define CY_GIG_ETH_IRQN1                       eth_0_interrupt_eth_1_IRQn
#define CY_GIG_ETH_IRQN2                       eth_0_interrupt_eth_2_IRQn

#endif /* CY_ETH_USER_CONFIG */
