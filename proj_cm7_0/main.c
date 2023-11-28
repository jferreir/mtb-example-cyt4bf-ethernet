/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for CM7_0 in the the Multi Core Empty
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
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
*******************************************************************************/
#include <inttypes.h>

#include "cyhal.h"
#include "cybsp.h"
#include "cy_log.h"

#include "cy_retarget_io.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "cy_ecm.h"

#define ETH_TASK_SIZE              (1024 * 16)
#define ETH_TASK_PRIORITY          (configMAX_PRIORITIES - 2)

/* Maximum number of connection retries to the ethernet network */
#define MAX_ETH_RETRY_COUNT                     (3u)

/* Ethernet connection manager handle */
static cy_ecm_t g_ecmHandle = NULL;

cy_rslt_t connectToEthernet(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    uint8_t retry_count = 0;

    /* Variables used by Ethernet connection manager.*/
    cy_ecm_phy_config_t ecm_phy_config;
    cy_ecm_ip_address_t ip_addr;

#if ENABLE_STATIC_IP_ADDRESS
    cy_ecm_ip_setting_t static_ip_addr;

    static_ip_addr.ip_address.version = CY_ECM_IP_VER_V4;
    static_ip_addr.ip_address.ip.v4 = STATIC_IP_ADDR;
    static_ip_addr.gateway.version = CY_ECM_IP_VER_V4;
    static_ip_addr.gateway.ip.v4 = STATIC_GATEWAY;
    static_ip_addr.netmask.version = CY_ECM_IP_VER_V4;
    static_ip_addr.netmask.ip.v4 = NETMASK;
#endif

    ecm_phy_config.interface_speed_type = CY_ECM_SPEED_TYPE_RMII;
    ecm_phy_config.mode = CY_ECM_DUPLEX_AUTO;
    ecm_phy_config.phy_speed = CY_ECM_PHY_SPEED_AUTO;

    /* Initialize ethernet connection manager */
    result = cy_ecm_init();
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Ethernet connection manager initialization failed! Error code: 0x%08"PRIx32"\n", (uint32_t)result);
        CY_ASSERT(0);
    }
    else
    {
        printf("Ethernet connection manager initialized.\n");
    }

    /* To change the MAC address, enter the desired MAC as the second parameter in cy_ecm_ethif_init() instead of NULL.
     * Default MAC address(00-03-19-45-00-00) is used when NULL is passed.
     */
    result =  cy_ecm_ethif_init(CY_ECM_INTERFACE_ETH0, NULL, &ecm_phy_config, &g_ecmHandle);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Ethernet interface initialization failed! Error code: 0x%08"PRIx32"\n", (uint32_t)result);
        CY_ASSERT(0);
    }
    
    /* Establish a connection to the ethernet network */
    while (1)
    {
#if ENABLE_STATIC_IP_ADDRESS
        /* Connect to the ethernet network with the assigned static IP address */
        result = cy_ecm_connect(g_ecmHandle, &static_ip_addr, &ip_addr);
#else
        /* Connect to the ethernet network with the dynamically allocated IP address by DHCP */
        result = cy_ecm_connect(g_ecmHandle, NULL, &ip_addr);
#endif
        if (result != CY_RSLT_SUCCESS)
        {
            retry_count++;
            if (retry_count >= MAX_ETH_RETRY_COUNT)
            {
                printf("Exceeded max ethernet connection attempts\n");
                return result;
            }
            printf("Connection to ethernet network failed. Retrying...\n");
            continue;
        }
        else
        {
            printf("Successfully connected to Ethernet.\n");
            printf("IP Address Assigned: %d.%d.%d.%d \n", (uint8)ip_addr.ip.v4,(uint8)(ip_addr.ip.v4 >> 8), (uint8)(ip_addr.ip.v4 >> 16),
                                (uint8)(ip_addr.ip.v4 >> 24));
#if ENABLE_STATIC_IP_ADDRESS
            ip_addr_t dnsserver;
            dnsserver.type = IPADDR_TYPE_V4;
            dnsserver.u_addr.ip4.addr = DNS_SERVER;
            dns_setserver(0, &dnsserver);
#endif
            break;
        }
    }
    return result;
}

void eth_task(void *arg)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Connect to ethernet network */
    result = connectToEthernet();
    if (result != CY_RSLT_SUCCESS)
    {
        printf("\n Failed to connect to ethernet.\n");
        CY_ASSERT(0);
    }

    vTaskSuspend(NULL);
}

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port. */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    result = cy_log_init( CY_LOG_INFO, NULL, NULL );
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Create the FOTA main task. */
    xTaskCreate(eth_task, "ETH TASK", ETH_TASK_SIZE, NULL, ETH_TASK_PRIORITY, NULL);

    /* Start the FreeRTOS scheduler. */
    vTaskStartScheduler();

    /* Should never get here. */
    CY_ASSERT(0);
}

/* [] END OF FILE */
