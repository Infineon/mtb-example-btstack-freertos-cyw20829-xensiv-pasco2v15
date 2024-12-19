/*******************************************************************************
 Copyright 2020-2024, Cypress Semiconductor Corporation (an Infineon company) or
 an affiliate of Cypress Semiconductor Corporation.  All rights reserved.

 This software, including source code, documentation and related
 materials ("Software") is owned by Cypress Semiconductor Corporation
 or one of its affiliates ("Cypress") and is protected by and subject to
 worldwide patent protection (United States and foreign),
 United States copyright laws and international treaty provisions.
 Therefore, you may use this Software only as provided in the license
 agreement accompanying the software package from which you
 obtained this Software ("EULA").
 If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 non-transferable license to copy, modify, and compile the Software
 source code solely for use in connection with Cypress's
 integrated circuit products.  Any reproduction, modification, translation,
 compilation, or representation of this Software except as specified
 above is prohibited without the express written permission of Cypress.

 Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 reserves the right to make changes to the Software without notice. Cypress
 does not assume any liability arising out of the application or use of the
 Software or any product or circuit described in the Software. Cypress does
 not authorize its products for use in any products where a malfunction or
 failure of the Cypress product may reasonably be expected to result in
 significant property damage, injury or death ("High Risk Product"). By
 including Cypress's product in a High Risk Product, the manufacturer
 of such system or application assumes all risk of such use and in doing
 so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/*******************************************************************************
 *        Header Files
 *******************************************************************************/
#include "cyhal.h"
#include "sensor_task.h"
#include "cy_retarget_io.h"
#include "timers.h"
#include "xensiv_pasco2_mtb.h"

#include "app_bt_gatt_handler.h"

/*******************************************************************************
 *        Macro Definitions
 *******************************************************************************/
#define POLL_TIMER_IN_MSEC              (1000u)
#define POLL_TIMER_FREQ                 (10000)

/* Check if notification is enabled for a valid connection ID */
#define IS_NOTIFIABLE(conn_id, cccd)    (((conn_id)!= 0)? (cccd) & GATT_CLIENT_CONFIG_NOTIFICATION: 0)

#define PIN_XENSIV_PASCO2_I2C_SDA       CYBSP_I2C_SDA
#define PIN_XENSIV_PASCO2_I2C_SCL       CYBSP_I2C_SCL
#define PIN_XENSIV_PASCO2_PWR_EN        CYBSP_A3

/* I2C Clock frequency in Hz */
#define I2C_CLK_FREQ_HZ                 (400000U)

/* Wait time for sensor ready (milliseconds) */
#define WAIT_SENSOR_RDY_MS              (1000)

/* The CO2 concentration value acquired by the sensor depends on the external atmospheric pressure.
   To compensate for this effect, pressure values can be acquired from a pressure sensor such as an
   Infineon XENSIV&trade; DPS3xx. (https://github.com/Infineon/sensor-xensiv-dps3xx) */
#define DEFAULT_PRESSURE_REF_HPA        (0x3F7)     /* Default atmospheric pressure to compensate for (hPa) */


/*******************************************************************************
 *        Variable Definitions
 *******************************************************************************/
static cyhal_i2c_t cyhal_i2c;
static xensiv_pasco2_t xensiv_pasco2;

/* Handle to timer object */
TimerHandle_t timer_handle;

/******************************************************************************
 *                          Function Prototypes
 ******************************************************************************/
void timer_callback(TimerHandle_t xTimer);

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
int32_t sensor_task_init()
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize XENSIV PAS CO2 power enable */
    result = cyhal_gpio_init(PIN_XENSIV_PASCO2_PWR_EN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Create timer */
    timer_handle = xTimerCreate("timer",
                                pdMS_TO_TICKS(POLL_TIMER_IN_MSEC),
                                pdTRUE,
                                NULL,
                                timer_callback);
    if (NULL == timer_handle)
    {
        return -1;
    }

    /* Start Timer */
    xTimerStart(timer_handle, 0);

    /* initialize sensor driver here */

    /* Initialize I2C */
    cyhal_i2c_cfg_t i2c_config_m = { false, 0, I2C_CLK_FREQ_HZ };

    result = cyhal_i2c_init(&cyhal_i2c, PIN_XENSIV_PASCO2_I2C_SDA, PIN_XENSIV_PASCO2_I2C_SCL, NULL);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    result = cyhal_i2c_configure(&cyhal_i2c, &i2c_config_m);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    Cy_SysLib_Delay(WAIT_SENSOR_RDY_MS);

    /* Initialize PAS CO2 sensor with default parameter values */
    result = xensiv_pasco2_mtb_init_i2c(&xensiv_pasco2, &cyhal_i2c);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("PAS CO2 device initialization error");
        CY_ASSERT(0);
    }

    return 0;
}

void sensor_task(void *pvParameters)
{
    uint16_t ppm;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        cy_rslt_t result = xensiv_pasco2_mtb_read(&xensiv_pasco2, DEFAULT_PRESSURE_REF_HPA, &ppm);
        if (result == CY_RSLT_SUCCESS)
        {
            printf("CO2 %d ppm.\r\n", ppm);

            /* Copy PPM value to BLE buffer */
            memcpy(&app_xensiv_sensor_shield_pasco2[0], &ppm, sizeof(ppm));

            if (IS_NOTIFIABLE (app_bt_conn_id, app_xensiv_sensor_shield_pasco2_client_char_config[0]) == 0)
            {
                if(!app_bt_conn_id)
                {
                    printf("This device is not connected to a central device\n");
                }else{
                    printf("This device is connected to a central device but\n"
                            "GATT client notifications are not enabled\n");
                }
            }
            else
            {
                wiced_bt_gatt_status_t gatt_status;

                /*
                * Sending notification, set the pv_app_context to NULL, since the
                * data 'app_xensiv_sensor_shield_pasco2' is not to be freed
                */
                gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                                     HDLC_XENSIV_SENSOR_SHIELD_PASCO2_VALUE,
                                                                     app_xensiv_sensor_shield_pasco2_len,
                                                                     (uint8_t *) app_xensiv_sensor_shield_pasco2,
                                                                     NULL);

                printf("Sent notification status 0x%x\n", gatt_status);
            }
        }
    }
}

void timer_callback(TimerHandle_t xTimer)
{
    (void) xTimer;

    BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(sensor_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
