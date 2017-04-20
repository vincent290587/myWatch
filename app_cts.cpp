/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_cts_c_main main.c
 * @{
 * @ingroup ble_sdk_app_cts_c
 * @brief Current Time Profile sample application.
 *
 * This file contains the source code for a sample application that uses Current Time Service.
 * This is the client role of the profile, implemented on a peripheral device.
 * When a central device connects, the application will trigger a security procedure (if this is not done
 * by the central side first). Completion of the security procedure will trigger a service
 * discovery. When the Current Time Service and Characteristic have been discovered on the
 * server, pressing button 1 will trigger a read of the current time and print it on the UART.
 *
 */

#include <stdint.h>
#include <string.h>
#include "sdk_config.h"
#include "app_error.h"
#include "app_util.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "boards.h"
#include "ble.h"
#include "ble_cts_c.h"
#include "ble_conn_state.h"
#include "ble_db_discovery.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "fstorage.h"
#include "peer_manager.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_gpio.h"

#include "Arduino.h"
#include "Global.h"

using namespace mvc;

#define NRF_LOG_MODULE_NAME "APP_CTS"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define TIME_UPDATE_DELAY               APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER)  /**< Delay after connection until security request is sent, if necessary (ticks). */

static ble_cts_c_t        m_cts;                                                    /**< Instance of Current Time Service. The instance uses this struct to store data related to the service. */

APP_TIMER_DEF(m_time_timer_id);                          /**< time request timer. */




void advUUIDinitCTS(ble_uuid_t* m_adv_uuids) {
	m_adv_uuids[0].uuid = BLE_UUID_CURRENT_TIME_SERVICE;
	m_adv_uuids[0].type = BLE_UUID_TYPE_BLE;
}

void connHandleRemoveCTS(ble_evt_t * p_ble_evt) {
	if (p_ble_evt->evt.gap_evt.conn_handle == m_cts.conn_handle)
	{
		m_cts.conn_handle = BLE_CONN_HANDLE_INVALID;
	}
}

/**@brief Function for handling the Current Time Service errors.
 *
 * @param[in]  nrf_error  Error code containing information about what went wrong.
 */
static void current_time_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}



static void update_time() {

	if (m_cts.conn_handle != BLE_CONN_HANDLE_INVALID)
	{
		uint32_t err_code = ble_cts_c_current_time_read(&m_cts);
		if (err_code == NRF_ERROR_NOT_FOUND)
		{
			NRF_LOG_ERROR("Current Time Service is not discovered.\r\n");
		} else {
			NRF_LOG_INFO("Timeupdate request sent\r\n");

		}
	}


}


static void time_update_handler(void * p_context)
{
	update_time();
}


/**@brief Function for handling the Current Time Service errors.
 *
 * @param[in] p_evt  Event received from the Current Time Service client.
 */
static void current_time_print(ble_cts_c_evt_t * p_evt)
{
    NRF_LOG_INFO("\r\nCurrent Time:\r\n");

    NRF_LOG_FLUSH();
    NRF_LOG_INFO("\tHours     %i\r\n",
                   p_evt->params.current_time.exact_time_256.day_date_time.date_time.hours);
    NRF_LOG_FLUSH();
    NRF_LOG_INFO("\tMinutes   %i\r\n",
                   p_evt->params.current_time.exact_time_256.day_date_time.date_time.minutes);
    NRF_LOG_FLUSH();
    NRF_LOG_INFO("\tSeconds   %i\r\n",
                   p_evt->params.current_time.exact_time_256.day_date_time.date_time.seconds);
    NRF_LOG_FLUSH();

    NRF_LOG_ERROR("Time updated :-)\r\n");

    app.timekeeper.setTime(p_evt->params.current_time.exact_time_256.day_date_time.date_time.seconds,
    		p_evt->params.current_time.exact_time_256.day_date_time.date_time.minutes,
			p_evt->params.current_time.exact_time_256.day_date_time.date_time.hours,
			p_evt->params.current_time.exact_time_256.day_date_time.date_time.day,
			p_evt->params.current_time.exact_time_256.day_date_time.date_time.month,
			p_evt->params.current_time.exact_time_256.day_date_time.date_time.year);
}


/**@brief Function for the timer initialization.
 *
 * @details Initializes the timer module.
 */
void cts_timers_init(void)
{
    uint32_t err_code;

    err_code = app_timer_create(&m_time_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
								time_update_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the timer initialization.
 *
 * @details Initializes the timer module.
 */
void cts_timers_start(void)
{
    uint32_t err_code;

    err_code = app_timer_start(m_time_timer_id, TIME_UPDATE_DELAY, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Current Time Service client events.
 *
 * @details This function will be called for all events in the Current Time Service client that
 *          are passed to the application.
 *
 * @param[in] p_evt Event received from the Current Time Service client.
 */
static void on_cts_c_evt(ble_cts_c_t * p_cts, ble_cts_c_evt_t * p_evt)
{
    uint32_t err_code;

    switch (p_evt->evt_type)
    {
        case BLE_CTS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Current Time Service discovered on server.\r\n");
            err_code = ble_cts_c_handles_assign(&m_cts,
                                                p_evt->conn_handle,
                                                &p_evt->params.char_handles);
            APP_ERROR_CHECK(err_code);

            err_code = app_timer_start(m_time_timer_id, TIME_UPDATE_DELAY, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_CTS_C_EVT_DISCOVERY_FAILED:
            NRF_LOG_ERROR("Current Time Service not found on server. \r\n");
            break;

        case BLE_CTS_C_EVT_DISCONN_COMPLETE:
            NRF_LOG_INFO("Disconnect Complete.\r\n");
            break;

        case BLE_CTS_C_EVT_CURRENT_TIME:
            NRF_LOG_INFO("Current Time received.\r\n");
            current_time_print(p_evt);
            break;

        case BLE_CTS_C_EVT_INVALID_TIME:
            NRF_LOG_INFO("Invalid Time received.\r\n");
            break;

        default:
            break;
    }
}



/**@brief Function for initializing services that will be used by the application.
 */
void cts_services_init(void)
{
    uint32_t         err_code;
    ble_cts_c_init_t cts_init_obj;

    cts_init_obj.evt_handler   = on_cts_c_evt;
    cts_init_obj.error_handler = current_time_error_handler;
    err_code                   = ble_cts_c_init(&m_cts, &cts_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Database Discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective service instances.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
void cts_db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_cts_c_on_db_disc_evt(&m_cts, p_evt);
}



/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
void cts_ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    /** The Connection state module has to be fed BLE events in order to function correctly
     * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
    ble_cts_c_on_ble_evt(&m_cts, p_ble_evt);
}


/**
 * @}
 */
