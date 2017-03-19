/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 */

/** @file
 *
 * @defgroup ble_sdk_apple_notification_main main.c
 * @{
 * @ingroup ble_sdk_app_apple_notification
 * @brief Apple Notification Client Sample Application main file. Disclaimer: 
 * This client implementation of the Apple Notification Center Service can and 
 * will be changed at any time by Nordic Semiconductor ASA.
 *
 * Server implementations such as the ones found in iOS can be changed at any 
 * time by Apple and may cause this client implementation to stop working.
 *
 * This file contains the source code for a sample application using the Apple 
 * Notification Center Service Client.
 */

#include "nrf_ble_ancs_c.h"
#include "ble_db_discovery.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble_hci.h"
#include "ble_gap.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "peer_manager.h"
#include "app_timer.h"
#include "nrf_soc.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "softdevice_handler.h"
#include "fds.h"
#include "fstorage.h"
#include "nrf_delay.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "ble_conn_state.h"

#define NRF_LOG_MODULE_NAME "MAIN"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "app.h"

#include "Arduino.h"


#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE           GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define CENTRAL_LINK_COUNT             0                                           /**< The number of central links used by the application. When changing this number remember to adjust the RAM settings. */
#define PERIPHERAL_LINK_COUNT          0                                           /**< The number of peripheral links used by the application. When changing this number remember to adjust the RAM settings. */
#define VENDOR_SPECIFIC_UUID_COUNT     10                                          /**< The number of vendor specific UUIDs used by this example. */

#define ATTR_DATA_SIZE                 BLE_ANCS_ATTR_DATA_MAX                      /**< Allocated size for attribute data. */

#define DEVICE_NAME                    "ANCS"                                      /**< Name of the device. Will be included in the advertising data. */
#define APP_ADV_FAST_INTERVAL          40                                          /**< The advertising interval (in units of 0.625 ms). The default value corresponds to 25 ms. */
#define APP_ADV_SLOW_INTERVAL          3200                                        /**< Slow advertising interval (in units of 0.625 ms). The default value corresponds to 2 seconds. */
#define APP_ADV_FAST_TIMEOUT           180                                         /**< The advertising time-out in units of seconds. */
#define APP_ADV_SLOW_TIMEOUT           180                                         /**< The advertising time-out in units of seconds. */
#define ADV_INTERVAL_FAST_PERIOD       30                                          /**< The duration of the fast advertising period (in seconds). */

#define APP_TIMER_OP_QUEUE_SIZE        5                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL              MSEC_TO_UNITS(500, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL              MSEC_TO_UNITS(1000, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                  0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT               MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating an event (connect or start of notification) to the first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT   3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define MESSAGE_BUFFER_SIZE            18                                          /**< Size of buffer holding optional messages in notifications. */

#define SEC_PARAM_BOND                 1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                 0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                 0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS             0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES      BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                  0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE         7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE         16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                      0xDEADBEEF                                  /**< Value used as error code on stack dump. Can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE      MAX(APP_TIMER_SCHED_EVT_SIZE, \
                                           BLE_STACK_HANDLER_SCHED_EVT_SIZE)       /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE               20                                          /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE               10                                          /**< Maximum number of events in the scheduler queue. */
#endif

APP_TIMER_DEF(m_app_timer);

static volatile bool app_ready;


void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t *p_file_name) {

  if (error_code == NRF_SUCCESS) return;

		NRF_LOG_ERROR("Erreur: 0x%x ligne %u file %s !!\n", (unsigned int)error_code, (unsigned int)line_num, (uint32_t) p_file_name);

}


void app_error_handler_bare(uint32_t error_code) {

  if (error_code == NRF_SUCCESS) return;

  NRF_LOG_ERROR("Erreur bare: 0x%x\n", error_code);


}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info) {

}


/**@brief Callback function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. 
 *          You must analyze how your product should react to asserts.
 * @warning On assert from the SoftDevice, the system can recover only on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}




/**@brief Function for initializing the timer module.
 */
static void timers_init(void) {
	// Initialize timer module, making it use the scheduler.
	APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);

}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
	ret_code_t ret;

	nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
	if (CLOCK_CONFIG_LF_SRC == NRF_CLOCK_LF_SRC_XTAL) {
		// raytak + 32kHz crystal
		clock_lf_cfg.source = NRF_CLOCK_LF_SRC_XTAL;
		clock_lf_cfg.rc_ctiv = 0;
		clock_lf_cfg.rc_temp_ctiv = 0;
		clock_lf_cfg.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM;
	}

	// Initialize the SoftDevice handler module.
	SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

	ble_enable_params_t ble_enable_params;
	ret = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
	PERIPHERAL_LINK_COUNT, &ble_enable_params);
	APP_ERROR_CHECK(ret);

	ble_enable_params.common_enable_params.vs_uuid_count = VENDOR_SPECIFIC_UUID_COUNT;

	// Check the ram settings against the used number of links
	CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

	// Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
	ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
	ret = softdevice_enable(&ble_enable_params);
	APP_ERROR_CHECK(ret);

}


/**@brief Function for initializing the Event Scheduler.
 */
static void scheduler_init(void) {
	APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void) {
	ret_code_t ret = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(ret);
}

/**@brief Function for the Power manager.
 */
static void power_manage(void) {
	ret_code_t ret = sd_app_evt_wait();
	APP_ERROR_CHECK(ret);
}

/**@brief Function for application main entry.
 */
int main(void) {

	APP app;
	bool erase_bonds = true;

	// Initialize.

	log_init();
	ble_stack_init();

	// app timer init
	timers_init();
	//app.init();

	if (erase_bonds == true) {
		NRF_LOG_INFO("Bonds erased!\r\n");
	}

	scheduler_init();

	// Start execution.
	NRF_LOG_INFO("BLE ANCS Started\r\n");


	// Enter main loop.
	for (;;) {

		// run user application
		app.sm_run();

		app_sched_execute();
		//  && !app.bigTick NRF_LOG_PROCESS() == false &&
		if (NRF_LOG_PROCESS() == false && !app.bigTick) {
			power_manage();

		}

//		if (millis() > 10000) {
//			sd_power_system_off();
//		}
	}

}

/**
 * @}
 */

