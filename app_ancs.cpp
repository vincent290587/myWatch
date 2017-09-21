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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

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

#include "Global.h"
#include "TimeKeeper.h"

using namespace mvc;

#define NRF_LOG_MODULE_NAME "APP_ANCS"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"


#define ATTR_DATA_SIZE                 BLE_ANCS_ATTR_DATA_MAX                      /**< Allocated size for attribute data. */

#define DISPLAY_MESSAGE_BUTTON_ID      1                                           /**< Button used to request notification attributes. */

#define SECURITY_REQUEST_DELAY         APP_TIMER_TICKS(1500, APP_TIMER_PRESCALER)  /**< Delay after connection until security request is sent, if necessary (ticks). */


#define SCHED_MAX_EVENT_DATA_SIZE      MAX(APP_TIMER_SCHED_EVT_SIZE, \
		BLE_STACK_HANDLER_SCHED_EVT_SIZE)       /**< Maximum size of scheduler events. */


/**@brief String literals for the iOS notification categories. used then printing to UART. */
static const char * lit_catid[BLE_ANCS_NB_OF_CATEGORY_ID] = { "Other",
		"Incoming Call", "Missed Call", "Voice Mail", "Social", "Schedule",
		"Email", "News", "Health And Fitness", "Business And Finance",
		"Location", "Entertainment" };

/**@brief String literals for the iOS notification event types. Used then printing to UART. */
static const char * lit_eventid[BLE_ANCS_NB_OF_EVT_ID] = { "Added", "Modified",
		"Removed" };

/**@brief String literals for the iOS notification attribute types. Used when printing to UART. */
//static const char * lit_attrid[BLE_ANCS_NB_OF_NOTIF_ATTR] = { "App Identifier",
//		"Title", "Subtitle", "Message", "Message Size", "Date",
//		"Positive Action Label", "Negative Action Label" };


/**@brief String literals for the iOS notification attribute types. Used When printing to UART. */
//static const char * lit_appid[BLE_ANCS_NB_OF_APP_ATTR] = { "Display Name" };

static ble_ancs_c_t m_ancs_c; /**< Structure used to identify the Apple Notification Service Client. */

static ble_ancs_c_evt_notif_t m_notification_latest; /**< Local copy to keep track of the newest arriving notifications. */
static ble_ancs_c_attr_t m_notif_attr_latest; /**< Local copy of the newest notification attribute. */
static ble_ancs_c_attr_t m_notif_attr_app_id_latest; /**< Local copy of the newest app attribute. */

// TODO replace by String
static uint8_t m_attr_appid[ATTR_DATA_SIZE]; /**< Buffer to store attribute data. */
static uint8_t m_attr_title[ATTR_DATA_SIZE]; /**< Buffer to store attribute data. */
static uint8_t m_attr_subtitle[ATTR_DATA_SIZE]; /**< Buffer to store attribute data. */
static uint8_t m_attr_message[ATTR_DATA_SIZE]; /**< Buffer to store attribute data. */
static uint8_t m_attr_message_size[ATTR_DATA_SIZE]; /**< Buffer to store attribute data. */
//static uint8_t m_attr_date[ATTR_DATA_SIZE]; /**< Buffer to store attribute data. */
//static uint8_t m_attr_posaction[ATTR_DATA_SIZE]; /**< Buffer to store attribute data. */
//static uint8_t m_attr_negaction[ATTR_DATA_SIZE]; /**< Buffer to store attribute data. */
//static uint8_t m_attr_disp_name[ATTR_DATA_SIZE]; /**< Buffer to store attribute data. */

SNotif last_notif;


void advUUIDinitANCS(ble_uuid_t* m_adv_uuids) {
	m_adv_uuids[0].uuid = ANCS_UUID_SERVICE;
	m_adv_uuids[0].type = m_ancs_c.service.service.uuid.type;
}

void connHandleRemoveANCS(ble_evt_t * p_ble_evt) {
	if (p_ble_evt->evt.gap_evt.conn_handle == m_ancs_c.conn_handle)
	{
		m_ancs_c.conn_handle = BLE_CONN_HANDLE_INVALID;
	}
}


/**@brief Function for setting up GATTC notifications from the Notification Provider.
 *
 * @details This function is called when a successful connection has been established.
 */
static void apple_notification_setup(void) {
	ret_code_t ret;

	nrf_delay_ms(100); // Delay because we cannot add a CCCD too close to starting encryption. iOS specific.

	ret = ble_ancs_c_notif_source_notif_enable(&m_ancs_c);
	APP_ERROR_CHECK(ret);

	ret = ble_ancs_c_data_source_notif_enable(&m_ancs_c);
	APP_ERROR_CHECK(ret);

	NRF_LOG_INFO("Notifications Enabled.\r\n");
}

/**@brief Function for printing an iOS notification.
 *
 * @param[in] p_notif  Pointer to the iOS notification.
 */
static void notif_print(ble_ancs_c_evt_notif_t * p_notif) {

	static uint32_t _millis = 0;
	uint32_t ret;

	if (millis() - _millis > 5000) {

		// print the notif
		NRF_LOG_WARNING("Notification\r\n");
		NRF_LOG_INFO("Event:       %s\r\n", (uint32_t )lit_eventid[p_notif->evt_id]);
		NRF_LOG_INFO("Category ID: %s\r\n", (uint32_t )lit_catid[p_notif->category_id]);
		NRF_LOG_INFO("Category Cnt:%u\r\n", (unsigned int ) p_notif->category_count);
		NRF_LOG_INFO("UID:         %u\r\n", (unsigned int ) p_notif->notif_uid);
		NRF_LOG_FLUSH();

		if (p_notif->evt_id == BLE_ANCS_EVENT_ID_NOTIFICATION_ADDED) {

			last_notif.type = p_notif->category_id;

			switch (p_notif->category_id) {
			case BLE_ANCS_CATEGORY_ID_INCOMING_CALL:
				_millis = millis();
				ret = nrf_ble_ancs_c_request_attrs(&m_ancs_c, p_notif);
				APP_ERROR_CHECK(ret);
				neopix.setNotify(WS_RED, 50);
				break;
			case BLE_ANCS_CATEGORY_ID_SOCIAL:
				_millis = millis();
				ret = nrf_ble_ancs_c_request_attrs(&m_ancs_c, p_notif);
				APP_ERROR_CHECK(ret);
				neopix.setNotify(WS_GREEN);
				break;
			default:
				break;
			}

		}
	}
}

/**@brief Function for printing iOS notification attribute data.
 *
 * @param[in] p_attr Pointer to an iOS notification attribute.
 */
static void notif_attr_print(ble_ancs_c_attr_t * p_attr) {

#ifdef DEBUG
	if (p_attr->attr_len != 0) {
		NRF_LOG_INFO("%s: %s\r\n", (uint32_t )lit_attrid[p_attr->attr_id], nrf_log_push((char * )p_attr->p_attr_data));
	} else if (p_attr->attr_len == 0) {
		NRF_LOG_INFO("%s: (N/A)\r\n", (uint32_t )lit_attrid[p_attr->attr_id]);
	}
	NRF_LOG_FLUSH();
#endif

	if (p_attr->attr_id == BLE_ANCS_NOTIF_ATTR_ID_TITLE) {
		last_notif.title = String((char*)p_attr->p_attr_data);
	} else if (p_attr->attr_id == BLE_ANCS_NOTIF_ATTR_ID_MESSAGE) {
		last_notif.msg = String((char*)p_attr->p_attr_data);

		vue.addNotification(&last_notif);
	}
}

/**@brief Function for printing iOS notification attribute data.
 *
 * @param[in] p_attr Pointer to an iOS App attribute.
 */
static void app_attr_print(ble_ancs_c_attr_t * p_attr) {
//	if (p_attr->attr_len != 0) {
//		NRF_LOG_INFO("%s: %s\r\n", (uint32_t )lit_appid[p_attr->attr_id],
//				(uint32_t )p_attr->p_attr_data);
//	} else if (p_attr->attr_len == 0) {
//		NRF_LOG_INFO("%s: (N/A)\r\n", (uint32_t ) lit_appid[p_attr->attr_id]);
//	}
}

/**@brief Function for printing out errors that originated from the Notification Provider (iOS).
 *
 * @param[in] err_code_np Error code received from NP.
 */
static void err_code_print(uint16_t err_code_np) {
#ifdef DEBUG
	switch (err_code_np) {
	case BLE_ANCS_NP_UNKNOWN_COMMAND:
		NRF_LOG_INFO("Error: Command ID was not recognized by the Notification Provider. \r\n");
		break;

	case BLE_ANCS_NP_INVALID_COMMAND:
		NRF_LOG_INFO("Error: Command failed to be parsed on the Notification Provider. \r\n");
		break;

	case BLE_ANCS_NP_INVALID_PARAMETER:
		NRF_LOG_INFO("Error: Parameter does not refer to an existing object on the Notification Provider. \r\n");
		break;

	case BLE_ANCS_NP_ACTION_FAILED:
		NRF_LOG_INFO("Error: Perform Notification Action Failed on the Notification Provider. \r\n");
		break;

	default:
		break;
	}
#endif
}


/**@brief Function for handling the Apple Notification Service client.
 *
 * @details This function is called for all events in the Apple Notification client that
 *          are passed to the application.
 *
 * @param[in] p_evt  Event received from the Apple Notification Service client.
 */
static void on_ancs_c_evt(ble_ancs_c_evt_t * p_evt) {
	ret_code_t ret = NRF_SUCCESS;

	switch (p_evt->evt_type) {
	case BLE_ANCS_C_EVT_DISCOVERY_COMPLETE:
		NRF_LOG_INFO("Apple Notification Center Service discovered on the server.\r\n");
		ret = nrf_ble_ancs_c_handles_assign(&m_ancs_c, p_evt->conn_handle, &p_evt->service);
		APP_ERROR_CHECK(ret);
		apple_notification_setup();
		break;

	case BLE_ANCS_C_EVT_NOTIF:
		m_notification_latest = p_evt->notif;
		notif_print(&m_notification_latest);
		break;

	case BLE_ANCS_C_EVT_NOTIF_ATTRIBUTE:
		m_notif_attr_latest = p_evt->attr;
		notif_attr_print(&m_notif_attr_latest);
		if (p_evt->attr.attr_id == BLE_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER) {
			m_notif_attr_app_id_latest = p_evt->attr;
		}
		break;
	case BLE_ANCS_C_EVT_DISCOVERY_FAILED:
		NRF_LOG_ERROR("Apple Notification Center Service not discovered on the server.\r\n");
		break;

	case BLE_ANCS_C_EVT_APP_ATTRIBUTE:
		app_attr_print(&p_evt->attr);
		break;
	case BLE_ANCS_C_EVT_NP_ERROR:
		err_code_print(p_evt->err_code_np);
		break;
	default:
		// No implementation needed.
		break;
	}
}


/**@brief Function for handling the Apple Notification Service client errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void apple_notification_error_handler(uint32_t nrf_error) {
	APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling Database Discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective service instances.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
void ancs_db_disc_handler(ble_db_discovery_evt_t * p_evt) {
	ble_ancs_c_on_db_disc_evt(&m_ancs_c, p_evt);
}


#ifdef DEBUG
/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event) {
	ret_code_t ret;

	switch (event) {
	case BSP_EVENT_SLEEP:
		break;

	case BSP_EVENT_DISCONNECT:
		break;

	case BSP_EVENT_WHITELIST_OFF:
		break;

	case BSP_EVENT_KEY_0:
		NRF_LOG_INFO("BTN 0");
		ret = nrf_ble_ancs_c_request_attrs(&m_ancs_c, &m_notification_latest);
		APP_ERROR_CHECK(ret);
		break;

	case BSP_EVENT_KEY_1:
		NRF_LOG_INFO("BTN 1");
//		if (m_notif_attr_app_id_latest.attr_id
//				== BLE_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER
//				&& m_notif_attr_app_id_latest.attr_len != 0) {
//			NRF_LOG_INFO("Request for %s: \r\n",
//					(uint32_t )m_notif_attr_app_id_latest.p_attr_data);
//			ret = nrf_ble_ancs_c_app_attr_request(&m_ancs_c,
//					m_notif_attr_app_id_latest.p_attr_data,
//					m_notif_attr_app_id_latest.attr_len);
//			APP_ERROR_CHECK(ret);
//		}
		break;

	case BSP_EVENT_KEY_2:
		NRF_LOG_INFO("BTN 2");
//		if (m_notification_latest.evt_flags.positive_action == true) {
//			NRF_LOG_INFO("Performing Positive Action.\r\n");
//			ret = nrf_ancs_perform_notif_action(&m_ancs_c,
//					m_notification_latest.notif_uid, ACTION_ID_POSITIVE);
//			APP_ERROR_CHECK(ret);
//		}
		break;

	case BSP_EVENT_KEY_3:
		NRF_LOG_INFO("BTN 3");
//		if (m_notification_latest.evt_flags.negative_action == true) {
//			NRF_LOG_INFO("Performing Negative Action.\r\n");
//			ret = nrf_ancs_perform_notif_action(&m_ancs_c,
//					m_notification_latest.notif_uid, ACTION_ID_NEGATIVE);
//			APP_ERROR_CHECK(ret);
//		}
		break;

	default:
		break;
	}
}
#endif

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
void ancs_ble_evt_dispatch(ble_evt_t * p_ble_evt) {
	/** The Connection state module has to be fed BLE events in order to function correctly
	 * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
	ble_ancs_c_on_ble_evt(&m_ancs_c, p_ble_evt);
}

/**@brief Function for initializing the Apple Notification Center Service.
 */
void ancs_services_init(void) {
	ble_ancs_c_init_t ancs_init_obj;
	ret_code_t ret;

	memset(&ancs_init_obj, 0, sizeof(ancs_init_obj));

	ret = nrf_ble_ancs_c_attr_add(&m_ancs_c,
			BLE_ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER, m_attr_appid, ATTR_DATA_SIZE);
	APP_ERROR_CHECK(ret);

//	ret = nrf_ble_ancs_c_app_attr_add(&m_ancs_c,
//			BLE_ANCS_APP_ATTR_ID_DISPLAY_NAME, m_attr_disp_name, sizeof(m_attr_disp_name));
//	APP_ERROR_CHECK(ret);

	ret = nrf_ble_ancs_c_attr_add(&m_ancs_c, BLE_ANCS_NOTIF_ATTR_ID_TITLE,
			m_attr_title, ATTR_DATA_SIZE);
	APP_ERROR_CHECK(ret);

	ret = nrf_ble_ancs_c_attr_add(&m_ancs_c, BLE_ANCS_NOTIF_ATTR_ID_MESSAGE,
			m_attr_message, ATTR_DATA_SIZE);
	APP_ERROR_CHECK(ret);

	ret = nrf_ble_ancs_c_attr_add(&m_ancs_c, BLE_ANCS_NOTIF_ATTR_ID_SUBTITLE,
			m_attr_subtitle, ATTR_DATA_SIZE);
	APP_ERROR_CHECK(ret);

	ret = nrf_ble_ancs_c_attr_add(&m_ancs_c, BLE_ANCS_NOTIF_ATTR_ID_MESSAGE_SIZE, m_attr_message_size,
			ATTR_DATA_SIZE);
	APP_ERROR_CHECK(ret);

//	ret = nrf_ble_ancs_c_attr_add(&m_ancs_c, BLE_ANCS_NOTIF_ATTR_ID_DATE,
//			m_attr_date, ATTR_DATA_SIZE);
//	APP_ERROR_CHECK(ret);
//
//	ret = nrf_ble_ancs_c_attr_add(&m_ancs_c, BLE_ANCS_NOTIF_ATTR_ID_POSITIVE_ACTION_LABEL, m_attr_posaction,
//			ATTR_DATA_SIZE);
//	APP_ERROR_CHECK(ret);
//
//	ret = nrf_ble_ancs_c_attr_add(&m_ancs_c, BLE_ANCS_NOTIF_ATTR_ID_NEGATIVE_ACTION_LABEL, m_attr_negaction,
//			ATTR_DATA_SIZE);
//	APP_ERROR_CHECK(ret);

	ancs_init_obj.evt_handler = on_ancs_c_evt;
	ancs_init_obj.error_handler = apple_notification_error_handler;

	ret = ble_ancs_c_init(&m_ancs_c, &ancs_init_obj);
	APP_ERROR_CHECK(ret);
}


/**
 * @}
 */

