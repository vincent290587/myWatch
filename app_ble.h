/*
 * app_ble.h
 *
 *  Created on: 19 avr. 2017
 *      Author: Vincent
 */

#ifndef APP_BLE_H_
#define APP_BLE_H_

#include "ble_db_discovery.h"

#define VENDOR_SPECIFIC_UUID_COUNT     10                                          /**< The number of vendor specific UUIDs used by this example. */

void bsp_event_handler(bsp_event_t event);

void cts_timers_init(void);
void cts_timers_start(void);

void ancs_services_init(void);
void cts_services_init(void);

void advUUIDinitCTS(ble_uuid_t* m_adv_uuids);
void advUUIDinitANCS(ble_uuid_t* m_adv_uuids);

void cts_db_disc_handler(ble_db_discovery_evt_t * p_evt);
void ancs_db_disc_handler(ble_db_discovery_evt_t * p_evt);

void cts_ble_evt_dispatch(ble_evt_t * p_ble_evt);
void ancs_ble_evt_dispatch(ble_evt_t * p_ble_evt);

void connHandleRemoveCTS(ble_evt_t * p_ble_evt);
void connHandleRemoveANCS(ble_evt_t * p_ble_evt);

// DFU related
void dfu_init();
void ble_evt_dfu(ble_evt_t * p_ble_evt);

#endif /* APP_BLE_H_ */
