/*
 * SPI.cpp
 *
 *  Created on: 25 févr. 2017
 *      Author: Vincent
 */

#include "nrf_drv_spi.h"

#define NRF_LOG_MODULE_NAME "SPI"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "SPI.h"


#define SPI_BUFFER_L 25

#define SPI_INSTANCE  0 /**< SPI instance index. */


static uint8_t m_tx_buf[SPI_BUFFER_L]; /**< TX buffer. */
static uint8_t m_rx_buf[SPI_BUFFER_L]; /**< RX buffer. */

static uint32_t m_length;

static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

static volatile bool spi_xfer_done;

/**
 * @brief SPI user event handler.
 * @param event
 */
static void spi_event_handler(nrf_drv_spi_evt_t const * p_event) {

	spi_xfer_done = true;
	//NRF_LOG_INFO("Transfer completed.\r\n");
	if (m_rx_buf[0] != 0) {
		NRF_LOG_INFO(" Received: \r\n");
		NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *) m_rx_buf));
	}
}

// init the SPI transfer
void spi_init(void) {

	m_length = 0;

	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	spi_config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED;
	spi_config.miso_pin = NRF_DRV_SPI_PIN_NOT_USED;
	spi_config.mosi_pin = 5;
	spi_config.sck_pin = 6;
	spi_config.frequency = NRF_DRV_SPI_FREQ_4M;
	spi_config.mode = NRF_DRV_SPI_MODE_0;

	APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));

}

// add a byte in the buffer
void spi_add_byte(uint8_t data_) {

	if (m_length < SPI_BUFFER_L - 1) {
		m_tx_buf[m_length++] = data_;
	} else {
		NRF_LOG_INFO(" Buffer full\r\n");
		spi_transfer();
	}

}

// add a byte in the buffer
void spi_reset_buffer() {

	NRF_LOG_INFO(" Buffer reset: %u\r\n", m_length);
	memset(m_rx_buf, 0, SPI_BUFFER_L);
	m_length = 0;

}

// transmit the complete buffer
void spi_transfer() {

	// Reset rx buffer and transfer done flag
	memset(m_rx_buf, 0, SPI_BUFFER_L);

	spi_xfer_done = false;

	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));

	// SPI in blocking mode
	while (!spi_xfer_done) {
		__WFE();
	}

	// reset counter
	m_length = 0;
}
