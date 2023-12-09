/****************************************************************************
 * **File:** csp/interfaces/csp_if_basic.h
 *
 * **Description:** basic interface (serial).
 ****************************************************************************/
#pragma once

#include <csp/csp_interface.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Default name of basic interface.
 */
#define CSP_IF_BASIC_DEFAULT_NAME "BASIC"

typedef void(*csp_basic_tx_func)();

typedef enum {
	BASIC_MODE_NOT_STARTED,  /**< No start detected */
	BASIC_MODE_STARTED,      /**< Started on a basic frame */
	BASIC_MODE_ESCAPED,      /**< Rx escape character */
	BASIC_MODE_SKIP_FRAME,   /**< Skip remaining frame, wait for end character */
} csp_BASIC_Mode_t;

typedef struct {
	csp_basic_tx_func tx_func; /**< Tx function */
	csp_BASIC_Mode_t rx_mode; /**< Rx mode/state. */
	unsigned int rx_length; /**< Rx length */
	bool rx_first; /**< Rx first - if set, waiting for first character
						(== TNC_DATA) after start */
	csp_packet_t * rx_packet; /**< CSP packet for storing Rx data. */
} csp_basic_interface_data_t;

int csp_basic_tx(csp_iface_t * iface, uint16_t via, csp_packet_t * packet, int from_me);
void csp_basic_rx(csp_iface_t * iface, const uint8_t * buf, size_t len, void * pxTaskWoken);
int csp_basic_add_interface(const char * ifname, csp_basic_tx_func tx_func, csp_iface_t **return_iface);

#ifdef __cplusplus
}
#endif
