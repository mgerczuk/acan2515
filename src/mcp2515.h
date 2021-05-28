//
//  MCP2515
//
//  Copyright 2021 Martin Gerczuk
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef MCP2515_H
#define MCP2515_H

#include "driver/spi_master.h"
#include "hal/gpio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   MCP2515 Controller operating modes
 */
typedef enum {
    MCP2515_MODE_NORMAL = 0,               /**< Normal operating mode where CAN controller can send/receive/acknowledge messages */
    MCP2515_MODE_SLEEP = 1,
    MCP2515_MODE_LOOPBACK = 2,               /**< Transmission does not require acknowledgment. Use this mode for self testing */
    MCP2515_MODE_LISTEN_ONLY = 3,          /**< The CAN controller will not influence the bus (No transmissions or acknowledgments) but can receive messages */
} mcp2515_mode_t;

typedef struct {    
    spi_device_interface_config_t devcfg;
    mcp2515_mode_t mode;            /**< Mode of CAN controller */
    gpio_num_t int_io_num;          /**< Interrupt GPIO number */
    uint32_t tx_queue_len;          /**< Number of messages TX queue can hold (set to 0 to disable TX Queue) */
    uint32_t rx_queue_len;          /**< Number of messages RX queue can hold */
} mcp2515_general_config_t;

typedef struct {
    uint8_t cnf1;
    uint8_t cnf2;
    uint8_t cnf3;
} mcp2515_timing_config_t;

#define MCP2515_TIMING_CONFIG_8MHZ_500KBITS() {0x40,0x90,0x02}

typedef struct {
} mcp2515_filter_config_t;

#define MCP2515_FRAME_MAX_DLC              8           /**< Max data bytes allowed in CAN message */

/**
 * @brief   Structure to store a CAN message
 */
typedef struct {
    union {
        struct {
            //The order of these bits must match deprecated message flags for compatibility reasons
            uint32_t extd: 1;           /**< Extended Frame Format (29bit ID) */
            uint32_t rtr: 1;            /**< Message is a Remote Frame */
            // uint32_t ss: 1;             /**< Transmit as a Single Shot Transmission. Unused for received. */
            // uint32_t self: 1;           /**< Transmit as a Self Reception Request. Unused for received. */
            // uint32_t dlc_non_comp: 1;   /**< Message's Data length code is larger than 8. This will break compliance with ISO 11898-1 */
            uint32_t reserved: 30;      /**< Reserved bits */
        };
        //Todo: Deprecate flags
        uint32_t flags;                 /**< Deprecated: Alternate way to set bits using message flags */
    };
    uint32_t identifier;                /**< 11 or 29 bit identifier */
    uint8_t data_length_code;           /**< Data length code */
    uint8_t data[MCP2515_FRAME_MAX_DLC];    /**< Data bytes (not relevant in RTR frame) */
} mcp2515_message_t;

esp_err_t mcp2515_driver_install(spi_host_device_t host_id, const mcp2515_general_config_t *g_config, const mcp2515_timing_config_t *t_config, const mcp2515_filter_config_t *f_config);

esp_err_t mcp2515_driver_uninstall(void);

esp_err_t mcp2515_start();

esp_err_t mcp2515_stop();

/**
 * @brief   Receive a CAN message
 *
 * This function receives a message from the RX queue. The flags field of the
 * message structure will indicate the type of message received. This function
 * will block if there are no messages in the RX queue
 *
 * @param[out]  message         Received message
 * @param[in]   ticks_to_wait   Number of FreeRTOS ticks to block on RX queue
 *
 * @warning The flags field of the received message should be checked to determine
 *          if the received message contains any data bytes.
 *
 * @return
 *      - ESP_OK: Message successfully received from RX queue
 *      - ESP_ERR_TIMEOUT: Timed out waiting for message
 *      - ESP_ERR_INVALID_ARG: Arguments are invalid
 *      - ESP_ERR_INVALID_STATE: TWAI driver is not installed
 */
esp_err_t mcp2515_receive(mcp2515_message_t *message, TickType_t ticks_to_wait);

#ifdef __cplusplus
}
#endif

#endif // MCP2515_H
