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

#include "mcp2515.h"
#include <string.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#define TAG "MCP2515"

#define MCP2515_CHECK(cond, ret_val) ({                                     \
            if (!(cond)) {                                                  \
                return (ret_val);                                           \
            }                                                               \
})
#define MCP2515_CHECK_FROM_CRIT(cond, ret_val) ({                           \
            if (!(cond)) {                                                  \
                MCP2515_EXIT_CRITICAL();                                    \
                return ret_val;                                             \
            }                                                               \
})
#define MCP2515_ERR_CHECK(esp_err) ({                                       \
            esp_err_t err = esp_err;                                        \
            if (err != ESP_OK) {                                            \
                return (err);                                               \
            }                                                               \
})

// --------------------------------------------------------------------------

static const uint8_t RESET_COMMAND = 0xC0;
static const uint8_t WRITE_COMMAND = 0x02 ;
static const uint8_t READ_COMMAND  = 0x03 ;
static const uint8_t BIT_MODIFY_COMMAND         = 0x05 ;
static const uint8_t READ_FROM_RXB0SIDH_COMMAND = 0x90 ;
static const uint8_t READ_FROM_RXB1SIDH_COMMAND = 0x94 ;
static const uint8_t RX_STATUS_COMMAND          = 0xB0 ;

// --------------------------------------------------------------------------

const uint8_t BFPCTRL_REGISTER   = 0x0C ;
const uint8_t TXRTSCTRL_REGISTER = 0x0D ;
const uint8_t CANSTAT_REGISTER   = 0x0E ;
const uint8_t CANCTRL_REGISTER   = 0x0F ;
const uint8_t TEC_REGISTER       = 0x1C ;
const uint8_t REC_REGISTER       = 0x1D ;
const uint8_t RXM0SIDH_REGISTER  = 0x20 ;
const uint8_t RXM1SIDH_REGISTER  = 0x24 ;
const uint8_t CNF3_REGISTER      = 0x28 ;
const uint8_t CNF2_REGISTER      = 0x29 ;
const uint8_t CNF1_REGISTER      = 0x2A ;
const uint8_t CANINTF_REGISTER   = 0x2C ;
const uint8_t EFLG_REGISTER      = 0x2D ;
const uint8_t TXB0CTRL_REGISTER  = 0x30 ;
const uint8_t TXB1CTRL_REGISTER  = 0x40 ;
const uint8_t TXB2CTRL_REGISTER  = 0x50 ;
const uint8_t RXB0CTRL_REGISTER  = 0x60 ;
const uint8_t RXB1CTRL_REGISTER  = 0x70 ;
const uint8_t RXFSIDH_REGISTER [6] = {0x00, 0x04, 0x08, 0x10, 0x14, 0x18} ;

// --------------------------------------------------------------------------

typedef struct {
    // spi_host_device_t host_id;
    spi_device_handle_t spi;
    mcp2515_timing_config_t timing;
    mcp2515_mode_t mode;
    TaskHandle_t taskHandle;
    QueueHandle_t tx_queue;
    QueueHandle_t rx_queue;
} mcp2515_obj_t;

static mcp2515_obj_t* p_mcp2515_obj = NULL;

static void mcp2515_ihandler_task();
static void IRAM_ATTR mcp2515_isr(void* p);
static esp_err_t mcp2515_reset();
static esp_err_t mcp2515_write_register(uint8_t reg, uint8_t value);
static esp_err_t mcp2515_write_registers(uint8_t reg, int n, uint8_t* values);
static esp_err_t mcp2515_read_register(uint8_t reg, uint8_t* value);

SemaphoreHandle_t mISRSemaphore;

esp_err_t mcp2515_driver_install(spi_host_device_t host_id, const mcp2515_general_config_t *g_config, const mcp2515_timing_config_t *t_config, const mcp2515_filter_config_t *f_config)
{
    MCP2515_CHECK(g_config != NULL, ESP_ERR_INVALID_ARG);
    MCP2515_CHECK(t_config != NULL, ESP_ERR_INVALID_ARG);
    // MCP2515_CHECK(f_config != NULL, ESP_ERR_INVALID_ARG);

    spi_device_handle_t spi;
    esp_err_t ret = spi_bus_add_device(host_id, &g_config->devcfg, &spi);
    if (ret != ESP_OK) return ret;

    QueueHandle_t tx_queue = (g_config->tx_queue_len > 0) ? xQueueCreate(g_config->tx_queue_len, sizeof(mcp2515_message_t)) : NULL;
    QueueHandle_t rx_queue = (g_config->rx_queue_len > 0) ? xQueueCreate(g_config->rx_queue_len, sizeof(mcp2515_message_t)) : NULL;

    mISRSemaphore = xSemaphoreCreateCounting (10, 0);

    TaskHandle_t taskHandle;
    if (pdPASS != xTaskCreate(mcp2515_ihandler_task, "mcp2515", 2048, NULL, 23, &taskHandle))
        return ESP_ERR_NO_MEM;

    ret = gpio_set_direction(g_config->int_io_num, GPIO_MODE_INPUT);
    if (ret != ESP_OK) return ret;
    ret = gpio_set_intr_type(g_config->int_io_num, GPIO_INTR_NEGEDGE);
    if (ret != ESP_OK) return ret;
    ret = gpio_isr_handler_add(g_config->int_io_num, mcp2515_isr, NULL);
    if (ret != ESP_OK) return ret;

    p_mcp2515_obj = (mcp2515_obj_t*)malloc(sizeof(mcp2515_obj_t));
    // p_mcp2515_obj->host_id = host_id;
    p_mcp2515_obj->spi = spi;
    p_mcp2515_obj->timing = *t_config;
    p_mcp2515_obj->mode = g_config->mode;
    p_mcp2515_obj->taskHandle = taskHandle;
    p_mcp2515_obj->tx_queue = tx_queue;
    p_mcp2515_obj->rx_queue = rx_queue;

    return ESP_OK;
}

esp_err_t mcp2515_driver_uninstall(void)
{
    if (p_mcp2515_obj == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = spi_bus_remove_device(p_mcp2515_obj->spi);

    free(p_mcp2515_obj);
    p_mcp2515_obj = NULL;
    return ret;
}

esp_err_t mcp2515_start()
{
    MCP2515_CHECK(p_mcp2515_obj != NULL, ESP_ERR_INVALID_STATE);

    esp_err_t ret = mcp2515_reset();
    if (ret != ESP_OK) return ret;

    // ---
    
    // ret = mcp2515_write_register(CNF1_REGISTER, 0x55);
    // if (ret != ESP_OK) return ret;

    // uint8_t val1 = 0xAA;
    // ret = mcp2515_read_register(CNF1_REGISTER, &val1);
    // if (ret != ESP_OK) return ret;

    // ESP_LOGE(TAG, "val = 0x%x", val1);
    // if (val1 != 0x55)
    // {
    //     return ESP_ERR_NOT_FOUND;
    // }
    
    // ret = mcp2515_write_register(CNF1_REGISTER, 0xAA);
    // if (ret != ESP_OK) return ret;

    // ret = mcp2515_read_register(CNF1_REGISTER, &val1);
    // if (ret != ESP_OK) return ret;

    // ESP_LOGE(TAG, "val = 0x%x", val1);
    // if (val1 != 0xAA)
    // {
    //     return ESP_ERR_NOT_FOUND;
    // }

    // ---

    // sets CNF3_REGISTER..CANINTE_REGISTER
    uint8_t buf[4] = { p_mcp2515_obj->timing.cnf3, p_mcp2515_obj->timing.cnf2, p_mcp2515_obj->timing.cnf1, 0x1F};
    ret = mcp2515_write_registers(CNF3_REGISTER, 4, buf);
    if (ret != ESP_OK) return ret;
    // ESP_LOGE(TAG, "buf = {0x%x,0x%x,0x%x, 0x%x}", buf[0], buf[1], buf[2], buf[3]);

    // ret = mcp2515_write_register(CNF1_REGISTER, p_mcp2515_obj->timing.cnf1);
    // if (ret != ESP_OK) return ret;
    // ret = mcp2515_write_register(CNF2_REGISTER, p_mcp2515_obj->timing.cnf2);
    // if (ret != ESP_OK) return ret;
    // ret = mcp2515_write_register(CNF3_REGISTER, p_mcp2515_obj->timing.cnf3);
    // if (ret != ESP_OK) return ret;

    // uint8_t val[4] = {0xFF,0xFF,0xFF,0xFF};
    // ret = mcp2515_read_register(CNF1_REGISTER, val+0);
    // if (ret != ESP_OK) return ret;
    // ret = mcp2515_read_register(CNF2_REGISTER, val+1);
    // if (ret != ESP_OK) return ret;
    // ret = mcp2515_read_register(CNF3_REGISTER, val+2);
    // if (ret != ESP_OK) return ret;
    // ESP_LOGE(TAG, "val = {0x%x,0x%x,0x%x,0x%x}", val[0], val[1], val[2], val[3]);
  
    ret = mcp2515_write_register (BFPCTRL_REGISTER, 0) ; // Deactivate the RXnBF Pins (High Impedance State)
    if (ret != ESP_OK) return ret;

    ret = mcp2515_write_register (TXRTSCTRL_REGISTER, 0); // Set TXnRTS as inputs
    if (ret != ESP_OK) return ret;

    ret = mcp2515_write_register (RXB0CTRL_REGISTER, 1 /* ((uint8_t) inSettings.mRolloverEnable) */ << 2) ; // RXBnCTRL
    if (ret != ESP_OK) return ret;
    ret = mcp2515_write_register (RXB1CTRL_REGISTER, 0x00) ;
    if (ret != ESP_OK) return ret;

    memset(buf, 0, 4);
    ret = mcp2515_write_registers ( RXM0SIDH_REGISTER, 4, buf) ;
    if (ret != ESP_OK) return ret;
    ret = mcp2515_write_registers ( RXM1SIDH_REGISTER, 4, buf) ;
    if (ret != ESP_OK) return ret;

    ret = mcp2515_write_register(CANCTRL_REGISTER, (p_mcp2515_obj->mode << 5) | 0x04);

    uint8_t mode = 0xFF;
    int64_t start = esp_timer_get_time();
    while (mode != p_mcp2515_obj->mode && esp_timer_get_time() - start < 2000000LL)
    {
        ret = mcp2515_read_register(CANSTAT_REGISTER, &mode);
        if (ret != ESP_OK) return ret;
        mode = (mode >> 5) & 0x7;
    }

    return ESP_OK;
}

esp_err_t mcp2515_stop()
{
    return ESP_OK;
}

esp_err_t mcp2515_receive(mcp2515_message_t *message, TickType_t ticks_to_wait)
{
    MCP2515_CHECK(p_mcp2515_obj != NULL, ESP_ERR_INVALID_STATE);
    MCP2515_CHECK(p_mcp2515_obj->rx_queue != NULL, ESP_ERR_INVALID_STATE);
    MCP2515_CHECK(message != NULL, ESP_ERR_INVALID_ARG);

    return (xQueueReceive(p_mcp2515_obj->rx_queue, message, ticks_to_wait) == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
}

// --------------------------------------------------------------------------

static esp_err_t mcp2515_reset()
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.cmd = RESET_COMMAND;
    t.length = 8;
    return spi_device_polling_transmit(p_mcp2515_obj->spi, &t);
}

static esp_err_t mcp2515_rx_status(uint8_t* value)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.cmd = RX_STATUS_COMMAND;
    t.length = 8;
    t.rxlength = 8;
    t.rx_buffer = value;
    return spi_device_polling_transmit(p_mcp2515_obj->spi, &t);
}

static esp_err_t mcp2515_write_register(uint8_t reg, uint8_t value)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags = SPI_TRANS_USE_TXDATA;
    t.cmd = WRITE_COMMAND;
    t.addr = reg;
    t.length = 1 * 8;
    t.tx_data[0] = value;
    return spi_device_polling_transmit(p_mcp2515_obj->spi, &t);
}

static esp_err_t mcp2515_write_registers(uint8_t reg, int n, uint8_t* values)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.cmd = WRITE_COMMAND;
    t.addr = reg;
    t.length = n * 8;
    t.tx_buffer = values;
    return spi_device_polling_transmit(p_mcp2515_obj->spi, &t);
}

static esp_err_t mcp2515_read_register(uint8_t reg, uint8_t* value)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    //t.flags = SPI_TRANS_USE_TXDATA;
    t.cmd = READ_COMMAND;
    t.addr = reg;
    // t.length = 0 * 8;
    t.rxlength = 8;
    t.rx_buffer = value;
    return spi_device_polling_transmit(p_mcp2515_obj->spi, &t);
}

static esp_err_t mcp2515_read_rx_buffer(uint8_t cmd, size_t n, uint8_t* buffer)
{
    spi_transaction_ext_t t;
    memset(&t, 0, sizeof(t));
    t.base.cmd = cmd;
    t.base.flags = SPI_TRANS_VARIABLE_ADDR;
    t.address_bits = 0;
    t.command_bits = 8;
    t.base.rxlength = n;
    t.base.rx_buffer = buffer;
    return spi_device_polling_transmit(p_mcp2515_obj->spi, &t);
}

static esp_err_t mcp2515_bit_modify(uint8_t addr, uint8_t mask, uint8_t data)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags = SPI_TRANS_USE_TXDATA;
    t.cmd = BIT_MODIFY_COMMAND;
    t.addr = addr;
    t.length = 2;
    t.tx_data[0] = mask;
    t.tx_data[1] = data;
    return spi_device_polling_transmit(p_mcp2515_obj->spi, &t);
}

// --------------------------------------------------------------------------

static void mcp2515_handle_rxb_interrupt()
{
    uint8_t rx_status;
    MCP2515_ERR_CHECK(mcp2515_rx_status(&rx_status));

    // ESP_LOGE(TAG, "0x%x", rx_status);
    if (rx_status & 0xC0)
    {
        uint8_t buf[13];
        uint8_t cmd = (rx_status & 0x40) ? READ_FROM_RXB0SIDH_COMMAND : READ_FROM_RXB1SIDH_COMMAND;
        MCP2515_ERR_CHECK(mcp2515_read_rx_buffer(cmd,sizeof buf, buf));
        // mcp2515_bit_modify(CANINTF_REGISTER,(rx_status & 0x40) ? 0x01 : 0x02, 0);

        mcp2515_message_t msg;

        msg.extd = (rx_status & 0x08) != 0;
        msg.rtr = (rx_status & 0x10) != 0;
        msg.identifier = (buf[0] << 3) | (buf[1] >> 5);
        if (msg.extd)
        {
            msg.identifier = (msg.identifier << 17) | ((buf[1] & 3) << 16) | (buf[2] << 8) | buf[3];
        }
        msg.data_length_code = buf[4] & 0xF;
        memcpy(msg.data, buf+5, 8);

        xQueueSendToBack(p_mcp2515_obj->rx_queue, &msg, 0);        
    }
    // if (rx_status & 0x80)
    // {
    //     uint8_t buf[13];
    //     uint8_t cmd = ;
    //     MCP2515_ERR_CHECK(mcp2515_read_rx_buffer(cmd,sizeof buf, buf));
    //     // mcp2515_bit_modify(CANINTF_REGISTER,(rx_status & 0x40) ? 0x01 : 0x02, 0);
    // }
}

static void mcp2515_ihandler_task()
{
    while (1)
    {
        // ESP_LOGE(TAG, "ihandler wait...");
        // xSemaphoreTake (mISRSemaphore, portMAX_DELAY) ;
        uint32_t ulNotifiedValue = ulTaskNotifyTake( pdFALSE, portMAX_DELAY  );
        if( ulNotifiedValue > 0 )
        {
        uint8_t val;
        do
        {
            // ESP_LOGE(TAG, "n=%u", ulNotifiedValue);
            MCP2515_ERR_CHECK(mcp2515_read_register(CANSTAT_REGISTER, &val));

            val = (val >> 1) & 7;
            // ESP_LOGE(TAG, "ihandler CANSTAT = 0x%x", val);

            switch (val)
            {
                // 000 = No interrupt
                // 001 = Error interrupt
                // 010 = Wake-up interrupt
                // 011 = TXB0 interrupt
                // 100 = TXB1 interrupt
                // 101 = TXB2 interrupt
                case 6: // RXB0 interrupt
                case 7: // RXB1 interrupt
                    mcp2515_handle_rxb_interrupt();
                    break;
            }
        } while (val != 0);
        }
    }
}

static void IRAM_ATTR mcp2515_isr(void* p)
{
    if (p_mcp2515_obj != NULL)
    {
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(p_mcp2515_obj->taskHandle, &higherPriorityTaskWoken);
        // xSemaphoreGive (mISRSemaphore) ;
        portYIELD_FROM_ISR(/* higherPriorityTaskWoken */);
// /**/    ESP_LOGE(TAG, "isr");
    }
}
