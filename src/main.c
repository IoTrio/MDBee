/**
 * MDBee
 * USB to MDB (Multi-drop bus) used by vending machines.
 * 
 * IoTrio GmbH
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "driver/uart.h"
#include <hardware_uart.h>
#include <mdb_interface.h>

#define MDBEE_VERSION  "0.3"

/**
 * USB-HW-UART
 * TXD  GPIO4
 * RXD  GPIO5
 * 
 * SW-UART-VMC
 * TXD  GPIO18
 * RXD  GPIO19
 * DBG  GPIO23
 * 
 * SW-UART-PERI
 * TXD  GPIO21
 * RXD  GPIO22
 * DBG  GPIO25
 */

/**
 * Mode bit
 * 'V'  Send byte to VMC, Mode bit set
 * 'v'  Send byte to VMC, Mode bit clear
 * 'P'  Send byte to Peripheral, Mode bit set
 * 'p'  Send byte to Peripheral, Mode bit clear
 * 
 * MDBee to VMC
 * Mode bit must be set on last byte.
 *
 * MDBee to Peripheral
 * Mode bit set: ADDRESS bytes
 * Mode bit clear: DATA bytes
 */
#define MODE_BIT_SET_VMC   'V'
#define MODE_BIT_CLR_VMC   'v'
#define MODE_BIT_SET_PERI  'P'
#define MODE_BIT_CLR_PERI  'p'

static void rx_task(void *arg);
static void tx_task(void *arg);

void app_main(void)
{
    // static const char *MAIN_TASK_TAG = "MAIN_TASK";

    // sw_uart_init_vmc();
    // sw_uart_init_peri();
    hw_uart_init();
    mdb_init();

    printf("# MDBee\n");
    printf("# =====\n");
    printf("# Version %s\n\n", MDBEE_VERSION);

    fflush(stdout);

    xTaskCreate(rx_task, "hw_uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);

    //while(1);

    while(1) {
        vTaskDelay(500 / portTICK_RATE_MS);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_SET, 0x35);
        // mdb_send(MDB_INTERFACE_PERI, MDB_MODE_SET, 0x35);
        // mdb_send(MDB_INTERFACE_PERI, MDB_MODE_CLR, 0xF0);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_CLR, 0xF0);
        vTaskDelay(10 / portTICK_RATE_MS);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_SET, 0x00);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_SET, 0x01);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_SET, 0x02);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_SET, 0x03);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_SET, 0x04);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_SET, 0x05);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_SET, 0x06);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_SET, 0x07);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_CLR, 0x08);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_CLR, 0x09);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_CLR, 0x0A);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_CLR, 0x0B);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_CLR, 0x0C);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_CLR, 0x0D);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_CLR, 0x0E);
        mdb_send(MDB_INTERFACE_VMC, MDB_MODE_CLR, 0x0F);
        
        vTaskDelay(5000 / portTICK_RATE_MS);
    }
}

/**
 * Hardware UART receives new bytes
 * byte, mode, [byte, mode, ...]
 */
static void rx_task(void *arg)
{
    printf("Starting rx_task.\n");
    static const char *RX_TASK_TAG = "USB_RX_TASK";

    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    enum rxStates {idle, mode_received, byte_received};
    enum rxStates rxState = idle;

    uint8_t rxMode = 0x0;
    uint8_t rxByte = 0x0;
    int rxByteAvail = 0;

    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    
        switch (rxState)  // Process the date after two bytes (mode, byte) are received.
        {
        case idle:
            // printf("Idle.\n");
            rxByteAvail = uart_read_bytes(UART_NUM_1, &rxMode, 1, 1000 / portTICK_RATE_MS);
            if(rxByteAvail > 0) {
                // printf("Mode received.\n");
                rxState = mode_received;
            }
            break;
        case mode_received:
            rxByteAvail = uart_read_bytes(UART_NUM_1, &rxByte, 1, 1000 / portTICK_RATE_MS);
            if(rxByteAvail > 0) {
                // printf("Byte received.\n");
                rxState = byte_received;
            }
            break;
        case byte_received:
            // printf("Mode: %x\n", rxMode);
            // printf("Byte: %x, ", rxByte);
            switch (rxMode)
            {
            case MODE_BIT_SET_VMC:
                // printf("Send last byte to VMC.\n");
                mdb_send(MDB_INTERFACE_VMC, MDB_MODE_SET, rxByte);
                break;
            case MODE_BIT_CLR_VMC:
                // printf("Send byte to VMC. Expecting next byte.\n");
                mdb_send(MDB_INTERFACE_VMC, MDB_MODE_CLR, rxByte);
                break;
            case MODE_BIT_SET_PERI:
                // printf("Send address to PERI.\n");
                mdb_send(MDB_INTERFACE_PERI, MDB_MODE_SET, rxByte);
                break;
            case MODE_BIT_CLR_PERI:
                // printf("Send data to PERI.\n");
                mdb_send(MDB_INTERFACE_PERI, MDB_MODE_CLR, rxByte);
                break;
            default:
                printf("Incorrect mode byte!\n");
                break;
            }
            rxState = idle;
            break;
        default:
            break;
        }
    }
}
