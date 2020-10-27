/**
 * MDB Software Serial Interface
 * 
 * IoTrio GmbH
 */

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "mdb_interface.h"

static xQueueHandle mdb_rx_gpio_evt_queue = NULL;  // ISR events from RX GPIOs
static xQueueHandle mdb_rx_dispatch_to_analyzer_vmc_queue;
static xQueueHandle mdb_rx_dispatch_to_analyzer_peri_queue;

static void IRAM_ATTR gpio_rx_evt_isr_handler(void* arg) {
    rx_events rx_event;
    rx_event.evt_time = esp_timer_get_time();
    rx_event.mdb_evt = gpio_get_level((uint32_t) arg);
    rx_event.gpio = (uint32_t) arg;
    xQueueSendFromISR(mdb_rx_gpio_evt_queue, &rx_event, NULL);
}

static void mdb_rx_evt_dispatcher(void* arg);
static void mdb_rx_evt_analyzer_vmc(void* arg);
static void mdb_rx_evt_analyzer_peri(void* arg);

static void mdb_vmc_periodic_timer_callback(void* arg);

// For the timer - could maybe be moved to callback arg
static uint_fast16_t mdb_vmc_timer_data;
static uint_fast8_t mdb_vmc_timer_bit_index = 0;
static uint_fast16_t mdb_peri_timer_data;
static uint_fast8_t mdb_peri_timer_bit_index = 0;

static const esp_timer_create_args_t mdb_vmc_periodic_timer_args = {
    .callback = &mdb_vmc_periodic_timer_callback,
    .name = "MDB VMC periodic"
};
static esp_timer_handle_t mdb_vmc_periodic_timer;

static xQueueHandle mdb_vmc_tx_data_queue = NULL;
static xQueueHandle mdb_peri_tx_data_queue = NULL;

void mdb_init(void) {
    gpio_config_t mdb_io_cfg;
    // Outputs
    mdb_io_cfg.intr_type = GPIO_INTR_DISABLE;
    mdb_io_cfg.mode = GPIO_MODE_OUTPUT;
    mdb_io_cfg.pin_bit_mask = MBD_OUTPUT_PIN_SEL;
    mdb_io_cfg.pull_down_en = 0;
    mdb_io_cfg.pull_up_en = 0;
    gpio_config(&mdb_io_cfg);

    // Set MDBs idle (possibly inverted)
    gpio_set_level(MDB_VMC_TXD, MDB_VMC_TXD_CLR);
    gpio_set_level(MDB_PERI_TXD, MDB_PERI_TXD_CLR);

    // Inputs
    mdb_io_cfg.intr_type = GPIO_INTR_ANYEDGE;
    mdb_io_cfg.mode = GPIO_MODE_INPUT;
    mdb_io_cfg.pin_bit_mask = MBD_INPUT_PIN_SEL;
    mdb_io_cfg.pull_up_en = 1;
    mdb_io_cfg.pull_down_en =0;
    gpio_config(&mdb_io_cfg);

    // Queue for GPIO events on MDB interface
    mdb_rx_gpio_evt_queue = xQueueCreate(128, sizeof(rx_events));

    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    gpio_isr_handler_add(MDB_VMC_RXD, gpio_rx_evt_isr_handler, (void*) MDB_VMC_RXD);
    gpio_isr_handler_add(MDB_PERI_RXD, gpio_rx_evt_isr_handler, (void*) MDB_PERI_RXD);
    
    mdb_rx_dispatch_to_analyzer_vmc_queue = xQueueCreate(128, sizeof(rx_events));
    mdb_rx_dispatch_to_analyzer_peri_queue = xQueueCreate(128, sizeof(rx_events));
    xTaskCreate(mdb_rx_evt_analyzer_vmc, "mdb_rx_evt_analyzer_vmc", 2048, NULL, 10, NULL);
    xTaskCreate(mdb_rx_evt_analyzer_peri, "mdb_rx_evt_analyzer_peri", 2048, NULL, 10, NULL);
    xTaskCreate(mdb_rx_evt_dispatcher, "mdb_rx_evt_dispatcher", 2048, NULL, 9, NULL);
    
    // Timer to generate "bit timing" on the MDB UART and queue for data to send
    esp_timer_create(&mdb_vmc_periodic_timer_args, &mdb_vmc_periodic_timer);
    mdb_vmc_tx_data_queue = xQueueCreate(128, sizeof(uint16_t));
    mdb_peri_tx_data_queue = xQueueCreate(128, sizeof(uint16_t));
    printf("Start periodic Timer\n");
    esp_timer_start_periodic(mdb_vmc_periodic_timer, MDB_SYMBOL_TIME_US);

}

void mdb_send(uint32_t interface, uint8_t mode, uint8_t data) {

    // Start bit, data, mode, stop bit is 0 anyways.
    uint16_t mdb_data = 1 | \
                        ((~data & 0xFF) << 1) | \
                        ((~mode & 0x1) << 9);

    // Invert bits if UART is inverted
    if((interface == MDB_INTERFACE_VMC && MDB_VMC_TXD_SET == 0) || (interface == MDB_INTERFACE_PERI && MDB_PERI_TXD_SET == 0)) {
        // data ^= 0xFF;
        // mode ^= 0x01;
        mdb_data ^= 0xFFFF;
    }

    switch (interface)
    {
    case MDB_INTERFACE_VMC:
        xQueueSend(mdb_vmc_tx_data_queue, &mdb_data, portMAX_DELAY);
        break;
    case MDB_INTERFACE_PERI:
        xQueueSend(mdb_peri_tx_data_queue, &mdb_data, portMAX_DELAY);
        break;
    default:
        printf("Invalid interface\n");
        break;
    }
}

void mdb_read(uint32_t interface, uint8_t* mode, uint8_t* data) {
    // Todo
}

int32_t mdb_data_avail(uint32_t interface) {
    return 0;  // Todo
}

static void mdb_rx_evt_dispatcher(void* arg) {
    rx_events rx_event;
    for(;;) {
        // Check if new events came from the MDB RX ISR
        while(xQueueReceive(mdb_rx_gpio_evt_queue, &rx_event, portMAX_DELAY)) {
            // printf("GPIO event at %lli on GPIO %i, level %i\n", rx_event.evt_time, rx_event.gpio, rx_event.mdb_evt);
            if(rx_event.gpio == MDB_VMC_RXD) {
                xQueueSend(mdb_rx_dispatch_to_analyzer_vmc_queue, &rx_event, 0);
            } else if (rx_event.gpio == MDB_PERI_RXD) {
                xQueueSend(mdb_rx_dispatch_to_analyzer_peri_queue, &rx_event, 0);
            } else {
                printf("Invalid GPIO in %s", __func__);
            }
        }
    }
}

static void mdb_rx_evt_analyzer_vmc(void* arg) {
    printf("Starting MDB RX Analyzer VMC...\n");
    
    uint8_t rx_data_vmc = 0x0;
    uint8_t rx_mode_vmc = 0x0;
    int32_t rx_counter_vmc = -1;
    uint64_t rx_start_time_vmc = 0;
    uint8_t rx_last_symbol_vmc = 0;

    rx_events rx_event;
    for(;;) {
        vTaskDelay(10 / portTICK_RATE_MS);
        
        // Check if there is data to assemble a byte from the VMC
        if(rx_counter_vmc == -1) {  // Idle - Waiting for start bit
            if(xQueueReceive(mdb_rx_dispatch_to_analyzer_vmc_queue, &rx_event, 5/portTICK_RATE_MS)) {
                // printf("SB\n");
                rx_start_time_vmc = rx_event.evt_time;
                rx_last_symbol_vmc = rx_event.mdb_evt;
                rx_data_vmc = 0;
                rx_mode_vmc = 0;
                rx_counter_vmc = 1;
            }
        } else if(rx_counter_vmc == 9) { // Mode bit
            // Mode symbol is determined either by GPIO event or through time out.
            uint64_t expected_symbol_time = rx_start_time_vmc + (rx_counter_vmc * MDB_SYMBOL_TIME_US);
            uint64_t current_time = esp_timer_get_time(); 
            if (current_time > (expected_symbol_time + 0.1 * MDB_SYMBOL_TIME_US)) {
                // Peek if there is a new event in the RX queue
                if (xQueuePeek(mdb_rx_dispatch_to_analyzer_vmc_queue, &rx_event, 5/portTICK_RATE_MS)) {  // Validate sample
                    if ((rx_event.evt_time > (expected_symbol_time - 0.05 * MDB_SYMBOL_TIME_US)) && (rx_event.evt_time < (expected_symbol_time + 0.05 * MDB_SYMBOL_TIME_US))) {
                        rx_last_symbol_vmc = rx_event.mdb_evt;
                        xQueueReceive(mdb_rx_dispatch_to_analyzer_vmc_queue, &rx_event, 0);  // Empty queue
                        // printf("Stop bit received as symbol at %i.\n", (int)(rx_event.evt_time - rx_start_time_vmc));
                    } else {  // Sample in queue but not related to the current symbol
                        // printf("Stop assumed, next symbol waiting.\n");
                    }
                } else {  // No sample in queue, assuming time-out
                    // printf("Stop through time-out at %i\n", (int)(rx_event.evt_time - rx_start_time_vmc));
                }
                rx_mode_vmc = rx_last_symbol_vmc;
                rx_counter_vmc++;
            }
        } else if (rx_counter_vmc == 10) { // Stop bit
            // Stop symbol is determined either by GPIO event or through time out.
            uint64_t expected_symbol_time = rx_start_time_vmc + (rx_counter_vmc * MDB_SYMBOL_TIME_US);
            uint64_t current_time = esp_timer_get_time(); 
            if (current_time > (expected_symbol_time + 0.1 * MDB_SYMBOL_TIME_US)) {
                // Peek if there is a new event in the RX queue
                if (xQueuePeek(mdb_rx_dispatch_to_analyzer_vmc_queue, &rx_event, 5/portTICK_RATE_MS)) {  // Validate sample
                    if ((rx_event.evt_time > (expected_symbol_time - 0.05 * MDB_SYMBOL_TIME_US)) && (rx_event.evt_time < (expected_symbol_time + 0.05 * MDB_SYMBOL_TIME_US))) {
                        xQueueReceive(mdb_rx_dispatch_to_analyzer_vmc_queue, &rx_event, 0);  // Empty queue
                        // printf("Stop bit received as symbol at %i.\n", (int)(rx_event.evt_time - rx_start_time_vmc));
                    } else {  // Sample in queue but not related to the current symbol
                        // printf("Stop assumed, next symbol waiting.\n");
                    }
                } else {  // No sample in queue, assuming time-out
                    // printf("Stop through time-out at %i\n", (int)(rx_event.evt_time - rx_start_time_vmc));
                }
                // Invert bits if UART is inverted
                if (MDB_VMC_TXD_CLR == 0) {
                    rx_data_vmc ^= 0xFF;
                    rx_mode_vmc ^= 0x01;
                }
                rx_counter_vmc = -1;  // Ready to receive again
                printf("0x%02x, mode %i\n", rx_data_vmc, rx_mode_vmc);
            }
        } else if (rx_counter_vmc <= 8 ) { // Receiving bit 1 to 8 and mode bit
            // Symbols are determined either by GPIO event or through time out.
            uint64_t expected_symbol_time = rx_start_time_vmc + (rx_counter_vmc * MDB_SYMBOL_TIME_US);
            uint64_t current_time = esp_timer_get_time(); 
            if (current_time > (expected_symbol_time + 0.1 * MDB_SYMBOL_TIME_US)) {
                if (xQueuePeek(mdb_rx_dispatch_to_analyzer_vmc_queue, &rx_event, 5/portTICK_RATE_MS)) {  // Validate sample
                    if ((rx_event.evt_time > (expected_symbol_time - 0.03 * MDB_SYMBOL_TIME_US)) && (rx_event.evt_time < (expected_symbol_time + 0.03 * MDB_SYMBOL_TIME_US))) {
                        rx_last_symbol_vmc = rx_event.mdb_evt;
                        xQueueReceive(mdb_rx_dispatch_to_analyzer_vmc_queue, &rx_event, 0);  // Empty queue
                        // printf("Stop bit received as symbol at %i.\n", (int)(rx_event.evt_time - rx_start_time_vmc));
                    } else {  // Sample in queue but not related to the current symbol
                        // printf("Stop assumed, next symbol waiting.\n");
                    }
                } else {  // No sample in queue, assuming time-out
                    // printf("Stop through time-out at %i\n", (int)(rx_event.evt_time - rx_start_time_vmc));
                }
                rx_data_vmc |= rx_last_symbol_vmc << (rx_counter_vmc - 1);
                rx_counter_vmc++;
            }
        }
    }
}

static void mdb_rx_evt_analyzer_peri(void* arg) {
    printf("Starting MDB RX Analyzer Peri...\n");
    
    uint8_t rx_data_peri = 0x0;
    uint8_t rx_mode_peri = 0x0;
    int32_t rx_counter_peri = -1;
    uint64_t rx_start_time_peri = 0;
    uint8_t rx_last_symbol_peri = 0;

    rx_events rx_event;
    for(;;) {
        vTaskDelay(10 / portTICK_RATE_MS);
        
        // Check if there is data to assemble a byte from the Peri
        if(rx_counter_peri == -1) {  // Idle - Waiting for start bit
            if(xQueueReceive(mdb_rx_dispatch_to_analyzer_peri_queue, &rx_event, 5/portTICK_RATE_MS)) {
                // printf("SB\n");
                rx_start_time_peri = rx_event.evt_time;
                rx_last_symbol_peri = rx_event.mdb_evt;
                rx_data_peri = 0;
                rx_mode_peri = 0;
                rx_counter_peri = 1;
            }
        } else if(rx_counter_peri == 9) { // Mode bit
            // Mode symbol is determined either by GPIO event or through time out.
            uint64_t expected_symbol_time = rx_start_time_peri + (rx_counter_peri * MDB_SYMBOL_TIME_US);
            uint64_t current_time = esp_timer_get_time(); 
            if (current_time > (expected_symbol_time + 0.1 * MDB_SYMBOL_TIME_US)) {
                // Peek if there is a new event in the RX queue
                if (xQueuePeek(mdb_rx_dispatch_to_analyzer_peri_queue, &rx_event, 5/portTICK_RATE_MS)) {  // Validate sample
                    if ((rx_event.evt_time > (expected_symbol_time - 0.05 * MDB_SYMBOL_TIME_US)) && (rx_event.evt_time < (expected_symbol_time + 0.05 * MDB_SYMBOL_TIME_US))) {
                        rx_last_symbol_peri = rx_event.mdb_evt;
                        xQueueReceive(mdb_rx_dispatch_to_analyzer_peri_queue, &rx_event, 0);  // Empty queue
                        // printf("Stop bit received as symbol at %i.\n", (int)(rx_event.evt_time - rx_start_time_peri));
                    } else {  // Sample in queue but not related to the current symbol
                        // printf("Stop assumed, next symbol waiting.\n");
                    }
                } else {  // No sample in queue, assuming time-out
                    // printf("Stop through time-out at %i\n", (int)(rx_event.evt_time - rx_start_time_peri));
                }
                rx_mode_peri = rx_last_symbol_peri;
                rx_counter_peri++;
            }
        } else if (rx_counter_peri == 10) { // Stop bit
            // Stop symbol is determined either by GPIO event or through time out.
            uint64_t expected_symbol_time = rx_start_time_peri + (rx_counter_peri * MDB_SYMBOL_TIME_US);
            uint64_t current_time = esp_timer_get_time();
            if (current_time > (expected_symbol_time + 0.1 * MDB_SYMBOL_TIME_US)) {
                // Peek if there is a new event in the RX queue
                if (xQueuePeek(mdb_rx_dispatch_to_analyzer_peri_queue, &rx_event, 5/portTICK_RATE_MS)) {  // Validate sample
                    if ((rx_event.evt_time > (expected_symbol_time - 0.05 * MDB_SYMBOL_TIME_US)) && (rx_event.evt_time < (expected_symbol_time + 0.05 * MDB_SYMBOL_TIME_US))) {
                        xQueueReceive(mdb_rx_dispatch_to_analyzer_peri_queue, &rx_event, 0);  // Empty queue
                        // printf("Stop bit received as symbol at %i.\n", (int)(rx_event.evt_time - rx_start_time_peri));
                    } else {  // Sample in queue but not related to the current symbol
                        // printf("Stop assumed, next symbol waiting.\n");
                    }
                } else {  // No sample in queue, assuming time-out
                    // printf("Stop through time-out at %i\n", (int)(rx_event.evt_time - rx_start_time_peri));
                }
                // Invert bits if UART is inverted
                if (MDB_PERI_TXD_CLR == 0) {
                    rx_data_peri ^= 0xFF;
                    rx_mode_peri ^= 0x01;
                }
                rx_counter_peri = -1;  // Ready to receive again
                printf("0x%02x, mode %i\n", rx_data_peri, rx_mode_peri);
            }
        } else if (rx_counter_peri <= 8 ) { // Receiving bit 1 to 8 and mode bit
            // Symbols are determined either by GPIO event or through time out.
            uint64_t expected_symbol_time = rx_start_time_peri + (rx_counter_peri * MDB_SYMBOL_TIME_US);
            uint64_t current_time = esp_timer_get_time(); 
            if (current_time > (expected_symbol_time + 0.1 * MDB_SYMBOL_TIME_US)) {
                if (xQueuePeek(mdb_rx_dispatch_to_analyzer_peri_queue, &rx_event, 5/portTICK_RATE_MS)) {  // Validate sample
                    if ((rx_event.evt_time > (expected_symbol_time - 0.03 * MDB_SYMBOL_TIME_US)) && (rx_event.evt_time < (expected_symbol_time + 0.03 * MDB_SYMBOL_TIME_US))) {
                        rx_last_symbol_peri = rx_event.mdb_evt;
                        xQueueReceive(mdb_rx_dispatch_to_analyzer_peri_queue, &rx_event, 0);  // Empty queue
                        // printf("Stop bit received as symbol at %i.\n", (int)(rx_event.evt_time - rx_start_time_peir));
                    } else {  // Sample in queue but not related to the current symbol
                        // printf("Stop assumed, next symbol waiting.\n");
                    }
                } else {  // No sample in queue, assuming time-out
                    // printf("Stop through time-out at %i\n", (int)(rx_event.evt_time - rx_start_time_peri));
                }
                rx_data_peri |= rx_last_symbol_peri << (rx_counter_peri - 1);
                rx_counter_peri++;
            }
        }
    }
}

static void mdb_vmc_periodic_timer_callback(void* arg) {
    // Check for new VMC data in the queue to send
    if(mdb_vmc_timer_bit_index == 0) {
        if(xQueueReceive(mdb_vmc_tx_data_queue, &mdb_vmc_timer_data, 0)) {
            mdb_vmc_timer_bit_index = 11;
        }
    }
    // Check for new PERI data in the queue to send
    if(mdb_peri_timer_bit_index == 0) {
        if(xQueueReceive(mdb_peri_tx_data_queue, &mdb_peri_timer_data, 0)) {
            mdb_peri_timer_bit_index = 11;
        }
    }
    // Sending data to VMC
    if(mdb_vmc_timer_bit_index > 0) {
        gpio_set_level(MDB_VMC_TXD, (mdb_vmc_timer_data & 0x1));
        mdb_vmc_timer_data >>= 1;
        mdb_vmc_timer_bit_index--;
    }
    // Sending data to PERI
    if(mdb_peri_timer_bit_index > 0) {
        gpio_set_level(MDB_PERI_TXD, (mdb_peri_timer_data & 0x1));
        mdb_peri_timer_data >>= 1;
        mdb_peri_timer_bit_index--;
    }
}
