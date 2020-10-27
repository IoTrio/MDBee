#ifndef HARDWARE_UART
#define HARDWARE_UART

#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

void hw_uart_init(void);
int sendData(const char* logName, const char* data);

#endif /* HARDWARE_UART */
