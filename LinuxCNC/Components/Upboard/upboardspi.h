#ifndef UPBOARD_H
#define UPBOARD_H

#include <stdint.h>

#define UPBOARD_VERSION 10000 /* Version 1.00 */

typedef enum
{
    SPI_BIT_ORDER_LSBFIRST = 0,  /*!< LSB First */
    SPI_BIT_ORDER_MSBFIRST = 1   /*!< MSB First */
}SPIBitOrder;

/*! This means pin HIGH, true, 3.3volts on a pin. */
//#define HIGH 0x1
/*! This means pin LOW, false, 0volts on a pin. */
//#define LOW  0x0

/*! Return the minimum of 2 numbers */
#ifndef MIN
#define MIN(a, b) (a < b ? a : b)
#endif

typedef enum
{
    UP_GPIO_P1_03        =  0,  /*!< Pin P1-03 */
    UP_GPIO_P1_05        =  1,  /*!< Pin P1-05 */
    UP_GPIO_P1_07        =  4,  /*!< Pin P1-07 */
    UP_GPIO_P1_08        = 14,  /*!< Pin P1-08, defaults to alt function 0 UART0_TXD */
    UP_GPIO_P1_10        = 15,  /*!< Pin P1-10, defaults to alt function 0 UART0_RXD */
    UP_GPIO_P1_11        = 17,  /*!< Pin P1-11 */
    UP_GPIO_P1_12        = 18,  /*!< Pin P1-12, can be PWM channel 0 in ALT FUN 5 */
    UP_GPIO_P1_13        = 21,  /*!< Pin P1-13 */
    UP_GPIO_P1_15        = 22,  /*!< Pin P1-15 */
    UP_GPIO_P1_16        = 23,  /*!< Pin P1-16 */
    UP_GPIO_P1_18        = 24,  /*!< Pin P1-18 */
    UP_GPIO_P1_19        = 10,  /*!< Pin P1-19, MOSI when SPI0 in use */
    UP_GPIO_P1_21        =  9,  /*!< Pin P1-21, MISO when SPI0 in use */
    UP_GPIO_P1_22        = 25,  /*!< Pin P1-22 */
    UP_GPIO_P1_23        = 11,  /*!< Pin P1-23, CLK when SPI0 in use */
    UP_GPIO_P1_24        =  8,  /*!< Pin P1-24, CE0 when SPI0 in use */
    UP_GPIO_P1_25        = 16,  /*!< Pin P1-25, Watchdog reset GPIO output > */
    UP_GPIO_P1_26        =  7,  /*!< Pin P1-26, CE1 when SPI0 in use */

} UpGPIOpin;
// CPU x5-Z8350

//gpio

//spi

//extern int upboard_gpio_set(void);
//extern int upboard_gpio_clr(void);
//extern void upboard_spi_transfernb(uint8_t, uint8_t, uint32_t)


extern int upboard_spi_init(void);
extern int upboard_spi_close(void);

//extern void  upboard_spi_set_debug(uint8_t debug);

extern unsigned int upboard_spi_version(void);


#endif