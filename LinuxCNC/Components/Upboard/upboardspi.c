/* upboardspi.c
// Support for UP Board SPI & GPIO
// Userland Linux spidev
// 
//
// Adapted from bcm2835.c
// C and C++ support for Broadcom BCM 2835 as used in Raspberry Pi
// http://elinux.org/RPi_Low-level_peripherals
// http://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
//
// Author: Mike McCauley
// Copyright (C) 2011-2013 Mike McCauley
// $Id: bcm2835.c,v 1.28 2020/01/11 05:07:13 mikem Exp mikem $
*/
#include <stdint.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include "upboardspi.h"
#include "c_gpio.h"
#include "c_gpio.c"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static const char *device = "/dev/spidev2.0"; //use CS0 device, the CS1 is GPIO pin and slow (10x slower than RPi)
static uint32_t mode;
static uint8_t bits = 8;
static char *input_file;
static char *output_file;
static uint32_t speed = 5000000;
static uint16_t delay=0;
static int verbose;
static int transfer_size;
static int iterations;
static int interval = 5; /* interval in seconds for showing transfer rate */
static int fd;

static char *input_tx;

static uint8_t debug = 0;

static void pabort(const char *s)
{
	if (errno != 0)
		perror(s);
	else
		printf("%s\n", s);

	abort();
}

static void hex_dump(const void *src, size_t length, size_t line_size, char *prefix)
{
	int i = 0;
	const unsigned char *address = src;
	const unsigned char *line = address;
	unsigned char c;

	printf("%s | ", prefix);
	while (length-- > 0) {
		printf("%02X ", *address++);
		if (!(++i % line_size) || (length == 0 && i % line_size)) {
			if (length == 0) {
				while (i++ % line_size)
					printf("__ ");
			}
			printf(" |");
			while (line < address) {
				c = *line++;
				printf("%c", (c < 32 || c > 126) ? '.' : c);
			}
			printf("|\n");
			if (length > 0)
				printf("%s | ", prefix);
		}
	}
}

int rt_upboard_init(void)
{
    int res = setup();
	if(res>0) 
	{
		printf("GPIO setup error: %x\n",res);
		return 0;
	}
    return 1; // OK    
}

int upboard_spi_init(void)
{
	//res = init_gpio(16);
    return 1; // OK    
}

// open spi device
int upboard_spi_begin(void)
{
    fd = open(device, O_RDWR);
	if (fd < 0){
		pabort("can't open device");
        return 0;
    } 
    return 1; // OK  
}


int upboard_spi_close(void)
{
    fd = 0;
    return 1;
}

//TBD
void upboard_spi_setDataMode()
{
   //  BCM2835_SPI_MODE0 = 0,  /*!< CPOL = 0, CPHA = 0 */
   
     mode |= SPI_CPHA;
    //mode |= SPI_CPOL;
 
}

//Set SPI speed Hz
void upboard_spi_set_speed( uint32_t speed ){
    int ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");
}

// internal spi transfer
void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	int ret;
	int out_fd;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

	if (mode & SPI_TX_QUAD)
		tr.tx_nbits = 4;
	else if (mode & SPI_TX_DUAL)
		tr.tx_nbits = 2;
    if (mode & SPI_RX_QUAD)
		tr.rx_nbits = 4;
	else if (mode & SPI_RX_DUAL)
		tr.rx_nbits = 2;
	if (!(mode & SPI_LOOP)) {
		if (mode & ( SPI_TX_QUAD | SPI_TX_DUAL))
			tr.rx_buf = 0;
		else if (mode & (SPI_RX_QUAD | SPI_RX_DUAL))
			tr.tx_buf = 0;
	}

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		pabort("can't send spi message");

	if (verbose)
		hex_dump(tx, len, 32, "TX");

	if (verbose)
		hex_dump(rx, len, 32, "RX");
}

// SPI buffered transfer 
void upboard_spi_transfernb(uint8_t *tx, uint8_t *rx, uint32_t len)
{
	tx = malloc(len);
	if (!tx)
		pabort("can't allocate tx buffer");

	rx = malloc(len);
	if (!rx)
		pabort("can't allocate rx buffer");

	transfer(fd, tx, rx, len);
	free(rx);
	free(tx);
}

// Set MSB/LSB order: SPI_BIT_ORDER_LSBFIRST / SPI_BIT_ORDER_MSBFIRST
void upboard_spi_setBitOrder(uint8_t order)
{
	int lsb_first = (order==SPI_BIT_ORDER_LSBFIRST)?0:1;
	int ret = ioctl(fd, SPI_IOC_RD_LSB_FIRST, &lsb_first);
	if (ret==-1) {
		pabort("ioctl(SPI_IOC_RD_LSB_FIRST) failed");
	}
	ret = ioctl(fd, SPI_IOC_WR_LSB_FIRST, &lsb_first);
	if (ret==-1) {
		pabort("ioctl(SPI_IOC_WR_LSB_FIRST) failed");
	}
}

/*
* GPIO
*/

void upboard_gpio_setup(int pin,int direction)
{
	setup_gpio(pin,direction,0);
}

void upboard_gpio_close()
{
	cleanup();
}

/* Set output pin */
void upboard_gpio_set(int pin)
{
   output_gpio(pin,1);
}

/* Clear output pin */
void upboard_gpio_clr(int pin)
{
   output_gpio(pin,0);
}

int upboard_gpio_direction(int pin, int direction)
{
   if(gpio_function(pin)!=direction)
   {
   		output_gpio(pin,1);
		printf("toggle direction");
   }
   return gpio_function(pin);
}

/* SPI bit order. SPI0 only supports MSBFIRST, so we instead 
 * have a software based bit reversal, based on a contribution by Damiano Benedetti
 */

//static uint8_t spi_bit_order = SPI_BIT_ORDER_MSBFIRST;

// static uint8_t correct_order(uint8_t b)
// {
//     if (spi_bit_order == SPI_BIT_ORDER_LSBFIRST)
// 	return byte_reverse_table[b];
//     else
// 	return b;
// }

// static uint8_t byte_reverse_table[] = 
// {
//     0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
//     0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
//     0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
//     0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
//     0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
//     0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
//     0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
//     0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
//     0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
//     0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
//     0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
//     0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
//     0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
//     0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
//     0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
//     0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
//     0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
//     0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
//     0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
//     0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
//     0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
//     0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
//     0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
//     0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
//     0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
//     0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
//     0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
//     0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
//     0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
//     0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
//     0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
//     0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff
// };
