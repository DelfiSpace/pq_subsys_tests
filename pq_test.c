/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== uartecho.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

#include <ti/drivers/ADC.h>
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Watchdog.h>

/* Example/Board Header files */
#include "OBC_Board.h"

/*
 *  ============ Test functions ============
 */


/*
 *  ------------ UART functions ------------
 *  Each subsystem has the same uart and rs-485 peripherals, so no need for change.
 */

/*Dummy definitions for compalation reasons, comment when necessary*/
//#define PQ9_EN 1
//#define PQ9 1
//#define DBG 1

void rs_test() {

      UART_Handle uart_pq9_bus;
      UART_Params uartParams;

      GPIO_write(PQ9_EN, 1);

      /* Create a UART with data processing off. */
      UART_Params_init(&uartParams);
       uartParams.writeMode = UART_MODE_BLOCKING;
       uartParams.writeDataMode = UART_DATA_BINARY;
       uartParams.readMode = UART_MODE_BLOCKING;
       uartParams.readDataMode = UART_DATA_BINARY;
       uartParams.readReturnMode = UART_RETURN_FULL;
       uartParams.readEcho = UART_ECHO_ON;
       uartParams.baudRate = 9600;

      uart_pq9_bus = UART_open(PQ9, &uartParams);

      UART_write(uart_pq9_bus, "Hello PQ9\n", 10);
}

UART_Handle uart_dbg_bus;

void uart_test() {


      UART_Params uartParams;

      UART_Params_init(&uartParams);
      uartParams.writeMode = UART_MODE_BLOCKING;
      uartParams.writeDataMode = UART_DATA_BINARY;
      uartParams.readMode = UART_MODE_BLOCKING;
      uartParams.readDataMode = UART_DATA_BINARY;
      uartParams.readReturnMode = UART_RETURN_FULL;
      uartParams.readEcho = UART_ECHO_ON;
      uartParams.baudRate = 9600;
      uart_dbg_bus = UART_open(DBG, &uartParams);

      UART_write(uart_dbg_bus, "Hello DBG\n", 10);

}

/*
 *  ------------ FRAM functions ------------
 *  each subsys has only on FRAM, so no need for changes.
 */

// Status register
#define FRAM_WPEN       0x80
#define FRAM_BP1        0x08
#define FRAM_BP0        0x04
#define FRAM_WEL        0x02

// OP-CODE
#define FRAM_WREN       0x06
#define FRAM_WRDI       0x04
#define FRAM_RDSR       0x05
#define FRAM_WRSR       0x01
#define FRAM_READ       0x03
#define FRAM_WRITE      0x02

#define FRAM_MAX_TRANSACTION 20 //mem size
#define FRAM_MEM_SIZE   0x8000  //Memory size

/*Dummy definitions for compalation reasons, comment when necessary*/
//#define FRAM 1
//#define FRAM_CS 1

SPI_Handle spi_fram;

void SPI_readWrite(void *writeBuf,
                   size_t count,
                   void *readBuf) {

  SPI_Transaction spiTransaction;

  spiTransaction.count = count;
  spiTransaction.txBuf = (void *)writeBuf;
  spiTransaction.rxBuf = (void *)readBuf;

  GPIO_write(FRAM_CS, 0);
  SPI_transfer(spi_fram, &spiTransaction);
  GPIO_write(FRAM_CS, 1);

}

/*
 *  Enable data writing in FRAM memory space
 *
 */
void FRAM_write_Enable()
{
  uint8_t tx = FRAM_WREN;
  uint8_t rx = 0;
  SPI_readWrite(&tx, 1, &rx);
}

/**  Disable data writing in FRAM memory space
 *
 */
void FRAM_write_Disable()
{
  uint8_t tx = FRAM_WRDI;
  uint8_t rx = 0;
  SPI_readWrite(&tx, 1, &rx);
}

/**  Returns content of FRAM status register
 *
 *   Returns
 *   unsigned char         status register value
 */
void FRAM_read_Status(uint8_t *res)
{
  uint8_t tx[2] = { FRAM_RDSR, 0x00 };
  uint8_t rx[2] = {0};
  SPI_readWrite(&tx, 2, &rx);
  *res = rx[1];
}

/**  reads sequential memory locations to buffer
 *
 *   Parameters
 *   unsigned int address       start address
 *   void *Ptr                  variable pointer
 *   unsigned int size          total number of bytes
 *
 */
void FRAM_read(uint16_t address, uint8_t *buf, size_t count)
{

  uint8_t tx[FRAM_MAX_TRANSACTION] = { FRAM_READ,
                                       (uint8_t)address >> 8,
                                       (uint8_t)address};
  uint8_t rx[FRAM_MAX_TRANSACTION] = {0};

  SPI_readWrite(&tx, count+3, &rx);
  memcpy(buf, &rx[3], count);

}


/**  writes to sequential memory locations from buffer
 *
 *   Parameters
 *   unsigned int address       start address
 *   void *Ptr                  variable pointer
 *   unsigned int size          total number of bytes
 *
 */
void FRAM_write(uint16_t address, uint8_t *buf, size_t count)
{
  uint8_t tx[FRAM_MAX_TRANSACTION] = { FRAM_WRITE,
                                       (uint8_t)address >> 8,
                                       (uint8_t)address};
  uint8_t rx[FRAM_MAX_TRANSACTION] = {0};
  memcpy(&tx[3], buf, count);

  FRAM_write_Enable();
  SPI_readWrite( &tx, count+3, &rx);
  FRAM_write_Disable();
}

void fram_test() {

         SPI_Params spiParams;

         SPI_Params_init(&spiParams);
         spiParams.frameFormat = SPI_POL0_PHA0;
         spiParams.bitRate = 10000;
         spi_fram = SPI_open(FRAM, &spiParams);

        uint8_t rx[15] = {0};
        uint8_t tx[15];

        FRAM_write_Disable();

        uint8_t res = 0;
        FRAM_read_Status( &res);

        for(uint16_t i=0; i <= 15; i++) {
            tx[i] = i+10;
        }
        FRAM_write(0x16, tx, 15);
        FRAM_read( 0x16, rx, 15);

        for(uint16_t i=0; i < 15; i++) {
                    if(!(rx[i] == i+10)) {
                        while(1) {}
                    }
        }

        // for(uint16_t i=0; i < 200; i++) {
        //  FRAM_write(EPS_FRAM_DEV_ID, 0x16, tx, 15);
        //  FRAM_read(EPS_FRAM_DEV_ID, 0x16, rx, 15);
        //   for(uint32_t d = 0; d < 100000; d++) {}
        // }

}

/*
 *  ------------ I2C functions ------------
 */

/*
 *  ------------ INA functions ------------
 */

/*Dummy definitions for compalation reasons, comment when necessary*/
#define MSP_EXP432P401R_I2CB0 1

I2C_Handle      i2c;

void ina_test(bool init_per) {

    /* Buffers used in this code example */
    uint8_t             txBuffer[10];
    uint8_t             rxBuffer[10];

    /* I2C ina226 read manufacture id test*/
    /* Initialize all buffers */
    for (uint8_t i = 0; i < 10; i++) {
        rxBuffer[i] = 0x00;
        txBuffer[i] = 0x00;
    }

    if(init_per) {
        I2C_Params      i2cParams;

      I2C_Params_init(&i2cParams);
      i2cParams.transferMode = I2C_MODE_BLOCKING;
      i2cParams.bitRate = I2C_100kHz;

      i2c = I2C_open(MSP_EXP432P401R_I2CB0, &i2cParams);
      //if (i2c == NULL) {
      //    while (1);
      //}
    }

    // response
    I2C_Transaction i2cTransaction;

        /*
         * FEh Manufacturer ID Register
         */
        txBuffer[0] = 0xFE;

        /*
         * Response from slave for GETSTATUS Cmd
         * rxBuffer[0] = status
         */
        i2cTransaction.slaveAddress = 0x48;
        i2cTransaction.writeBuf = txBuffer;
        i2cTransaction.writeCount = 1;
        i2cTransaction.readBuf = rxBuffer;
        i2cTransaction.readCount = 2;

    uint8_t i2c_add = 0;

    uint8_t writeBuf[2] = {0xFF, 0};
    uint8_t readBuf[2] = {0};

    i2cTransaction.slaveAddress = 0x40;
    i2cTransaction.writeBuf = writeBuf;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = readBuf;
    i2cTransaction.readCount = 2;
    I2C_transfer(i2c, &i2cTransaction);
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{

    /* Call driver init functions */
    GPIO_init();
    UART_init();
    SPI_init();

    uart_test();
    rs_test();
    fram_test();

    /* Loop forever echoing */
    while (1) {

    }
}


