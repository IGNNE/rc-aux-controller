#ifndef _TWISLAVE_H
#define _TWISLAVE_H

#include <stdint.h>
#include <Wire.h>

/**
 * @file i2cslave.h I2C slave library, inspired from https://rn-wissen.de/wiki/index.php/TWI_Slave_mit_avr-gcc#Slave_Testprogramm,
 * but with Wire and in C++ (original code didn't work on my 328p, no idea why)
 */


/**
 * Try to set a byte in the buffer. If the byte is marked read-only by ::I2CDATA_RO_BYTES or out of range, 
 * do nothing.
 */
void try_set_i2cdata(uint8_t index, uint8_t data);

/**
 * Like ::try_set_i2cdata, but writes read-only bytes
 */
void force_set_i2cdata(uint8_t index, uint8_t data);

/**
 * Maybe some "adc on demand"? But probably not worth it, ::loop() is empty anyway.
 */
uint8_t get_i2cdata(uint8_t index);

/**
 * Initialize the lib
*/
void init_i2c_slave(uint8_t adr);

#endif // _TWISLAVE_H


