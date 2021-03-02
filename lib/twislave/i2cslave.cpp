#include "i2cslave.h"

/// anonymous namespace to prevent direct variable access
namespace {

/// size of the buffer
#define I2CDATA_SIZE 10
/// number of bytes that should be read only, starting from byte 0
#define I2CDATA_RO_BYTES 2

volatile uint8_t i2cdata[I2CDATA_SIZE];

volatile uint8_t buffer_adr;

volatile bool address_set = false; //< first write (false): register address, second write (true): data

}

void force_set_i2cdata(uint8_t index, uint8_t data) {
    if(index < I2CDATA_SIZE) {
        i2cdata[index] = data;
    }
}

void try_set_i2cdata(uint8_t index, uint8_t data) {
    if(index >= I2CDATA_RO_BYTES) {
        force_set_i2cdata(index, data);
    }
}

uint8_t get_i2cdata(uint8_t index) {
    if(index < I2CDATA_SIZE) {
        return i2cdata[index];
    } else {
        // nope
        return 0xFE;
    }

}

void receive_callback(int bytes) {
    if(bytes == 1) {
        // one byte, must be an address
        buffer_adr = Wire.read();
        // set to true, strictly speaking this flag is probably not necessary, but I feel better with it
        address_set = true;
    } else {
        // more than one byte, must be a write transaction
        // first byte is (start) address
        buffer_adr = Wire.read();
        // as long as there is data, write current byte and increment index
        for(int i = 0; i < bytes; i++) {
            // TODO: multiple reads are untested
            try_set_i2cdata(buffer_adr++, Wire.read());
        }
        address_set = false;
    }
}

void request_callback() {
    // either write was called before, or we'll just take the last address
    // anyway, here comes the data
    Wire.write(get_i2cdata(buffer_adr++));
    address_set = false;
}

void init_i2c_slave(uint8_t adr)
{
    Wire.begin(adr);
    Wire.onReceive(&receive_callback);
    Wire.onRequest(&request_callback);

    // I don't care about memset and performance here, we're in setup() after all
    for(uint8_t i=0; i<I2CDATA_SIZE; i++)
    {
        force_set_i2cdata(i, 0);
    }
}
