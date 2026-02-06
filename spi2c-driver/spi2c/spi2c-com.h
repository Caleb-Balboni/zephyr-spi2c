#ifndef SPI2C_COM_H
#define SPI2C_COM_H

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#define MAX_I2C_DEVS 8
#define MAX_PACKET_SIZE 255

// register values for driver register (uint8_t d_stat)
typedef enum {
	SPI2C_UNINIT,		  // the driver is not initalized
	SPI2C_INIT,		    // the driver is initalized
	SPI2C_I2C_ERR,		// one or more i2c devices failed to init
	SPI2C_SPI_ERR,		// the spi device failed to init
} spi2c_driver_state;

typedef enum {
	SPI2C_SUCCESS,		// a command was successfully executed
	SPI2C_NO_BUS,		  // the i2c reg being read or read to does not exist
  SPI2C_NO_REG,     // the reg being read or written to does not exist
	SPI2C_INMEM,		  // the size of the data packet was incorrect for the cmd
	SPI2C_SPI_RWERR,	// a read/write error occured in the spi device
  SPI2C_INVAL_CRC,  // the crc for the command to the slave was invalid
	SPI2C_I2C_RWERR,	// a read/write error occured in the i2c device
	SPI2C_INVAL_CMD,	// a cmd recieved by the device was invalid
} spi2c_cmd_state;

typedef enum {
	SPI2C_DSTAT_REG		  = 0xAA,
} spi2c_reg_adr;

typedef enum {
	SPI2C_I2C_WRITE     = 0x00,
	SPI2C_I2C_READ		  = 0x01,
  SPI2C_REG_READ      = 0x02,
  SPI2C_REG_WRITE     = 0x03,
} spi2c_cmd;

struct spi2c_com_cfg {
	uint8_t i2c_dev_num; // the number of i2c devices
  const struct gpio_dt_spec signal_gpio;
	const struct i2c_dt_spec i2c_devs[MAX_I2C_DEVS]; // i2c devices
	const struct spi_dt_spec spi_dev; // spi
};

// packet of size + 4 bytes
struct packet
{
    uint8_t crc8:8;              // bits 0:7, CRC8 of the rest of the packet
    uint8_t initiator:1;        // bit 8, 0 if transfer was init by master, 1 if init by slave
    uint8_t seqnum:7;           // bits 9:15, 7-bit sequence number according to initiator seqnum counter
    uint16_t size:16;           // bits 16:31, 16-bit data size field, the total size of the data array
    uint8_t data[];             // arbitrary size data field according to size
}__attribute__((packed));

struct spi2c_com_data {
	uint8_t d_stat; // status of the driver
};

// All possible command function handlers
// NOTE: these functions are only to be used by the spi slave, they are only induced by the master
// and recieved by these functions

// handles the i2c write cmd, which writes data from the master to the specified i2c device
// incoming packet fmt of i2c cmd write --> |packet header|CMD:0x00 (1 byte)|i2c adr (1 byte)| write data (header.size - 2 bytes)| 
// outgoing packet fmt of i2c cmd write --> |packet header|ERR CODE (1 byte)|
// @param dev - the slave spi + i2c device interface
// @param in - the input packet containing the data to write
// @param out - data returned to master by write (in this case just the status of the cmd)
void spi2c_i2c_write(const struct device* dev, struct packet* in, struct packet* out);

// handles the i2c read cmd, gets data from the i2c device to be passed back to the master
// incoming packet fmt of i2c cmd read --> |packet header|CMD:0x01 (1 byte)|i2c adr (1 byte)|read size (1 byte)| 
// outgoing packet fmt of i2c cmd read --> |packet header|ERR CODE (1 byte)|read data (header.size - 1 bytes)|
// @param dev - the slave spi + i2c device interface
// @param in - the input packet defining the kind and area of which to grab data
// @param out - the status of the command followed by the read data
void spi2c_i2c_read(const struct device* dev, struct packet* in, struct packet* out);

// handles the read reg cmd, which reads the data from a specified register back to the master
// incoming packet fmt of reg cmd read --> |packet header|CMD:0x02 (1 byte)|reg adr (1 byte)|read size (1 byte)| 
// outgoing packet fmt of reg cmd read --> |packet header|ERR CODE (1 byte)|read data (header.size - 1 bytes)|
// @param dev - the slave spi + i2c device interface
// @param in - the input packet defining what register to read from and how much data
// @param out - the status of the command followed by the output packet to give back to the master
void spi2c_read_reg(const struct device* dev, struct packet* in, struct packet* out);

// handles the write register cmd, which writes data from the master to a register
// incoming packet fmt of reg cmd write --> |packet header|CMD:0x03 (1 byte)|reg adr (1 byte)| write data (header.size - 2 bytes)| 
// outgoing packet fmt of reg cmd write --> |packet header|ERR CODE (1 byte)|
// @param dev - the slave spi + i2c device interface
// @param in - the input packet containing the information about which register and the data to write
// @param out - the status of the command followed by the read data from the register
void spi2c_write_reg(const struct device* dev, struct packet* in, struct packet* out);

typedef uint8_t (*spi2c_begin_t)(const struct device* dev);

struct spi2c_driver {
	spi2c_begin_t spi2c_begin;
};

// begins worker threads that wait for incoming master commands to read and write data
// from registers and i2c devices
// @param dev - the slave spi + i2c device interface
// @return - 0 on success else a standard zephyr error code
static inline int spi2c_begin_com(const struct device* dev) {
  struct spi2c_driver* api = (struct spi2c_driver*)dev->api;
  return api->spi2c_begin(dev);
}

#endif
