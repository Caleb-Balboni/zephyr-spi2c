#ifndef SPI2C_COM_H
#define SPI2C_COM_H

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zerphyr/device.h>

#define MAX_I2C_DEVS 8

// register values for CMD register (uint8_t cmd_stat)
typedef enum {
	SPI2C_SUCCESS,		// a command was successfully executed
	SPI2C_NO_BUS,		// the i2c reg being read or read to does not exist
	SPI2C_INMEM,		// the amount of data being read or written was invalid
	SPI2C_SPI_RWERR,	// a read/write error occured in the spi device
	SPI2C_I2C_RWERR,	// a read/write error occured in the i2c device
	SPI2C_INVAL_CMD,	// a cmd recieved by the device was invalid
} spi2c_cmd_state;

// register values for driver register (uint8_t d_stat)
typedef enum {
	SPI2C_UNINIT,		// the driver is not initalized
	SPI2C_INIT,		// the driver is initalized
	SPI2C_I2C_ERR,		// one or more i2c devices failed to init
	SPI2C_SPI_ERR,		// the spi device failed to init
} spi2c_driver_state;

typedef enum {
	SPI2C_DSTAT_REG		= 0x50,
	SPI2C_CSTAT_REG		= 0x60,
} spi2c_reg;

typedef enum {
	SPI2C_CMD_WRITE		= 0x00,
	SPI2C_CMD_READ		= 0x01,
	SPI2C_CMD_READ_RX	= 0x02,
	SPI2C_CMD_READ_REG	= 0x03,
} spi2c_cmd;

struct spi2c_com_cfg {
	uint8_t i2c_dev_num; // the number of i2c devices
	const i2c_dt_spec i2c_devs[MAX_I2C_DEVS]; // i2c devices
	const spi_dt_spec spi_dev; // spi
};

struct spi2c_com_data {
	uint8_t d_stat; // status of the driver
	uint8_t c_stat; // status of the cmd
	uint8_t rx_reg[256]; // read data reg
};

// All possible command function handlers
// NOTE: these functions are only to be used by the spi slave, they are only induced by the master
// and recieved by these functions

// handles the 'write' cmd, which writes recieved bytes from the master
// to one of the slaves i2c devices
// @param dev - the slave spi + i2c device interface
// @param cmd_data - three bytes of info: |CMD|NUM OF BYTES|i2c ADR| (each one byte)
// @return - SPI2C_SUCCESS if execution was a success, else an error code (above)
int spi2c_write(const struct device* dev, uint8_t cmd_data[3]);

// handles the 'read' cmd, which reads information from the i2c device into the rx_reg
// @param dev - the slave spi + i2c device interface
// @param cmd_data - three bytes of info: |CMD|NUM OF BYTES|i2c ADR| (each one byte)
// @return - SPI2C_SUCCESS if execution was a success, else an error code (above)
int spi2c_read(const struct device* dev, uint8_t cmd_data[3]);

// handles the 'read rx' cmd, which reads the data bytes from the rx register
// @param dev - the slave spi + i2c device interface
// @param cmd_data - three bytes of info: |CMD|NUM OF BYTES|DUMMY| (each one byte)
// @return - SPI2C_SUCCESS if execution was a success, else an error code (above)
int spi2c_read_rx(const struct device* dev, uint8_t cmd_data[3]);

// handles the 'read reg' cmd, which reads from a particular register (driver or cmd stat)
// @param dev - the slave spi + i2c device interface
// @param cmd_data - three bytes of info: |CMD|register|DUMMY| (each one byte)
// @return - SPI2C_SUCCESS if execution was a success, else an error code (above)
int spi2c_read_reg(const struct device* dev, uint8_t cmd_data[3]);

typedef int (*spi2c_begin_t)(const struct device* dev);

struct spi2c_driver {
	spi2c_begin_t spi2c_begin;
};

#endif
