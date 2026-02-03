#define DT_DRV_COMPAT hn_spi2c

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <spi2c-com.h>

#define SPI_OP (SPI_OP_MODE_SLAVE | SPI_MODE_CPOL | SPI_MODE_CPHA | \
		SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_LINES_SINGLE)

#define CMD_THREAD_SIZE 1024
#define CMD_AMT 4

// commands correspond to indexes in array
typedef int (*cmd_func)(const struct device*, uint8_t*);
static const cmd_func cmd_funcs[] = { spi2c_write, spi2c_read, spi2c_read_rx, spi2c_read_reg };

static void init_buffer(struct spi_buf_set* buf_set, struct spi_buf* buf, size_t len, void* data) {
	buf->buf = data;
	buf->len = len;
	buf_set->buffers = buf;
	buf_set->count = 1;
	return;
}

// tries to find a i2c device that corresponds with the given i2c address
static const struct i2c_dt_spec* match_i2c_dt_adr(struct spi2c_com_cfg* cfg, uint8_t addr) {

	for (uint8_t i = 0; i < cfg->i2c_dev_num; i++) {
		const struct i2c_dt_spec* i2c_dev = &cfg->i2c_devs[i];
		if (i2c_dev->addr == addr) {
			return i2c_dev;
		}
	}
	return NULL;
}

// CMD: 0x00
int spi2c_write(const struct device* dev, uint8_t cmd_data[3]) {
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;
	if (cmd_data[0] != SPI2C_CMD_WRITE) { return SPI2C_SPI_RWERR; }
	size_t size = (size_t)cmd_data[1];
	uint8_t i2c_adr = cmd_data[2];

	uint8_t data_buf_rx[size];
	struct spi_buf_set buf_set_rx;
	struct spi_buf buf_rx;
	init_buffer(&buf_set_rx, &buf_rx, size, data_buf_rx);
	uint8_t data_buf_tx[size];
	struct spi_buf_set buf_set_tx;
	struct spi_buf buf_tx;
	init_buffer(&buf_set_tx, &buf_tx, size, data_buf_tx);

	spi_transceive_dt(cfg->spi_dev, &buf_set_tx, &buf_set_rx);
	const struct i2c_dt_spec* i2c_dev = match_i2c_dt_adr(cfg, i2c_adr);
	if (i2c_dev == NULL) { return SPI2C_NO_BUS; }
	if (i2c_write_dt(i2c_dev, data_buf_rx, size)) { return SPI2C_I2C_RWERR; }
	return SPI2C_SUCCESS;
}

// CMD: 0x01
int spi2c_read(const struct device* dev, uint8_t cmd_data[3]) {
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;
	if (cmd_data[0] != SPI2C_CMD_READ) { return SPI2C_SPI_RWERR; }
	size_t size = (size_t)cmd_data[1];
	uint8_t i2c_adr = cmd_data[2];

	const struct i2c_dt_spec* i2c_dev = match_i2c_dt_adr(cfg, i2c_adr);
	if (i2c_dev == NULL) { return SPI2C_NO_BUS; }
	uint8_t read_buf[size];
	if (i2c_read_dt(i2c_dev, read_buf, size)) { return SPI2C_I2C_RWERR; }
	memcpy(data->rx_reg, read_buf, size);
	return SPI2C_SUCCESS;
}

// CMD: 0x02
int spi2c_read_rx(const struct device* dev, uint8_t cmd_data[3]) {
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;
	if (cmd_data[0] != SPI2C_CMD_READ_RX) { return SPI2C_SPI_RWERR; }
	size_t size = (size_t)cmd_data[1];

	uint8_t data_buf_rx[size];
	struct spi_buf_set buf_set_rx;
	struct spi_buf buf_rx;
	init_buffer(&buf_set_rx, &buf_rx, size, data_buf_rx);
	uint8_t data_buf_tx[size];
	memcpy(data_buf_tx, dev->rx_reg, size);
	struct spi_buf_set buf_set_tx;
	struct spi_buf buf_tx;
	init_buffer(&buf_set_tx, &buf_tx, size, data_buf_tx);

	if (spi_transceive_dt(&cfg->spi_dev, &buf_set_tx, &buf_set_rx)) {
		return SPI2C_SPI_RWERR;
	}
	return SPI2C_SUCCESS;
}

static uint8_t get_reg_data(struct spi2c_com_data* data, uint8_t reg) {
	switch(reg) {
		case SPI2C_DSTAT_REG:
			return data->d_stat;
		case SPI2C_CSTAT_REG:
			return data->c_stat;
		default:
			return 0;
	}
}

// CMD: 0x03
int spi2c_read_reg(const struct device* dev, uint8_t cmd_data[3]) {
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;
	if (cmd_data[0] != SPI2C_CMD_READ_CMD_STATUS) { return SPI2C_SPI_RWERR; }
	uint8_t reg = cmd_data[1];

	uint8_t data_buf_tx = get_reg_data(data, reg);
	struct spi_buf_set buf_set_tx;
	struct spi_buf buf_tx;
	init_buffer(&buf_set_tx, &buf_tx, size, &data_buf_tx);
	uint8_t data_buf_rx = 0;
	struct spi_buf_set buf_set_rx;
	struct spi_buf buf_rx;
	init_buffer(&buf_set_rx, &buf_rx, 1, &data_buf_rx);
	if (spi_transceive_dt(&cfg->spi_dev, &buf_set_tx, &buf_set_rx)) {
		return SPI2C_SPI_RWERR;
	}
	return SPI2C_SUCCESS;
}

// main running thread, recieves commands and dispatches them to their corresponding function
// handler
void spi2c_cmd_thread(void* p1, void* p2, void* p3) {
	const struct device* dev = (const struct device*)p1;
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;
	uint8_t cmd_buf_tx[3] = {0};
	struct spi_buf_set buf_set_tx;
	struct spi_buf buf_tx;
	init_buffer(buf_set_tx, buf_tx, sizeof(cmd_buf_tx), &cmd_buf_tx);

	for (;;) {
		uint8_t cmd_buf_rx[3] = {0};
		struct spi_buf_set buf_set_rx;
		struct spi_buf buf_rx;
		init_buffer(buf_set_rx, buf_rx, sizeof(cmd_buf_rx), &cmd_buf_rx);

		if (spi_transceive_dt(cfg->spi_dev, &buf_set_tx, &buf_set_rx) <= 0) {
			data->c_stat = SPI2C_SPI_RWERR;
			continue;
		}
		uint8_t cmd_num = cmd_buf_rx[0];
		if (cmd_num < 0 || cmd_num > CMD_AMT - 1) {
			data->c_stat = SPI2C_INVAL_CMD;
			continue;
		}

		data->c_stat = cmd_funcs[cmd_num](dev, &cmd_buf_rx);
	}
}

K_THREAD_STACK_DEFINE(cmd_stack, 1024)
static struct k_thread cmd_thread;

static int spi2c_begin_impl(const struct device* dev) {
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;
	struct spi2c_comd_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
	for (int i = 0; i < cfg->i2c_dev_num; i++) {
		if (!i2c_is_ready_dt(&cfg->i2c_devs[i])) {
			data->d_stat = SPI2C_I2C_ERR;
			return SPI2C_I2C_ERR;
		}
	}
	if (!spi_is_ready_dt(&cfg->spi_dev)) {
		data->d_stat = SPI2C_SPI_ERR;
		return SPI2C_SPI_ERR;
	}
	if (data->d_stat == SPI2C_INIT) {
		return SPI2C_INIT; // thread already started
	}
	struct k_thread cmd_thread;
	k_thread_create(&cmd_thread,
			cmd_stack,
			K_THREAD_STACK_SIZEOF(cmd_stack),
			spi2c_cmd_thread,
			dev,
			NULL,
			NULL,
			0, 0, K_NO_WAIT);
	data->d_stat = SPI2C_INIT;
	return 0;
}

static const struct spi2c_driver spi2c_driver_impl = {
	.spi2c_begin = spi2c_begin_impl,
};

#define SPI2C_I2C_ELEM_TO_SPEC(inst, prop, idx) \
	[idx] = I2C_DT_SPEC_GET(DT_INST_PROP_BY_IDX(inst, prop, idx)),

#define SPI2C_DEFINE(inst)                                                \
	static struct spi2c_com_data spi2c_com_data_##inst = {                  \
		.d_stat = SPI2C_UNINIT,                                               \
		.c_stat = SPI2C_SUCCESS,                                              \
		.rx_reg = {0},                                                        \
	};                                                                      \
                                                                          \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, i2c_devs) <= MAX_I2C_DEVS,          \
		     "i2c-devs has more than MAX_I2C_DEVS entries");                  \
                                                                          \
	static const struct spi2c_com_cfg spi2c_com_cfg_##inst = {              \
		.i2c_dev_num = (uint8_t)DT_INST_PROP_LEN(inst, i2c_devs),             \
		.i2c_devs = {                                                         \
			DT_INST_FOREACH_PROP_ELEM(inst, i2c_devs,                           \
						  SPI2C_I2C_ELEM_TO_SPEC)                                     \
		},                                                                    \
		.spi_dev = SPI_DT_SPEC_GET(DT_INST_PHANDLE(inst, spi_dev), SPI_OP),   \
	};                                                                      \
                                                                          \
	DEVICE_DT_INST_DEFINE(inst,                                             \
			      NULL, NULL,                                                   \
			      &spi2c_com_data_##inst,                                       \
			      &spi2c_com_cfg_##inst,                                        \
			      POST_KERNEL,                                                  \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                           \
			      &spi2c_driver_impl);                                          \

DT_INST_FOREACH_STATUS_OKAY(SPI2C_DEFINE)
