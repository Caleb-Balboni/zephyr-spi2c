#define DT_DRV_COMPAT hn_spi2c

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdbool.h>
#include <spi2c-com.h>

#define SPI_OP (SPI_OP_MODE_SLAVE | SPI_MODE_CPOL | SPI_MODE_CPHA | \
		SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_LINES_SINGLE)

#define THREAD_S_SIZE 1024

static void init_buffer(struct spi_buf_set* buf_set, struct spi_buf* buf, size_t len, void* data) {
	buf->buf = data;
	buf->len = len;
	buf_set->buffers = buf;
	buf_set->count = 1;
	return;
}

static int test_crc(struct packet* p) {
  return 0; // for now the packet is always correct
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

int spi2c_i2c_write(const struct device* dev, struct packet* in) {
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;
	if (int->cmd != SPI2C_I2C_WRITE) { return -1; }
  
  uint8_t i2c_adr = in->data[0];
	const struct i2c_dt_spec* i2c_dev = match_i2c_dt_adr(cfg, i2c_adr);
	if (i2c_dev == NULL) { return SPI2C_NO_BUS; }
	if (i2c_write_dt(i2c_dev, (uint8_t*)(data + 1), in->size - 1)) { return -1; }
	return 0;
}

int spi2c_i2c_load_helper(const struct device* dev, uint8_t i2c_adr, size_t data_size) {
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

// CMD: 0x00
int spi2c_load(const struct device* dev, uint8_t* data) {
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;
	if (cmd_data[0] != SPI2C_CMD_READ_RX) { return SPI2C_SPI_RWERR; }
	size_t size = (size_t)cmd_data[1];

	uint8_t data_buf_rx[size];
	uint8_t data_buf_tx[size];
	memcpy(data_buf_tx, data->rx_reg, size);

	if (spi2c_transcieve_helper(&cfg->spi_dev, data_buf_rx, size, data_buf_tx, size)) {
		return SPI2C_SPI_RWERR;
	}
	return SPI2C_SUCCESS;
}

// CMD: 0x01
int spi2c_write(const struct device* dev, uint8_t* data) {
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;
}

K_THREAD_STACK_DEFINE(cmd_handler_stack, THREAD_S_SIZE)
static struct k_thread cmd_handler_thread;
K_MSGQ_DEFINE(cmd_queue, sizeof(packet) + 64, 32, 4);

void spi2c_cmd_handler_thread(void* p1, void* p2, void* p3) {
  struct device* dev = (struct device*)p1;
  struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
  struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;

  for (;;) {
    struct packet* rx;
    if (k_msgq_get(&cmd_queue, rx, K_FOREVER)) {
      // should never get here
      continue;
    }
    test_crc(rx);
    struct packet tx;  
    int code = 0; 
    switch(rx->cmd) {
      case SPI2C_I2C_WRITE:
        code = spi2c_i2c_write(dev, rx);
        break;
      case SPI2C_I2C_READ:
        code = spi2c_i2c_read(dev, rx, &tx);
        break;
    }
    k_free(rx);
    // this needs to be better fleshed out with an actual error system
    if (code) { continue; }

    struct packet* tx_queue = k_malloc(sizeof(struct packet) + tx.size);
    k_msgq_put(&spi_queue, tx_queue, K_FOREVER);
  }
}

void spi2c_cmd_cb(const struct device* dev, int result, void* data) {
  struct k_sem* tranfer_fin  = (struct k_sem*)data;
  k_sem_take(transfer_fin, K_FOREVER);
}

K_THREAD_STACK_DEFINE(transceive_stack, THREAD_S_SIZE)
static struct k_thread transceive_thread;
K_MSGQ_DEFINE(spi_queue, sizeof(packet) + 64, 4);

void spi2c_transceive_thread(void* p1, void* p2, void* p3) {
  struct device* dev = (struct device*)p1;
  struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
  struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;

  struct spi_buf_set tx_buf_set;
  struct spi_buf_set rx_buf_set;
  struct spi_buf tx_buf;
  struct spi_buf rx_buf;
  struct k_sem transfer_fin;
  k_sem_init(&tranfer_fin, 0, 1);
  for (;;) {
    struct packet tx;
    struct packet rx;
    if (k_msgq_get(&spi_queue, &tx, K_NO_WAIT)) {
      // there is a msg to send
      // pull gpio line
       
    } 
    
    // start async transfer 
    spi_transceive_cb(cfg->spi_dev->bus, &cfg->spi_dev->config, )

    k_sem_take(&transfer_fin, K_FOREVER);
    size_t msg_size = sizeof(struct packet) + rx.size;
    struct packet* queue_rx = k_malloc(msg_size);
    memcpy(queue_rx, &rx, msg_size);
    // waiting could be an issue, check in dbg
    k_msgq_put(&cmd_queue, queue_rx, K_FOREVER);
  }
}

static int spi2c_begin_impl(const struct device* dev) {
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
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
	data->d_stat = SPI2C_INIT;
	return 0;
}

static const struct spi2c_driver spi2c_driver_impl = {
	.spi2c_begin = spi2c_begin_impl,
};

#define SPI2C_I2C_ELEM_TO_SPEC(node_id, prop, idx) \
	[idx] = I2C_DT_SPEC_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)),

#define SPI2C_DEFINE(inst)                                                \
	static struct spi2c_com_data spi2c_com_data_##inst = {                  \
    .tx_ready = ATOMIC_INIT(1),                                           \
		.d_stat = SPI2C_UNINIT,                                               \
		.c_stat = SPI2C_SUCCESS,                                              \
		.rx_reg = {0},                                                        \
	};                                                                      \
                                                                          \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, i2c_devs) <= MAX_I2C_DEVS,          \
		     "i2c-devs has more than MAX_I2C_DEVS entries");                  \
                                                                          \
	static const struct spi2c_com_cfg spi2c_com_cfg_##inst = {              \
		.i2c_dev_num = (uint8_t)DT_PROP_LEN(DT_DRV_INST(inst), i2c_devs),     \
		.i2c_devs = {                                                         \
			DT_FOREACH_PROP_ELEM(DT_DRV_INST(inst), i2c_devs,                   \
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
