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

K_THREAD_STACK_DEFINE(transceive_stack, THREAD_S_SIZE)
static struct k_thread transceive_thread;
K_MSGQ_DEFINE(spi_queue, MAX_PACKET_SIZE, 128, 4);

K_THREAD_STACK_DEFINE(cmd_handler_stack, THREAD_S_SIZE)
static struct k_thread cmd_handler_thread;
K_MSGQ_DEFINE(cmd_queue, sizeof(void*), 128, 4);

static void init_buffer(struct spi_buf_set* buf_set, struct spi_buf* buf, size_t len, void* data) {
	buf->buf = data;
	buf->len = len;
	buf_set->buffers = buf;
	buf_set->count = 1;
	return;
}

// return the crc of the packet
static int preform_crc8(struct packet* p) {
  return 0; // for now the packet is always correct
}

static void slave_packet_create(struct packet* out, uint8_t seq, uint16_t size, uint8_t* data) {
  out->seqnum = seq;
  out->initiator = 1;
  out->size = size;
  memcpy(out->data, data, size);
  out->crc = preform_crc8(out);
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

static uint8_t* get_reg_data(struct spi2c_com_data* data, uint8_t reg) {
	switch(reg) {
		case SPI2C_DSTAT_REG:
			return &data->d_stat;
		default:
			return NULL;
	}
}

// incoming packet fmt of i2c cmd write --> |packet header|CMD:0x00 (1 byte)|i2c adr (1 byte)| write data (header.size - 2 bytes)| 
// outgoing packet fmt of i2c cmd write --> |packet header|ERR CODE (1 byte)|
void spi2c_i2c_write(const struct device* dev, struct packet* in, struct packet* out) {
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;

  uint8_t code = SPI2C_SUCCESS;
  uint8_t i2c_adr = *(uint8_t*)(in->data + 1);
	const struct i2c_dt_spec* i2c_dev = match_i2c_dt_adr(cfg, i2c_adr);
	if (i2c_dev == NULL) { 
    code = SPI2C_NO_BUS;
    goto create_err_packet;
  }
  uint8_t* w_data = (uint8_t*)(in->data + 2); // beginning of data to write
  uint8_t w_data_size = in->size - 2;
	if (i2c_write_dt(i2c_dev, w_data, w_data_size)) { 
    code = SPI2C_I2C_RWERR;
  }
  create_err_packet:
  slave_packet_create(out, in->seqnum, 1, &code);
  return;   
}

// incoming packet fmt of i2c cmd read --> |packet header|CMD:0x01 (1 byte)|i2c adr (1 byte)|read size (1 byte)| 
// outgoing packet fmt of i2c cmd read --> |packet header|ERR CODE (1 byte)|read data (header.size - 1 bytes)|
void spi2c_i2c_read(const struct device* dev, struct packet* in, struct packet* out) {
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;

  uint8_t code = SPI2C_SUCCESS;
  uint8_t size = *(uint8_t*)(in->data + 3);
  if (size != 3) {
    code = SPI2C_INMEM;
    goto create_err_packet;
  }
  uint8_t i2c_adr = *(uint8_t*)(in->data + 1);
	const struct i2c_dt_spec* i2c_dev = match_i2c_dt_adr(cfg, i2c_adr);
	if (i2c_dev == NULL) { 
    code = SPI2C_NO_BUS;
    goto create_err_packet;
  }

	uint8_t read_buf[size + 1];
	if (i2c_read_dt(i2c_dev, (uint8_t*)(read_buf + 1), size)) { 
    code = SPI2C_I2C_RWERR;
    goto create_err_packet;
  }
  read_buf[0] = code;
  slave_packet_create(out, in->seqnum, size + 1, read_buf);
  return;

  create_err_packet:
  slave_packet_create(out, in->seqnum, 1, &code);
  return;   
}

// incoming packet fmt of reg cmd read --> |packet header|CMD:0x02 (1 byte)|reg adr (1 byte)|read size (1 byte)| 
// outgoing packet fmt of reg cmd read --> |packet header|ERR CODE (1 byte)|read data (header.size - 1 bytes)|
void spi2c_read_reg(const struct device* dev, struct packet* in, struct packet* out) {
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;
  uint8_t reg_adr = *(uint8_t*)(in->data + 2);  
  uint8_t read_size = *(uint8_t*)(in->data + 3);

  uint8_t code = SPI2C_SUCCESS;
  uint8_t* reg = get_reg_data(data, reg_adr); 
  if (reg == NULL) {
    code = SPI2C_NO_REG;
    goto create_err_packet;
  }
  if (sizeof(*reg) != read_size) {
    code = SPI2C_INMEM;
    goto create_err_packet;
  }
  uint8_t read_data[read_size + 1];
  read_data[0] = code;
  memcpy((uint8_t*)(read_data + 1), reg, read_size);
  slave_packet_create(out, in->seqnum, read_size + 1, read_data);
  return;

  create_err_packet:
  slave_packet_create(out, in->seqnum, 1, &code); 
}

// incoming packet fmt of reg cmd write --> |packet header|CMD:0x03 (1 byte)|reg adr (1 byte)| write data (header.size - 2 bytes)| 
// outgoing packet fmt of reg cmd write --> |packet header|ERR CODE (1 byte)|
int spi2c_write_reg(const struct device* dev, struct packet* in, struct packet* out) {
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;

  uint8_t code = SPI2C_SUCCESS;
  uint8_t reg_adr = *(uint8_t*)(in->data + 1);
  uint8_t* reg = get_reg_data(data, reg_adr);
	if (reg == NULL) { 
    code = SPI2C_NO_REG;
    goto create_err_packet;
  }
  uint8_t* w_data = (uint8_t*)(in->data + 2); // beginning of data to write
  uint8_t w_data_size = in->size - 2;
  if (w_data_size != sizeof(*reg)) {
    code = SPI2C_INMEM;
    goto create_err_packet;
  }
  memcpy(reg, w_data, w_data_size);

  create_err_packet:
  slave_packet_create(out, in->seqnum, 1, &code);
  return; 
}

void spi2c_cmd_handler_thread(void* p1, void* p2, void* p3) {
  struct device* dev = (struct device*)p1;
  struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
  struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;

  for (;;) {
    struct packet rx;
    k_msgq_get(&cmd_queue, &rx, K_FOREVER);
    uint8_t tx_buffer[MAX_PACKET_SIZE] = {0};
    struct packet* tx = (struct packet*)tx_buffer
    if (preform_crc8(rx) != rx->crc8) {
      uint8_t code = SPI2C_INVAL_CRC;
      slave_packet_create(tx, tx->seqnum, 1, &code);
      goto finish;
    }
    uint8_t cmd = rx->data[0];
    switch(cmd) {
      case SPI2C_I2C_WRITE:
        spi2c_i2c_write(dev, rx, tx);
        break;
      case SPI2C_I2C_READ:
        spi2c_i2c_read(dev, rx, tx);
        break;
      case SPI2C_REG_WRITE:
        spi2c_write_reg(dev, rx, tx);
        break;
      case SPI2C_REG_READ:
        spi2c_read_reg(dev, rx, tx);
        break;
      default:
        uint8_t code = SPI2C_INVAL_CMD;
        slave_packet_create(tx, tx->seqnum, 1, &code);
        break;
    }

    finish:
    k_free(rx);
    k_msgq_put(&spi_queue, tx, K_FOREVER);
  }
}

void spi2c_cmd_cb(const struct device* dev, int result, void* data) {
  struct k_sem* tranfer_fin  = (struct k_sem*)data;
  k_sem_take(transfer_fin, K_FOREVER);
}

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
  static uint8_t tx_dummy[MAX_PACKET_SIZE] = {0};
  for (;;) {
    struct packet tx;
    struct packet rx;
    uint8_t tx_buffer[MAX_PACKET_SIZE] = {0};
    uint8_t rx_buffer[MAX_PACKET_SIZE] = {0};
    struct packet* tx = (struct packet*)tx_buffer;
    struct packet* rx = (struct packet*)rx_buffer;
    if (k_msgq_get(&spi_queue, tx, K_NO_WAIT)) {
      // there is a msg to send
      // pull gpio line
      gpio_pin_set_dt(&cfg->signal_gpio, 1); // set high 
      init_buffer(rx_buf_set, rx_buf, sizeof(struct packet) + tx->size, (void*)rx);
      init_buffer(tx_buf_set, rx_buf, sizeof(struct packet) + tx->size, (void*)tx);
      spi_transceive_cb(cfg->spi_dev->bus, &cfg->spi_dev->config, tx_buf_set, rx_buf_set, spi2c_cmd_cb, (void*)&transfer_fin);
      k_sem_take(&transfer_fin, K_FOREVER);
      gpio_pin_set_dt(&cfg->signal_gpio, 0); // set low
      k_free(tx);
      continue;
    } 
    init_buffer(rx_buf_set, rx_buf, MAX_PACKET_SIZE, (void*)rx);
    init_buffer(tx_buf_set, rx_buf, MAX_PACKET_SIZE, (void*)tx_dummy);
    spi_transceive_cb(cfg->spi_dev->bus, &cfg->spi_dev->config, tx_buf_set, rx_buf_set, spi2c_cmd_cb, (void*)&transfer_fin);
    k_sem_take(&transfer_fin, K_FOREVER);
    // waiting could be an issue, check in dbg
    k_msgq_put(&cmd_queue, rx, K_FOREVER);
  }
}

static uint8_t spi2c_init_devices(const struct device* dev) {
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
  if (!gpio_is_ready_dt(&cfg->signal_gpio)) {
    data->d_stat = SPI2C_UNINIT;
    return SPI2C_UNINIT;
  }
  if (gpio_pin_configure_dt(&cfg->signal_gpio, GPIO_OUTPUT_INACTIVE)) {
    data->d_stat = SPI2C_UNINIT;
    return SPI2C_UNINIT;
  }
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
	data->d_stat = SPI2C_INIT;
	return SPI2C_INIT;
}

static uint8_t spi2c_begin_impl(const struct device* dev) {
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
  if (spi2c_init_devices(dev) != SPI2C_INIT) {
    return SPI2C_UNINIT;
  }
	if (data->d_stat == SPI2C_INIT) {
		return SPI2C_INIT; // thread already started
	}
  k_thread_create(&transceive_thread,
                  transceive_stack,
                  K_THREAD_STACK_SIZEOF(transceive_stack),
                  spi2c_transceive_thread,
                  dev, NULL, NULL,
                  0, 0, K_NO_WAIT);
  k_thread_create(&cmd_handler_thread,
                  cmd_handler_stack,
                  K_THREAD_STACK_SIZEOF(cmd_handler_stack),
                  spi2c_cmd_handler_thread,
                  dev, NULL, NULL,
                  0, 0, K_NO_WAIT);
	return SPI2C_INIT;
}

static const struct spi2c_driver spi2c_driver_impl = {
	.spi2c_begin = spi2c_begin_impl,
};

#define SPI2C_I2C_ELEM_TO_SPEC(node_id, prop, idx) \
	[idx] = I2C_DT_SPEC_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)),

#define SPI2C_DEFINE(inst)                                                \
	static struct spi2c_com_data spi2c_com_data_##inst = {                  \
		.d_stat = SPI2C_UNINIT,                                               \
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
    .signal_gpio = GPIO_DT_SPEC_INST_GET(inst, signal_gpios),             \
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
