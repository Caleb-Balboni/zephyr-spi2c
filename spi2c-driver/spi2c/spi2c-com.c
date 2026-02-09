#define DT_DRV_COMPAT hn_spi2c

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdbool.h>
#include <spi2c-com.h>
#include <custom-dev.h>

#define SPI_OP (SPI_OP_MODE_SLAVE | SPI_MODE_CPOL | SPI_MODE_CPHA | \
		SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_LINES_SINGLE)

#define THREAD_S_SIZE 1024

K_THREAD_STACK_DEFINE(transceive_stack, THREAD_S_SIZE)
static struct k_thread transceive_thread;
K_MSGQ_DEFINE(spi_queue, MAX_PACKET_SIZE, 8, 4);

K_THREAD_STACK_DEFINE(cmd_handler_stack, THREAD_S_SIZE)
static struct k_thread cmd_handler_thread;
K_MSGQ_DEFINE(cmd_queue, MAX_PACKET_SIZE, 8, 4);

static void init_buffer(struct spi_buf_set* buf_set, struct spi_buf* buf, size_t len, void* data) {
	buf->buf = data;
	buf->len = len;
	buf_set->buffers = buf;
	buf_set->count = 1;
	return;
}

// return the crc of the packet
static uint8_t preform_crc8(struct packet* p) {
  // TODO
  return 0;
}

static void slave_packet_create(struct packet* out, uint8_t seq, uint16_t size, uint8_t* data) {
  out->seqnum = seq;
  out->initiator = 1;
  out->size = size;
  memcpy(out->data, data, size);
  out->crc8 = preform_crc8(out);
  return;
}

// tries to find a device that corresponds with the given address
static const struct device* match_dev_adr(struct spi2c_com_cfg* cfg, uint8_t addr) {

	for (uint8_t i = 0; i < cfg->dev_amt; i++) {
		const struct device* dev = cfg->custom_devs[i];
    uint8_t dev_adr = spi2c_get_custom_dev_adr(dev); 
		if (spi2c_get_custom_dev_adr(dev) == addr) {
			return dev;
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

// incoming packet fmt of dev cmd write --> |packet header|CMD:0x00 (1 byte)|i2c adr (1 byte)| write data (header.size - 2 bytes)| 
// outgoing packet fmt of dev cmd write --> |packet header|ERR CODE (1 byte)|
void spi2c_dev_write(const struct device* dev, struct packet* in, struct packet* out) {
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;

  uint8_t dev_adr = *(uint8_t*)(in->data + 1);
	const struct device* custom_dev = match_dev_adr(cfg, dev_adr);
  uint8_t code = SPI2C_SUCCESS;
  if (custom_dev == NULL) {
    code = SPI2C_NO_BUS;
    slave_packet_create(out, in->seqnum, 1, &code);
    return;
  } 
  code = spi2c_read_custom_dev(custom_dev, in, out);
  if (code != SPI2C_SUCCESS) {
    slave_packet_create(out, in->seqnum, 1, &code);
  }
  return;   
}

// incoming packet fmt of dev cmd read --> |packet header|CMD:0x01 (1 byte)|i2c adr (1 byte)|data (impl specific)| 
// outgoing packet fmt of dev cmd read --> |packet header|ERR CODE (1 byte)|data (impl specific)|
void spi2c_dev_read(const struct device* dev, struct packet* in, struct packet* out) {
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;

  uint8_t code = SPI2C_SUCCESS;
  uint8_t dev_adr = *(uint8_t*)(in->data + 1);
	const struct device* custom_dev = match_dev_adr(cfg, dev_adr);
	if (custom_dev == NULL) { 
    code = SPI2C_NO_BUS;
    slave_packet_create(out, in->seqnum, 1, &code);
    return;
  }
  // calling the user implemented api function
  code = spi2c_read_custom_dev(custom_dev, in, out);
  if (code != SPI2C_SUCCESS) {
    slave_packet_create(out, in->seqnum, 1, &code);
  }
  return;   
}

// incoming packet fmt of reg cmd read --> |packet header|CMD:0x02 (1 byte)|reg adr (1 byte)|read size (1 byte)| 
// outgoing packet fmt of reg cmd read --> |packet header|ERR CODE (1 byte)|read data (header.size - 1 bytes)|
void spi2c_read_reg(const struct device* dev, struct packet* in, struct packet* out) {
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;
  uint8_t reg_adr = *(uint8_t*)(in->data + 1);  
  uint8_t read_size = *(uint8_t*)(in->data + 2);

  uint8_t code = SPI2C_SUCCESS;
  uint8_t* reg = get_reg_data(data, reg_adr); 
  uint8_t read_data[read_size + 1];
  if (reg == NULL) {
    code = SPI2C_NO_REG;
    goto create_err_packet;
  }
  if (sizeof(*reg) != read_size) {
    code = SPI2C_INMEM;
    goto create_err_packet;
  }
  read_data[0] = code;
  memcpy((uint8_t*)(read_data + 1), reg, read_size);
  slave_packet_create(out, in->seqnum, read_size + 1, read_data);
  return;

  create_err_packet:
  slave_packet_create(out, in->seqnum, 1, &code); 
}

// incoming packet fmt of reg cmd write --> |packet header|CMD:0x03 (1 byte)|reg adr (1 byte)| write data (header.size - 2 bytes)| 
// outgoing packet fmt of reg cmd write --> |packet header|ERR CODE (1 byte)|
void spi2c_write_reg(const struct device* dev, struct packet* in, struct packet* out) {
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;

  uint8_t code = SPI2C_SUCCESS;
  uint8_t reg_adr = *(uint8_t*)(in->data + 1);
  uint8_t* reg = get_reg_data(data, reg_adr);
	if (reg == NULL) { 
    code = SPI2C_NO_REG;
    goto create_err_packet;
  }
  uint8_t* w_data = (uint8_t*)(in->data + 2);
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

  for (;;) {
    uint8_t rx_buffer[MAX_PACKET_SIZE] = {0};
    struct packet* rx = (struct packet*)rx_buffer;
    k_msgq_get(&cmd_queue, rx, K_FOREVER);
    uint8_t tx_buffer[MAX_PACKET_SIZE] = {0};
    struct packet* tx = (struct packet*)tx_buffer;
    /*
    TODO
    if (preform_crc8(rx) != rx->crc8) {
      uint8_t code = SPI2C_INVAL_CRC;
      slave_packet_create(tx, tx->seqnum, 1, &code);
      goto finish;
    }
    */
    uint8_t cmd = *(uint8_t*)rx->data;
    switch(cmd) {
      case SPI2C_DEV_WRITE:
        spi2c_dev_write(dev, rx, tx);
        break;
      case SPI2C_DEV_READ:
        spi2c_dev_read(dev, rx, tx);
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
    k_msgq_put(&spi_queue, tx, K_FOREVER);
  }
}

// callback function, lets main thread know spi transfer completed
void spi2c_cmd_cb(const struct device* dev, int result, void* data) {
  struct k_sem* transfer_fin  = (struct k_sem*)data;
  k_sem_give(transfer_fin);
}

// main spi thread, all spi transactions are completed through here
void spi2c_transceive_thread(void* p1, void* p2, void* p3) {
  struct device* dev = (struct device*)p1;
  struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
  struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;

  struct spi_buf_set tx_buf_set;
  struct spi_buf_set rx_buf_set;
  struct spi_buf tx_buf;
  struct spi_buf rx_buf;
  struct k_sem transfer_fin;
  k_sem_init(&transfer_fin, 0, 1);
  static uint8_t tx_dummy[MAX_PACKET_SIZE] = {0};
  for (;;) {
    int code;
    uint8_t tx_buffer[MAX_PACKET_SIZE] = {0};
    uint8_t rx_buffer[MAX_PACKET_SIZE] = {0};
    struct packet* tx = (struct packet*)tx_buffer;
    struct packet* rx = (struct packet*)rx_buffer;
    if (k_msgq_get(&spi_queue, tx, K_NO_WAIT) == 0) {
      // there is a msg to send
      // pull gpio line
      init_buffer(&rx_buf_set, &rx_buf, sizeof(struct packet) + tx->size, (void*)rx);
      init_buffer(&tx_buf_set, &tx_buf, sizeof(struct packet) + tx->size, (void*)tx);
      spi_transceive_cb(cfg->spi_dev.bus, &cfg->spi_dev.config, &tx_buf_set, &rx_buf_set, spi2c_cmd_cb, (void*)&transfer_fin);
      if (gpio_pin_set_dt(&cfg->signal_gpio, 1)) {
        printk("failed to set gpio pin\n");
      }  
      k_sem_take(&transfer_fin, K_FOREVER);
      gpio_pin_set_dt(&cfg->signal_gpio, 0); // set low
      continue;
    } 
    init_buffer(&rx_buf_set, &rx_buf, MAX_PACKET_SIZE, (void*)rx);
    init_buffer(&tx_buf_set, &tx_buf, MAX_PACKET_SIZE, (void*)tx_dummy);
    if (code = spi_transceive_cb(cfg->spi_dev.bus, &cfg->spi_dev.config, &tx_buf_set, &rx_buf_set, spi2c_cmd_cb, (void*)&transfer_fin)) {
      printk("transceive failed, code: %d\n", code);
    }
    k_sem_take(&transfer_fin, K_FOREVER);
    k_msgq_put(&cmd_queue, rx, K_FOREVER);
  }
}

static uint8_t spi2c_init_devices(const struct device* dev) {
	struct spi2c_com_data* data = (struct spi2c_com_data*)dev->data;
	struct spi2c_com_cfg* cfg = (struct spi2c_com_cfg*)dev->config;
  for (int i = 0; i < cfg->dev_amt; i++) {
    if (!device_is_ready(cfg->custom_devs[i])) {
      data->d_stat = SPI2C_UNINIT;
      return SPI2C_UNINIT;
    }
  }
  if (!gpio_is_ready_dt(&cfg->signal_gpio)) {
    data->d_stat = SPI2C_UNINIT;
    return SPI2C_UNINIT;
  }
  if (gpio_pin_configure_dt(&cfg->signal_gpio, GPIO_OUTPUT_INACTIVE)) {
    data->d_stat = SPI2C_UNINIT;
    return SPI2C_UNINIT;
  }
  gpio_pin_set_dt(&cfg->signal_gpio, 0);
	if (!spi_is_ready_dt(&cfg->spi_dev)) {
		data->d_stat = SPI2C_SPI_ERR;
		return SPI2C_SPI_ERR;
	}
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

  data->d_stat = SPI2C_INIT;
	return SPI2C_INIT;
}

static const struct spi2c_driver spi2c_driver_impl = {
	.spi2c_begin = spi2c_begin_impl,
};

#define SPI2C_ELEM_TO_DEV(node_id, prop, idx) \
	[idx] = DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)),

#define SPI2C_DEFINE(inst)                                                \
	static struct spi2c_com_data spi2c_com_data_##inst = {                  \
		.d_stat = SPI2C_UNINIT,                                               \
	};                                                                      \
                                                                          \
	static const struct spi2c_com_cfg spi2c_com_cfg_##inst = {              \
		.custom_devs = {                                                      \
			DT_FOREACH_PROP_ELEM(DT_DRV_INST(inst), custom_devs,                \
						  SPI2C_ELEM_TO_DEV)                                          \
		},                                                                    \
		.spi_dev = SPI_DT_SPEC_GET(DT_INST_PHANDLE(inst, spi_dev), SPI_OP),   \
    .dev_amt = (uint8_t)DT_INST_PROP(inst, device_amt),                   \
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
