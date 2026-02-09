#ifndef CUSTOM_DEV_H
#define CUSTOM_DEV_H
#include <zephyr/device.h>
#include <spi2c-com.h>

typedef uint8_t (*write_custom_dev_t)(const struct device* dev, struct packet* in, struct packet* out);
typedef uint8_t (*read_custom_dev_t)(const struct device* dev, struct packet* in, struct packet* out);
typedef uint8_T (*get_custom_dev_adr_t)(const struct device* dev);

struct custom_dev_api {
  write_custom_dev_t write;
  read_custom_dev_t read;
  get_custom_dev_adr_t get_custom_dev_adr;
};

static inline uint8_t spi2c_write_custom_dev(const struct device* dev, struct packet* in, struct packet* out) {
  struct custom_dev_api* api = (struct custom_dev_api*)dev->api;
  return api->write(dev, in, out);
}

static inline uint8_t spi2c_read_custom_dev(const struct device* dev, struct packet* in, struct packet* out) {
  struct custom_dev_api* api = (struct custom_dev_api*)dev->api;
  return api->read(dev, in, out);
}

static inline uint8_t spi2c_get_custom_dev_adr(const struct device* dev) {
  struct custom_dev_api* api = (struct custom_dev_api*)dev->api;
  return api->get_custom_dev_adr(dev);
}

#endif
