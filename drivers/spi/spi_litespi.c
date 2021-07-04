/*
 * Copyright (c) 2019 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT litex_spi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_litespi);
#include "spi_litespi.h"
#include <stdbool.h>

static inline uint32_t spi_master_control_read(void) {
        return *((uint32_t*)SPI_CONTROL_REG);
}
static inline void spi_master_control_write(uint32_t v) {
	*((uint32_t*)SPI_CONTROL_REG) = v;
}
static inline uint32_t spi_master_control_start_extract(uint32_t oldword) {
        uint32_t mask = ((1 << 1)-1);
        return ( (oldword >> 0) & mask );
}
static inline uint32_t spi_master_control_start_read(void) {
        uint32_t word = spi_master_control_read();
        return spi_master_control_start_extract(word);
}
static inline uint32_t spi_master_control_start_replace(uint32_t oldword, uint32_t plain_value) {
        uint32_t mask = ((1 << 1)-1);
        return (oldword & (~(mask << 0))) | (mask & plain_value)<< 0 ;
}                                                       
static inline void spi_master_control_start_write(uint32_t plain_value) {
        uint32_t oldword = spi_master_control_read();
        uint32_t newword = spi_master_control_start_replace(oldword, plain_value);
        spi_master_control_write(newword);
}
static inline uint32_t spi_master_control_length_extract(uint32_t oldword) {
        uint32_t mask = ((1 << 8)-1);
        return ( (oldword >> 8) & mask );
}
static inline uint32_t spi_master_control_length_read(void) {
        uint32_t word = spi_master_control_read();
        return spi_master_control_length_extract(word);
}
static inline uint32_t spi_master_control_length_replace(uint32_t oldword, uint32_t plain_value) {
        uint32_t mask = ((1 << 8)-1);
        return (oldword & (~(mask << 8))) | (mask & plain_value)<< 8 ;
}
static inline void spi_master_control_length_write(uint32_t plain_value) {
        uint32_t oldword = spi_master_control_read();
        uint32_t newword = spi_master_control_length_replace(oldword, plain_value);
        spi_master_control_write(newword);
}

/* Helper Functions */
static int spi_config(const struct spi_config *config)
{
	uint8_t cs = 0x00;

	if (config->slave != 0) {
		if (config->slave >= SPI_MAX_CS_SIZE) {
			LOG_ERR("More slaves than supported");
			return -ENOTSUP;
		}
		cs = (uint8_t)(config->slave);
	}

	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		LOG_ERR("Word size must be %d", SPI_WORD_SIZE);
		return -ENOTSUP;
	}

	if (config->operation & SPI_CS_ACTIVE_HIGH) {
		LOG_ERR("CS active high not supported");
		return -ENOTSUP;
	}

	if (config->operation & SPI_LOCK_ON) {
		LOG_ERR("Lock On not supported");
		return -ENOTSUP;
	}

	if ((config->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_ERR("Only supports single mode");
		return -ENOTSUP;
	}

	if (config->operation & SPI_TRANSFER_LSB) {
		LOG_ERR("LSB first not supported");
		return -ENOTSUP;
	}

	if (config->operation & (SPI_MODE_CPOL | SPI_MODE_CPHA)) {
		LOG_ERR("Only supports CPOL=CPHA=0");
		return -ENOTSUP;
	}

	if (config->operation & SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode not supported");
		return -ENOTSUP;
	}

	/* Set Loopback */
	if (config->operation & SPI_MODE_LOOP) {
		litex_write8(SPI_ENABLE, SPI_LOOPBACK_REG);
	}
	/* Set word size */
	spi_master_control_length_write(SPI_WORD_SIZE);
	return 0;
}

static void spi_litespi_send(const struct device *dev, uint8_t frame)
{
	/* Write frame to register */
	litex_write8(frame, SPI_MOSI_DATA_REG);
	/* Start the transfer */
	spi_master_control_start_write(SPI_ENABLE);
	/* Wait until the transfer ends */
	while (!(litex_read8(SPI_STATUS_REG)))
		;
}

static uint8_t spi_litespi_recv(void)
{
    /* Return data inside MISO register */
	return litex_read8(SPI_MISO_DATA_REG);
}

static void spi_litespi_xfer(const struct device *dev,
			     const struct spi_config *config)
{
	struct spi_context *ctx = &SPI_DATA(dev)->ctx;
	uint32_t send_len = spi_context_longest_current_buf(ctx);
	uint8_t read_data;

	for (uint32_t i = 0; i < send_len; i++) {
		/* Send a frame */
		if (i < ctx->tx_len) {
			spi_litespi_send(dev, (uint8_t) (ctx->tx_buf)[i]);
		} else {
			/* Send dummy bytes */
			spi_litespi_send(dev, 0);
		}
		/* Receive a frame */
		read_data = spi_litespi_recv();
		if (i < ctx->rx_len) {
			ctx->rx_buf[i] = read_data;
		}
	}
	spi_context_complete(ctx, 0);
}

/* API Functions */

static int spi_litespi_init(const struct device *dev)
{
	return 0;
}

static int spi_litespi_transceive(const struct device *dev,
				  const struct spi_config *config,
				  const struct spi_buf_set *tx_bufs,
				  const struct spi_buf_set *rx_bufs)
{
	spi_config(config);
	litex_write8(1, SPI_CS_REG);
	spi_context_buffers_setup(&SPI_DATA(dev)->ctx, tx_bufs, rx_bufs, 1);
	spi_litespi_xfer(dev, config);
	litex_write8(0, SPI_CS_REG);
	spi_master_control_start_write(0);
	spi_master_control_length_write(0);
	return 0;
}

#ifdef CONFIG_SPI_ASYNC
static int spi_litespi_transceive_async(const struct device *dev,
					const struct spi_config *config,
					const struct spi_buf_set *tx_bufs,
					const struct spi_buf_set *rx_bufs,
					struct k_poll_signal *async)
{
	return -ENOTSUP;
}
#endif /* CONFIG_SPI_ASYNC */

static int spi_litespi_release(const struct device *dev,
			       const struct spi_config *config)
{
	if (!(litex_read8(SPI_STATUS_REG))) {
		return -EBUSY;
	}
	return 0;
}

/* Device Instantiation */
static struct spi_driver_api spi_litespi_api = {
	.transceive = spi_litespi_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_litespi_transceive_async,
#endif /* CONFIG_SPI_ASYNC */
	.release = spi_litespi_release,
};

#define SPI_INIT(n)	\
	static struct spi_litespi_data spi_litespi_data_##n = { \
		SPI_CONTEXT_INIT_LOCK(spi_litespi_data_##n, ctx), \
		SPI_CONTEXT_INIT_SYNC(spi_litespi_data_##n, ctx), \
	}; \
	static struct spi_litespi_cfg spi_litespi_cfg_##n = { \
		.base = DT_INST_REG_ADDR_BY_NAME(n, control), \
	}; \
	DEVICE_DT_INST_DEFINE(n, \
			spi_litespi_init, \
			device_pm_control_nop, \
			&spi_litespi_data_##n, \
			&spi_litespi_cfg_##n, \
			POST_KERNEL, \
			CONFIG_SPI_INIT_PRIORITY, \
			&spi_litespi_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_INIT)
