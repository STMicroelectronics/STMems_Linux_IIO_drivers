/*
 * STMicroelectronics lps22df driver
 *
 * Copyright 2021 STMicroelectronics Inc.
 *
 * Mario Tesi <mario.tesi@st.com>
 *
 * Licensed under the GPL-2.
 */

#ifndef __ST_LPS22DF_H
#define __ST_LPS22DF_H

#include <linux/module.h>
#include <linux/types.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>

#define ST_LPS22DF_MAX_FIFO_LENGTH		127

#define ST_LPS22DF_INTERRUPT_CFG_ADDR		0x0b
#define ST_LPS22DF_LIR_MASK			BIT(2)

#define ST_LPS22DF_WHO_AM_I_ADDR		0x0f
#define ST_LPS22DF_WHO_AM_I_VAL			0xb4

#define ST_LPS22DF_CTRL_REG1_ADDR		0x10
#define ST_LPS22DF_AVG_MASK			0x07
#define ST_LPS22DF_ODR_MASK			0x78

#define ST_LPS22DF_CTRL_REG2_ADDR		0x11
#define ST_LPS22DF_SWRESET_MASK			BIT(2)
#define ST_LPS22DF_BDU_MASK			BIT(3)
#define ST_LPS22DF_EN_LPFP_MASK			BIT(4)
#define ST_LPS22DF_BOOT_MASK			BIT(7)

#define ST_LPS22DF_CTRL3_ADDR			0x12
#define ST_LPS22DF_IF_ADD_INC_MASK		BIT(0)
#define ST_LPS22DF_PP_OD_MASK			BIT(1)
#define ST_LPS22DF_INT_H_L_MASK			BIT(3)

#define ST_LPS22DF_CTRL4_ADDR			0x13
#define ST_LPS22DF_INT_F_WTM_MASK		BIT(1)

#define ST_LPS22DF_FIFO_CTRL_ADDR		0x14
#define ST_LPS22DF_FIFO_MODE_MASK		0x03

#define ST_LPS22DF_FIFO_WTM_ADDR		0x15
#define ST_LPS22DF_FIFO_THS_MASK		0x7f

#define ST_LPS22DF_FIFO_STATUS1_ADDR		0x25
#define ST_LPS22DF_FIFO_SRC_DIFF_MASK		0xff

#define ST_LPS22DF_FIFO_STATUS2_ADDR		0x26
#define ST_LPS22DF_FIFO_WTM_IA_MASK		BIT(7)

#define ST_LPS22DF_PRESS_OUT_XL_ADDR		0x28

#define ST_LPS22DF_TEMP_OUT_L_ADDR		0x2b

#define ST_LPS22DF_FIFO_DATA_OUT_PRESS_XL_ADDR	0x78

#define ST_LPS22DF_PRESS_FS_AVL_GAIN		(1000000000UL / 4096UL)
#define ST_LPS22DF_TEMP_FS_AVL_GAIN		(1000000000UL / 100UL)

#define ST_LPS22DF_ODR_LIST_NUM			9

enum st_lps22df_sensor_type {
	ST_LPS22DF_PRESS = 0,
	ST_LPS22DF_TEMP,
	ST_LPS22DF_SENSORS_NUMB,
};

enum st_lps22df_fifo_mode {
	ST_LPS22DF_BYPASS = 0x0,
	ST_LPS22DF_STREAM = 0x2,
};

#define ST_LPS22DF_PRESS_SAMPLE_LEN		3
#define ST_LPS22DF_TEMP_SAMPLE_LEN		2

#define ST_LPS22DF_TX_MAX_LENGTH		64
#define ST_LPS22DF_RX_MAX_LENGTH		((ST_LPS22DF_MAX_FIFO_LENGTH + 1) * \
						 ST_LPS22DF_PRESS_SAMPLE_LEN)

struct st_lps22df_transfer_buffer {
	u8 rx_buf[ST_LPS22DF_RX_MAX_LENGTH];
	u8 tx_buf[ST_LPS22DF_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct st_lps22df_transfer_function {
	int (*write)(struct device *dev, u8 addr, int len, u8 *data);
	int (*read)(struct device *dev, u8 addr, int len, u8 *data);
};

struct st_lps22df_hw {
	struct device *dev;
	int irq;

	struct mutex fifo_lock;
	struct mutex lock;
	u8 watermark;

	struct iio_dev *iio_devs[ST_LPS22DF_SENSORS_NUMB];
	u8 enable_mask;
	u8 odr;

	s64 last_fifo_ts;
	s64 delta_ts;
	s64 ts_irq;
	s64 ts;

	const struct st_lps22df_transfer_function *tf;
	struct st_lps22df_transfer_buffer tb;
};

struct st_lps22df_sensor {
	struct st_lps22df_hw *hw;
	enum st_lps22df_sensor_type type;
	char name[32];

	u32 gain;
	u8 odr;
};

int st_lps22df_common_probe(struct device *dev, int irq, const char *name,
			    const struct st_lps22df_transfer_function *tf_ops);
int st_lps22df_write_with_mask(struct st_lps22df_hw *hw, u8 addr, u8 mask,
			       u8 data);
int st_lps22df_allocate_buffers(struct st_lps22df_hw *hw);
int st_lps22df_deallocate_buffers(struct st_lps22df_hw *hw);
int st_lps22df_set_enable(struct st_lps22df_sensor *sensor, bool enable);
ssize_t st_lps22df_sysfs_set_hwfifo_watermark(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count);
ssize_t st_lps22df_sysfs_flush_fifo(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size);

#endif /* __ST_LPS22DF_H */
