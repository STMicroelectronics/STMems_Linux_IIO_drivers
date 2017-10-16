/*
 * STMicroelectronics lps33hw buffer driver
 *
 * Copyright 2017 STMicroelectronics Inc.
 *
 * Lorenzo Bianconi <lorenzo.bianconi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/interrupt.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/events.h>

#include "st_lps33hw.h"

#define ST_LPS33HW_FIFO_CTRL_ADDR		0x14
#define ST_LPS33HW_FIFO_THS_MASK		0x1f
#define ST_LPS33HW_FIFO_MODE_MASK		0xe0
#define ST_LPS33HW_INT_FTH_MASK			0x10

#define ST_LPS33HW_FIFO_SRC_ADDR		0x26
#define ST_LPS33HW_FIFO_SRC_DIFF_MASK		0x1f

#define ST_LPS33HW_PRESS_OUT_XL_ADDR		0x28

#define ST_LPS33HW_PRESS_SAMPLE_LEN		3
#define ST_LPS33HW_TEMP_SAMPLE_LEN		2
#define ST_LPS33HW_FIFO_SAMPLE_LEN		(ST_LPS33HW_PRESS_SAMPLE_LEN + \
						 ST_LPS33HW_TEMP_SAMPLE_LEN)

static inline s64 st_lps33hw_get_time_ns(void)
{
	struct timespec ts;

	get_monotonic_boottime(&ts);

	return timespec_to_ns(&ts);
}

#define ST_LPS33HW_EWMA_LEVEL			96
#define ST_LPS33HW_EWMA_DIV			128
static inline s64 st_lps33hw_ewma(s64 old, s64 new, int weight)
{
	s64 diff, incr;

	diff = new - old;
	incr = div_s64((ST_LPS33HW_EWMA_DIV - weight) * diff,
		       ST_LPS33HW_EWMA_DIV);

	return old + incr;
}

static int st_lps33hw_set_fifo_mode(struct st_lps33hw_hw *hw,
				    enum st_lps33hw_fifo_mode mode)
{
	switch (mode) {
	case ST_LPS33HW_BYPASS:
	case ST_LPS33HW_STREAM:
		break;
	default:
		return -EINVAL;
	}
	return st_lps33hw_write_with_mask(hw, ST_LPS33HW_FIFO_CTRL_ADDR,
					  ST_LPS33HW_FIFO_MODE_MASK, mode);
}

static int st_lps33hw_update_fifo_watermark(struct st_lps33hw_hw *hw, u8 val)
{
	int err;

	err = st_lps33hw_write_with_mask(hw, ST_LPS33HW_FIFO_CTRL_ADDR,
					 ST_LPS33HW_FIFO_THS_MASK, val);
	if (err < 0)
		return err;

	hw->watermark = val;

	return 0;
}

ssize_t st_lps33hw_sysfs_set_hwfifo_watermark(struct device * dev,
					      struct device_attribute * attr,
					      const char *buf, size_t count)
{
	struct st_lps33hw_sensor *sensor = iio_priv(dev_get_drvdata(dev));
	int err, watermark;

	err = kstrtoint(buf, 10, &watermark);
	if (err < 0)
		return err;

	if (watermark < 1 || watermark > ST_LPS33HW_MAX_FIFO_LENGTH)
		return -EINVAL;

	err = st_lps33hw_update_fifo_watermark(sensor->hw, watermark);

	return err < 0 ? err : count;
}

static int st_lps33hw_read_fifo(struct st_lps33hw_hw *hw)
{
	u8 iio_buff[ALIGN(sizeof(u32) + sizeof(s64), sizeof(s64))];
	u8 status, buff[ST_LPS33HW_FIFO_SAMPLE_LEN];
	int err, i, fifo_depth;

	err = hw->tf->read(hw->dev, ST_LPS33HW_FIFO_SRC_ADDR,
			   sizeof(status), &status);
	if (err < 0)
		return err;

	fifo_depth = (status & ST_LPS33HW_FIFO_SRC_DIFF_MASK);
	for (i = 0; i < fifo_depth; i++) {
		err = hw->tf->read(hw->dev, ST_LPS33HW_PRESS_OUT_XL_ADDR,
				   sizeof(buff), buff);
		if (err < 0)
			return err;
		/* press sample */
		memcpy(iio_buff, buff, ST_LPS33HW_PRESS_SAMPLE_LEN);
		iio_push_to_buffers_with_timestamp(
				hw->iio_devs[ST_LPS33HW_PRESS],
				iio_buff, hw->ts);
		/* temp sample */
		memcpy(iio_buff, buff + ST_LPS33HW_PRESS_SAMPLE_LEN,
		       ST_LPS33HW_TEMP_SAMPLE_LEN);
		iio_push_to_buffers_with_timestamp(
				hw->iio_devs[ST_LPS33HW_TEMP],
				iio_buff, hw->ts);
		hw->ts += hw->delta_ts;
	}

	return fifo_depth;
}

ssize_t st_lps33hw_sysfs_flush_fifo(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct st_lps33hw_sensor *sensor = iio_priv(indio_dev);
	struct st_lps33hw_hw *hw = sensor->hw;
	u64 type, event;
	int len;

	mutex_lock(&indio_dev->mlock);
	if (!iio_buffer_enabled(indio_dev)) {
		mutex_unlock(&indio_dev->mlock);
		return -EINVAL;
	}

	mutex_lock(&hw->fifo_lock);
	len = st_lps33hw_read_fifo(hw);
	mutex_unlock(&hw->fifo_lock);

	type = len > 0 ? IIO_EV_DIR_FIFO_DATA : IIO_EV_DIR_FIFO_EMPTY;
	if (sensor->type == ST_LPS33HW_PRESS)
		event = IIO_UNMOD_EVENT_CODE(IIO_PRESSURE, -1,
					     IIO_EV_TYPE_FIFO_FLUSH, type);
	else
		event = IIO_UNMOD_EVENT_CODE(IIO_TEMP, -1,
					     IIO_EV_TYPE_FIFO_FLUSH, type);
	iio_push_event(indio_dev, event, st_lps33hw_get_time_ns());
	mutex_unlock(&indio_dev->mlock);

	return size;
}


static irqreturn_t st_lps33hw_irq_handler(int irq, void *private)
{
	s64 delta_ts, ts = st_lps33hw_get_time_ns();
	struct st_lps33hw_hw *hw = private;

	delta_ts = div_s64((ts - hw->ts_irq), hw->watermark);
	if (hw->odr >= 50)
		hw->delta_ts = st_lps33hw_ewma(hw->delta_ts, delta_ts,
					       ST_LPS33HW_EWMA_LEVEL);
	else
		hw->delta_ts = delta_ts;
	hw->ts_irq = ts;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t st_lps33hw_irq_thread(int irq, void *private)
{
	struct st_lps33hw_hw *hw = private;

	mutex_lock(&hw->fifo_lock);
	st_lps33hw_read_fifo(hw);
	mutex_unlock(&hw->fifo_lock);

	return IRQ_HANDLED;
}

static int st_lps33hw_buffer_preenable(struct iio_dev *indio_dev)
{
	struct st_lps33hw_sensor *sensor = iio_priv(indio_dev);
	struct st_lps33hw_hw *hw = sensor->hw;
	int err;

	err = st_lps33hw_set_fifo_mode(sensor->hw, ST_LPS33HW_STREAM);
	if (err < 0)
		return err;

	err = st_lps33hw_update_fifo_watermark(hw, hw->watermark);
	if (err < 0)
		return err;

	err = st_lps33hw_write_with_mask(sensor->hw, ST_LPS33HW_CTRL3_ADDR,
					 ST_LPS33HW_INT_FTH_MASK, true);
	if (err < 0)
		return err;

	err = st_lps33hw_set_enable(sensor, true);
	if (err < 0)
		return err;

	hw->delta_ts = div_s64(1000000000UL, hw->odr);
	hw->ts = st_lps33hw_get_time_ns();
	hw->ts_irq = hw->ts;

	return 0;
}

static int st_lps33hw_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct st_lps33hw_sensor *sensor = iio_priv(indio_dev);
	int err;

	err = st_lps33hw_set_fifo_mode(sensor->hw, ST_LPS33HW_BYPASS);
	if (err < 0)
		return err;

	err = st_lps33hw_write_with_mask(sensor->hw, ST_LPS33HW_CTRL3_ADDR,
					 ST_LPS33HW_INT_FTH_MASK, false);
	if (err < 0)
		return err;

	return st_lps33hw_set_enable(sensor, false);
}

static const struct iio_buffer_setup_ops st_lps33hw_buffer_ops = {
	.preenable = st_lps33hw_buffer_preenable,
	.postdisable = st_lps33hw_buffer_postdisable,
};

int st_lps33hw_allocate_buffers(struct st_lps33hw_hw *hw)
{
	struct iio_buffer *buffer;
	int err, i;

	err = devm_request_threaded_irq(hw->dev, hw->irq,
					st_lps33hw_irq_handler,
					st_lps33hw_irq_thread,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					"lps33hw", hw);
	if (err)
		return err;

	for (i = 0; i < ST_LPS33HW_SENSORS_NUMB; i++) {
		buffer = iio_kfifo_allocate(hw->iio_devs[i]);
		if (!buffer) {
			err = -ENOMEM;
			goto fifo_allocate_error;
		}

		iio_device_attach_buffer(hw->iio_devs[i], buffer);
		hw->iio_devs[i]->modes |= INDIO_BUFFER_HARDWARE;
		hw->iio_devs[i]->setup_ops = &st_lps33hw_buffer_ops;

		err = iio_buffer_register(hw->iio_devs[i],
					  hw->iio_devs[i]->channels,
					  hw->iio_devs[i]->num_channels);
		if (err)
			goto fifo_allocate_error;
	}
	return 0;

fifo_allocate_error:
	for (i--; i >= 0; i--)
		iio_buffer_unregister(hw->iio_devs[i]);
	for (i = 0; i < ST_LPS33HW_SENSORS_NUMB; i++) {
		if (!hw->iio_devs[i]->buffer)
			continue;
		iio_kfifo_free(hw->iio_devs[i]->buffer);
	}
	return err;
}

int st_lps33hw_deallocate_buffers(struct st_lps33hw_hw *hw)
{
	int i;

	for (i = 0; i < ST_LPS33HW_SENSORS_NUMB; i++) {
		iio_buffer_unregister(hw->iio_devs[i]);
		iio_kfifo_free(hw->iio_devs[i]->buffer);
	}
	return 0;
}

MODULE_DESCRIPTION("STMicroelectronics lps33hw buffer driver");
MODULE_AUTHOR("Lorenzo Bianconi <lorenzo.bianconi@st.com>");
MODULE_LICENSE("GPL v2");
