/*
 * STMicroelectronics st_lsm6dso32x FIFO buffer library driver
 *
 * Copyright 2021 STMicroelectronics Inc.
 *
 * Tesi Mario <mario.tesi@st.com>
 *
 * Licensed under the GPL-2.
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/events.h>
#include <linux/iio/buffer.h>
#include <asm/unaligned.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/of.h>

#include "st_lsm6dso32x.h"

#define ST_LSM6DSO32X_SAMPLE_DISCHARD			0x7ffd

/* Timestamp convergence filter parameters */
#define ST_LSM6DSO32X_EWMA_LEVEL			120
#define ST_LSM6DSO32X_EWMA_DIV				128

/* FIFO tags */
enum {
	ST_LSM6DSO32X_GYRO_TAG = 0x01,
	ST_LSM6DSO32X_ACC_TAG = 0x02,
	ST_LSM6DSO32X_TEMP_TAG = 0x03,
	ST_LSM6DSO32X_TS_TAG = 0x04,
	ST_LSM6DSO32X_EXT0_TAG = 0x0f,
	ST_LSM6DSO32X_EXT1_TAG = 0x10,
	ST_LSM6DSO32X_SC_TAG = 0x12,
};

/* Default timeout before to re-enable gyro */
static int lsm6dso32x_delay_gyro = 10;
module_param(lsm6dso32x_delay_gyro, int, 0644);
MODULE_PARM_DESC(lsm6dso32x_delay_gyro, "Delay for Gyro arming");
static bool delayed_enable_gyro;

static inline s64 st_lsm6dso32x_ewma(s64 old, s64 new, int weight)
{
	s64 diff, incr;

	diff = new - old;
	incr = div_s64((ST_LSM6DSO32X_EWMA_DIV - weight) * diff,
			ST_LSM6DSO32X_EWMA_DIV);

	return old + incr;
}

static inline int st_lsm6dso32x_reset_hwts(struct st_lsm6dso32x_hw *hw)
{
	u8 data = 0xaa;

	hw->ts = st_lsm6dso32x_get_time_ns();
	hw->ts_offset = hw->ts;
	hw->tsample = 0ull;

	return st_lsm6dso32x_write_locked(hw, ST_LSM6DSO32X_REG_TIMESTAMP2_ADDR,
					  data);
}

int st_lsm6dso32x_set_fifo_mode(struct st_lsm6dso32x_hw *hw,
				enum st_lsm6dso32x_fifo_mode fifo_mode)
{
	int err;

	err = st_lsm6dso32x_write_with_mask_locked(hw,
					ST_LSM6DSO32X_REG_FIFO_CTRL4_ADDR,
					ST_LSM6DSO32X_REG_FIFO_MODE_MASK,
					fifo_mode);
	if (err < 0)
		return err;

	hw->fifo_mode = fifo_mode;

	return 0;
}

static inline int
st_lsm6dso32x_set_sensor_batching_odr(struct st_lsm6dso32x_sensor *sensor,
				      bool enable)
{
	struct st_lsm6dso32x_hw *hw = sensor->hw;
	enum st_lsm6dso32x_sensor_id id = sensor->id;
	u8 data = 0;
	int err;

	if (enable) {
		err = st_lsm6dso32x_get_batch_val(sensor, sensor->odr,
						  sensor->uodr, &data);
		if (err < 0)
			return err;
	}

	return st_lsm6dso32x_update_bits_locked(hw,
				hw->odr_table_entry[id].batching_reg.addr,
				hw->odr_table_entry[id].batching_reg.mask,
				data);
}

int st_lsm6dso32x_update_watermark(struct st_lsm6dso32x_sensor *sensor,
				   u16 watermark)
{
	u16 fifo_watermark = ST_LSM6DSO32X_MAX_FIFO_DEPTH, cur_watermark = 0;
	struct st_lsm6dso32x_hw *hw = sensor->hw;
	struct st_lsm6dso32x_sensor *cur_sensor;
	__le16 wdata;
	int i, err;
	int data = 0;

	for (i = ST_LSM6DSO32X_ID_GYRO;
	     i <= ST_LSM6DSO32X_ID_STEP_COUNTER;
	     i++) {
		if (!hw->iio_devs[i])
			continue;

		cur_sensor = iio_priv(hw->iio_devs[i]);

		if (!(hw->enable_mask & BIT(cur_sensor->id)))
			continue;

		cur_watermark = (cur_sensor == sensor) ? watermark
						       : cur_sensor->watermark;

		fifo_watermark = min_t(u16, fifo_watermark, cur_watermark);
	}

	fifo_watermark = max_t(u16, fifo_watermark, 2);

	mutex_lock(&hw->page_lock);
	err = regmap_read(hw->regmap, ST_LSM6DSO32X_REG_FIFO_CTRL1_ADDR + 1,
			  &data);
	if (err < 0)
		goto out;

	fifo_watermark = ((data << 8) & ~ST_LSM6DSO32X_REG_FIFO_WTM_MASK) |
			 (fifo_watermark & ST_LSM6DSO32X_REG_FIFO_WTM_MASK);
	wdata = cpu_to_le16(fifo_watermark);

	err = regmap_bulk_write(hw->regmap, ST_LSM6DSO32X_REG_FIFO_CTRL1_ADDR,
				&wdata, sizeof(wdata));
out:
	mutex_unlock(&hw->page_lock);

	return err;
}

static struct
iio_dev *st_lsm6dso32x_get_iiodev_from_tag(struct st_lsm6dso32x_hw *hw, u8 tag)
{
	struct iio_dev *iio_dev;

	switch (tag) {
	case ST_LSM6DSO32X_GYRO_TAG:
		iio_dev = hw->iio_devs[ST_LSM6DSO32X_ID_GYRO];
		break;
	case ST_LSM6DSO32X_ACC_TAG:
		iio_dev = hw->iio_devs[ST_LSM6DSO32X_ID_ACC];
		break;
	case ST_LSM6DSO32X_TEMP_TAG:
		iio_dev = hw->iio_devs[ST_LSM6DSO32X_ID_TEMP];
		break;
	case ST_LSM6DSO32X_EXT0_TAG:
		if (hw->enable_mask & BIT(ST_LSM6DSO32X_ID_EXT0))
			iio_dev = hw->iio_devs[ST_LSM6DSO32X_ID_EXT0];
		else
			iio_dev = hw->iio_devs[ST_LSM6DSO32X_ID_EXT1];
		break;
	case ST_LSM6DSO32X_EXT1_TAG:
		iio_dev = hw->iio_devs[ST_LSM6DSO32X_ID_EXT1];
		break;
	case ST_LSM6DSO32X_SC_TAG:
		iio_dev = hw->iio_devs[ST_LSM6DSO32X_ID_STEP_COUNTER];
		break;
	default:
		iio_dev = NULL;
		break;
	}

	return iio_dev;
}

static int st_lsm6dso32x_read_fifo(struct st_lsm6dso32x_hw *hw)
{
	u8 iio_buf[ALIGN(ST_LSM6DSO32X_SAMPLE_SIZE, sizeof(s64)) + sizeof(s64)];
	u8 buf[6 * ST_LSM6DSO32X_FIFO_SAMPLE_SIZE], tag, *ptr;
	int i, err, word_len, fifo_len, read_len;
	struct st_lsm6dso32x_sensor *sensor;
	struct iio_dev *iio_dev;
	s64 ts_irq, hw_ts_old;
	__le16 fifo_status;
	u16 fifo_depth;
	s16 drdymask;
	u32 val;

	/* return if FIFO is already disabled */
	if (hw->fifo_mode == ST_LSM6DSO32X_FIFO_BYPASS)
		return 0;

	ts_irq = hw->ts - hw->delta_ts;

	err = st_lsm6dso32x_read_locked(hw, ST_LSM6DSO32X_REG_FIFO_STATUS1_ADDR,
					&fifo_status, sizeof(fifo_status));
	if (err < 0)
		return err;

	fifo_depth = le16_to_cpu(fifo_status) &
		     ST_LSM6DSO32X_REG_FIFO_STATUS_DIFF;
	if (!fifo_depth)
		return 0;

	fifo_len = fifo_depth * ST_LSM6DSO32X_FIFO_SAMPLE_SIZE;
	read_len = 0;

	while (read_len < fifo_len) {
		word_len = min_t(int, fifo_len - read_len, sizeof(buf));
		err = st_lsm6dso32x_read_locked(hw,
				ST_LSM6DSO32X_REG_FIFO_DATA_OUT_TAG_ADDR,
				buf, word_len);
		if (err < 0)
			return err;

		for (i = 0;
		     i < word_len;
		     i += ST_LSM6DSO32X_FIFO_SAMPLE_SIZE) {
			ptr = &buf[i + ST_LSM6DSO32X_TAG_SIZE];
			tag = buf[i] >> 3;

			if (tag == ST_LSM6DSO32X_TS_TAG) {
				val = get_unaligned_le32(ptr);
				hw_ts_old = hw->hw_ts;
				hw->hw_ts = val * hw->ts_delta_ns;
				hw->ts_offset = st_lsm6dso32x_ewma(hw->ts_offset,
							ts_irq - hw->hw_ts,
							ST_LSM6DSO32X_EWMA_LEVEL);
				ts_irq += hw->hw_ts;

				if (!hw->tsample)
					hw->tsample = hw->ts_offset + hw->hw_ts;
				else
					hw->tsample = hw->tsample + hw->hw_ts -
						      hw_ts_old;
			} else {
				iio_dev = st_lsm6dso32x_get_iiodev_from_tag(hw, tag);
				if (!iio_dev)
					continue;

				sensor = iio_priv(iio_dev);

				/* Skip samples if not ready */
				drdymask = (s16)le16_to_cpu(get_unaligned_le16(ptr));
				if (unlikely(drdymask >=
				    ST_LSM6DSO32X_SAMPLE_DISCHARD)) {
					continue;
				}

				/*
				 * hw ts in not queued in FIFO if only step
				 * counter enabled
				 */
				if (sensor->id == ST_LSM6DSO32X_ID_STEP_COUNTER) {
					val = get_unaligned_le32(ptr + 2);
					hw->tsample = val * hw->ts_delta_ns;
				} else {
					hw->tsample = min_t(s64,
						st_lsm6dso32x_get_time_ns(),
						hw->tsample);
					sensor->last_fifo_timestamp = hw->tsample;
				}

				memcpy(iio_buf, ptr, ST_LSM6DSO32X_SAMPLE_SIZE);

				/* support decimation for ODR < 12.5 Hz */
				if (sensor->dec_counter > 0) {
					sensor->dec_counter--;
				} else {
					sensor->dec_counter = sensor->decimator;
					iio_push_to_buffers_with_timestamp(iio_dev,
									iio_buf,
									hw->tsample);
				}
			}
		}
		read_len += word_len;
	}

	return read_len;
}

ssize_t st_lsm6dso32x_get_max_watermark(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);
	struct st_lsm6dso32x_sensor *sensor = iio_priv(iio_dev);

	return sprintf(buf, "%d\n", sensor->max_watermark);
}

ssize_t st_lsm6dso32x_get_watermark(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);
	struct st_lsm6dso32x_sensor *sensor = iio_priv(iio_dev);

	return sprintf(buf, "%d\n", sensor->watermark);
}

ssize_t st_lsm6dso32x_set_watermark(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);
	struct st_lsm6dso32x_sensor *sensor = iio_priv(iio_dev);
	int err, val;

	err = iio_device_claim_direct_mode(iio_dev);
	if (err)
		return err;

	err = kstrtoint(buf, 10, &val);
	if (err < 0)
		goto out;

	err = st_lsm6dso32x_update_watermark(sensor, val);
	if (err < 0)
		goto out;

	sensor->watermark = val;
	iio_device_release_direct_mode(iio_dev);

out:
	return err < 0 ? err : size;
}

ssize_t st_lsm6dso32x_flush_fifo(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);
	struct st_lsm6dso32x_sensor *sensor = iio_priv(iio_dev);
	struct st_lsm6dso32x_hw *hw = sensor->hw;
	s64 type;
	s64 event;
	int count;
	s64 fts;
	s64 ts;

	mutex_lock(&hw->fifo_lock);
	ts = st_lsm6dso32x_get_time_ns();
	hw->delta_ts = ts - hw->ts;
	hw->ts = ts;
	set_bit(ST_LSM6DSO32X_HW_FLUSH, &hw->state);
	count = st_lsm6dso32x_read_fifo(hw);
	sensor->dec_counter = 0;
	if (count > 0)
		fts = sensor->last_fifo_timestamp;
	else
		fts = ts;
	mutex_unlock(&hw->fifo_lock);

	type = count > 0 ? IIO_EV_DIR_FIFO_DATA : IIO_EV_DIR_FIFO_EMPTY;
	event = IIO_UNMOD_EVENT_CODE(iio_dev->channels[0].type, -1,
				     IIO_EV_TYPE_FIFO_FLUSH, type);
	iio_push_event(iio_dev, event, fts);

	return size;
}

int st_lsm6dso32x_suspend_fifo(struct st_lsm6dso32x_hw *hw)
{
	int err;

	mutex_lock(&hw->fifo_lock);
	st_lsm6dso32x_read_fifo(hw);
	err = st_lsm6dso32x_set_fifo_mode(hw, ST_LSM6DSO32X_FIFO_BYPASS);
	mutex_unlock(&hw->fifo_lock);

	return err;
}

int st_lsm6dso32x_update_batching(struct iio_dev *iio_dev, bool enable)
{
	struct st_lsm6dso32x_sensor *sensor = iio_priv(iio_dev);
	struct st_lsm6dso32x_hw *hw = sensor->hw;
	int err;

	disable_irq(hw->irq);
	err = st_lsm6dso32x_set_sensor_batching_odr(sensor, enable);
	enable_irq(hw->irq);

	return err;
}

static int st_lsm6dso32x_update_fifo(struct iio_dev *iio_dev, bool enable)
{
	struct st_lsm6dso32x_sensor *sensor = iio_priv(iio_dev);
	struct st_lsm6dso32x_hw *hw = sensor->hw;
	int err;

	if (sensor->id == ST_LSM6DSO32X_ID_GYRO && !enable)
		delayed_enable_gyro = true;

	if (sensor->id == ST_LSM6DSO32X_ID_GYRO &&
	    enable && delayed_enable_gyro) {
		delayed_enable_gyro = false;
		msleep(lsm6dso32x_delay_gyro);
	}

	disable_irq(hw->irq);

	switch (sensor->id) {
	case ST_LSM6DSO32X_ID_EXT0:
	case ST_LSM6DSO32X_ID_EXT1:
		err = st_lsm6dso32x_shub_set_enable(sensor, enable);
		if (err < 0)
			goto out;
		break;
	case ST_LSM6DSO32X_ID_STEP_COUNTER:
		err = st_lsm6dso32x_step_counter_set_enable(sensor, enable);
		if (err < 0)
			goto out;
		break;
	case ST_LSM6DSO32X_ID_TEMP:
		/*
		 * This is an auxiliary sensor, it need to get batched
		 * toghether at least with a primary sensor (Acc/Gyro).
		 */
		if (!(hw->enable_mask & (BIT(ST_LSM6DSO32X_ID_ACC) |
					 BIT(ST_LSM6DSO32X_ID_GYRO)))) {
			struct st_lsm6dso32x_sensor *acc_sensor;
			u8 data = 0;

			acc_sensor = iio_priv(hw->iio_devs[ST_LSM6DSO32X_ID_ACC]);
			if (enable) {
				err = st_lsm6dso32x_get_batch_val(acc_sensor,
								  sensor->odr,
								  sensor->uodr,
								  &data);
				if (err < 0)
					goto out;
			}

			err = st_lsm6dso32x_update_bits_locked(hw,
				hw->odr_table_entry[ST_LSM6DSO32X_ID_ACC].batching_reg.addr,
				hw->odr_table_entry[ST_LSM6DSO32X_ID_ACC].batching_reg.mask,
				data);
			if (err < 0)
				goto out;
		}
		break;
	default:
		err = st_lsm6dso32x_sensor_set_enable(sensor, enable);
		if (err < 0)
			goto out;

		err = st_lsm6dso32x_set_sensor_batching_odr(sensor, enable);
		if (err < 0)
			goto out;
		break;
	}

	err = st_lsm6dso32x_update_watermark(sensor, sensor->watermark);
	if (err < 0)
		goto out;

	if (enable && hw->fifo_mode == ST_LSM6DSO32X_FIFO_BYPASS) {
		st_lsm6dso32x_reset_hwts(hw);
		err = st_lsm6dso32x_set_fifo_mode(hw, ST_LSM6DSO32X_FIFO_CONT);
	} else if (!hw->enable_mask) {
		err = st_lsm6dso32x_set_fifo_mode(hw, ST_LSM6DSO32X_FIFO_BYPASS);
	}

out:
	enable_irq(hw->irq);

	return err;
}

static irqreturn_t st_lsm6dso32x_handler_irq(int irq, void *private)
{
	struct st_lsm6dso32x_hw *hw = (struct st_lsm6dso32x_hw *)private;
	s64 ts = st_lsm6dso32x_get_time_ns();

	hw->delta_ts = ts - hw->ts;
	hw->ts = ts;

	return IRQ_WAKE_THREAD;
}

static irqreturn_t st_lsm6dso32x_handler_thread(int irq, void *private)
{
	struct st_lsm6dso32x_hw *hw = (struct st_lsm6dso32x_hw *)private;

#ifdef CONFIG_IIO_ST_LSM6DSO32X_MLC
	st_lsm6dso32x_mlc_check_status(hw);
#endif /* CONFIG_IIO_ST_LSM6DSO32X_MLC */

	mutex_lock(&hw->fifo_lock);
	st_lsm6dso32x_read_fifo(hw);
	clear_bit(ST_LSM6DSO32X_HW_FLUSH, &hw->state);
	mutex_unlock(&hw->fifo_lock);

	if (hw->enable_mask & (BIT(ST_LSM6DSO32X_ID_STEP_DETECTOR) |
			       BIT(ST_LSM6DSO32X_ID_TILT) |
			       BIT(ST_LSM6DSO32X_ID_SIGN_MOTION))) {
		struct iio_dev *iio_dev;
		u8 status[3];
		s64 event;
		int err;

		err = regmap_bulk_read(hw->regmap,
				       ST_LSM6DSO32X_REG_EMB_FUNC_STATUS_MAINPAGE,
				       status, sizeof(status));
		if (err < 0)
			goto out;

		/* embedded function sensors */
		if (status[0] & ST_LSM6DSO32X_REG_INT_STEP_DET_MASK) {
			iio_dev = hw->iio_devs[ST_LSM6DSO32X_ID_STEP_DETECTOR];
			event = IIO_UNMOD_EVENT_CODE(IIO_STEP_DETECTOR, -1,
						     IIO_EV_TYPE_THRESH,
						     IIO_EV_DIR_RISING);
			iio_push_event(iio_dev, event,
				       st_lsm6dso32x_get_time_ns());
		}

		if (status[0] & ST_LSM6DSO32X_REG_INT_SIGMOT_MASK) {
			iio_dev = hw->iio_devs[ST_LSM6DSO32X_ID_SIGN_MOTION];
			event = IIO_UNMOD_EVENT_CODE(IIO_SIGN_MOTION, -1,
						     IIO_EV_TYPE_THRESH,
						     IIO_EV_DIR_RISING);
			iio_push_event(iio_dev, event,
				       st_lsm6dso32x_get_time_ns());
		}

		if (status[0] & ST_LSM6DSO32X_REG_INT_TILT_MASK) {
			iio_dev = hw->iio_devs[ST_LSM6DSO32X_ID_TILT];
			event = IIO_UNMOD_EVENT_CODE(IIO_TILT, -1,
						     IIO_EV_TYPE_THRESH,
						     IIO_EV_DIR_RISING);
			iio_push_event(iio_dev, event,
				       st_lsm6dso32x_get_time_ns());
		}
	}

out:
	return IRQ_HANDLED;
}

static int st_lsm6dso32x_fifo_preenable(struct iio_dev *iio_dev)
{
	return st_lsm6dso32x_update_fifo(iio_dev, true);
}

static int st_lsm6dso32x_fifo_postdisable(struct iio_dev *iio_dev)
{
	return st_lsm6dso32x_update_fifo(iio_dev, false);
}

static const struct iio_buffer_setup_ops st_lsm6dso32x_fifo_ops = {
	.preenable = st_lsm6dso32x_fifo_preenable,
	.postdisable = st_lsm6dso32x_fifo_postdisable,
};

int st_lsm6dso32x_buffers_setup(struct st_lsm6dso32x_hw *hw)
{
	struct device_node *np = hw->dev->of_node;
	struct iio_buffer *buffer;
	unsigned long irq_type;
	bool irq_active_low;
	int i, err;

	irq_type = irqd_get_trigger_type(irq_get_irq_data(hw->irq));
	if (irq_type == IRQF_TRIGGER_NONE)
		irq_type = IRQF_TRIGGER_HIGH;

	switch (irq_type) {
	case IRQF_TRIGGER_HIGH:
	case IRQF_TRIGGER_RISING:
		irq_active_low = false;
		break;
	case IRQF_TRIGGER_LOW:
	case IRQF_TRIGGER_FALLING:
		irq_active_low = true;
		break;
	default:
		dev_info(hw->dev, "mode %lx unsupported\n", irq_type);
		return -EINVAL;
	}

	err = regmap_update_bits(hw->regmap, ST_LSM6DSO32X_REG_CTRL3_C_ADDR,
				 ST_LSM6DSO32X_REG_H_LACTIVE_MASK,
				 ST_LSM6DSO32X_SHIFT_VAL(irq_active_low,
				  ST_LSM6DSO32X_REG_H_LACTIVE_MASK));
	if (err < 0)
		return err;

	if (np && of_property_read_bool(np, "drive-open-drain")) {
		err = regmap_update_bits(hw->regmap,
					 ST_LSM6DSO32X_REG_CTRL3_C_ADDR,
					 ST_LSM6DSO32X_REG_PP_OD_MASK,
					 ST_LSM6DSO32X_SHIFT_VAL(1,
					  ST_LSM6DSO32X_REG_PP_OD_MASK));
		if (err < 0)
			return err;

		irq_type |= IRQF_SHARED;
	}

	err = devm_request_threaded_irq(hw->dev, hw->irq,
					st_lsm6dso32x_handler_irq,
					st_lsm6dso32x_handler_thread,
					irq_type | IRQF_ONESHOT,
					ST_LSM6DSO32X_DEV_NAME, hw);
	if (err) {
		dev_err(hw->dev, "failed to request trigger irq %d\n",
			hw->irq);
		return err;
	}

	for (i = ST_LSM6DSO32X_ID_GYRO;
	     i <= ST_LSM6DSO32X_ID_STEP_COUNTER;
	     i++) {
		if (!hw->iio_devs[i])
			continue;

		buffer = devm_iio_kfifo_allocate(hw->dev);
		if (!buffer)
			return -ENOMEM;

		iio_device_attach_buffer(hw->iio_devs[i], buffer);
		hw->iio_devs[i]->modes |= INDIO_BUFFER_SOFTWARE;
		hw->iio_devs[i]->setup_ops = &st_lsm6dso32x_fifo_ops;
	}

	return regmap_update_bits(hw->regmap,
				  ST_LSM6DSO32X_REG_FIFO_CTRL4_ADDR,
				  ST_LSM6DSO32X_REG_DEC_TS_MASK,
				  ST_LSM6DSO32X_SHIFT_VAL(1,
				   ST_LSM6DSO32X_REG_DEC_TS_MASK));
}

