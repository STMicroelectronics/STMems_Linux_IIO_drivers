/*
 * STMicroelectronics st_lsm6dso32x embedded function sensor driver
 *
 * Copyright 2021 STMicroelectronics Inc.
 *
 * Mario Tesi <mario.tesi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/iio/iio.h>

#include "st_lsm6dso32x.h"

static int
st_lsm6dso32x_ef_pg1_sensor_set_enable(struct st_lsm6dso32x_sensor *sensor,
				       u8 mask, u8 irq_mask, bool enable)
{
	struct st_lsm6dso32x_hw *hw = sensor->hw;
	int err;

	err = st_lsm6dso32x_sensor_set_enable(sensor, enable);
	if (err < 0)
		return err;

	mutex_lock(&hw->page_lock);
	err = st_lsm6dso32x_set_page_access(hw, true,
					    ST_LSM6DSO32X_REG_FUNC_CFG_MASK);
	if (err < 0)
		goto unlock;

	err = __st_lsm6dso32x_write_with_mask(hw,
					      ST_LSM6DSO32X_EMB_FUNC_EN_A_ADDR,
					      mask, enable);
	if (err < 0)
		goto reset_page;

	err = __st_lsm6dso32x_write_with_mask(hw, hw->embfunc_irq_reg, irq_mask,
					      enable);

reset_page:
	st_lsm6dso32x_set_page_access(hw, false,
				      ST_LSM6DSO32X_REG_FUNC_CFG_MASK);
unlock:
	mutex_unlock(&hw->page_lock);

	return err;
}

/**
 * Enable Embedded Function sensor [EMB_FUN]
 *
 * @param  sensor: ST IMU sensor instance
 * @param  enable: Enable/Disable sensor
 * @return  < 0 if error, 0 otherwise
 */
int st_lsm6dso32x_embfunc_sensor_set_enable(
				struct st_lsm6dso32x_sensor *sensor,
				bool enable)
{
	int err;

	switch (sensor->id) {
	case ST_LSM6DSO32X_ID_STEP_DETECTOR:
		err = st_lsm6dso32x_ef_pg1_sensor_set_enable(sensor,
					ST_LSM6DSO32X_PEDO_EN_MASK,
					ST_LSM6DSO32X_INT_STEP_DET_MASK,
					enable);
		break;
	case ST_LSM6DSO32X_ID_SIGN_MOTION:
		err = st_lsm6dso32x_ef_pg1_sensor_set_enable(sensor,
					ST_LSM6DSO32X_SIGN_MOTION_EN_MASK,
					ST_LSM6DSO32X_INT_SIGMOT_MASK,
					enable);
		break;
	case ST_LSM6DSO32X_ID_TILT:
		err = st_lsm6dso32x_ef_pg1_sensor_set_enable(sensor,
						ST_LSM6DSO32X_TILT_EN_MASK,
						ST_LSM6DSO32X_INT_TILT_MASK,
						enable);
		break;
	default:
		err = -EINVAL;
		break;
	}

	return err;
}

/**
 * Enable Step Counter Sensor [EMB_FUN]
 *
 * @param  sensor: ST IMU sensor instance
 * @param  enable: Enable/Disable sensor
 * @return  < 0 if error, 0 otherwise
 */
int st_lsm6dso32x_step_counter_set_enable(struct st_lsm6dso32x_sensor *sensor,
					  bool enable)
{
	struct st_lsm6dso32x_hw *hw = sensor->hw;
	int err;

	err = st_lsm6dso32x_sensor_set_enable(sensor, enable);
	if (err < 0)
		return err;

	mutex_lock(&hw->page_lock);
	err = st_lsm6dso32x_set_page_access(hw, true,
					    ST_LSM6DSO32X_REG_FUNC_CFG_MASK);
	if (err < 0)
		goto unlock;

	err = __st_lsm6dso32x_write_with_mask(hw,
					      ST_LSM6DSO32X_EMB_FUNC_EN_A_ADDR,
					      ST_LSM6DSO32X_PEDO_EN_MASK,
					      enable);
	if (err < 0)
		goto reset_page;

	err = __st_lsm6dso32x_write_with_mask(hw,
					ST_LSM6DSO32X_EMB_FUNC_FIFO_CFG_ADDR,
					ST_LSM6DSO32X_PEDO_FIFO_EN_MASK,
					enable);

reset_page:
	st_lsm6dso32x_set_page_access(hw, false,
				      ST_LSM6DSO32X_REG_FUNC_CFG_MASK);
unlock:
	mutex_unlock(&hw->page_lock);

	return err;
}

/**
 * Reset Step Counter value [EMB_FUN]
 *
 * @param  iio_dev: IIO device
 * @return  < 0 if error, 0 otherwise
 */
int st_lsm6dso32x_reset_step_counter(struct iio_dev *iio_dev)
{
	struct st_lsm6dso32x_sensor *sensor = iio_priv(iio_dev);
	struct st_lsm6dso32x_hw *hw = sensor->hw;
	u16 prev_val, val = 0;
	__le16 data;
	int err;

	mutex_lock(&iio_dev->mlock);
	if (iio_buffer_enabled(iio_dev)) {
		err = -EBUSY;
		goto unlock_iio_dev;
	}

	err = st_lsm6dso32x_step_counter_set_enable(sensor, true);
	if (err < 0)
		goto unlock_iio_dev;

	mutex_lock(&hw->page_lock);
	err = st_lsm6dso32x_set_page_access(hw, true,
					    ST_LSM6DSO32X_REG_FUNC_CFG_MASK);
	if (err < 0)
		goto unlock_page;

	do {
		prev_val = val;
		err = __st_lsm6dso32x_write_with_mask(hw,
					ST_LSM6DSO32X_REG_EMB_FUNC_SRC_ADDR,
					ST_LSM6DSO32X_REG_PEDO_RST_STEP_MASK,
					1);
		if (err < 0)
			goto reset_page;

		msleep(100);

		err = regmap_bulk_read(hw->regmap,
				       ST_LSM6DSO32X_REG_STEP_COUNTER_L_ADDR,
				       (u8 *)&data, sizeof(data));
		if (err < 0)
			goto reset_page;

		val = le16_to_cpu(data);
	} while (val && val >= prev_val);

reset_page:
	st_lsm6dso32x_set_page_access(hw, false,
				      ST_LSM6DSO32X_REG_FUNC_CFG_MASK);
unlock_page:
	mutex_unlock(&hw->page_lock);

	err = st_lsm6dso32x_step_counter_set_enable(sensor, false);
unlock_iio_dev:
	mutex_unlock(&iio_dev->mlock);

	return err;
}

/**
 * Initialize Embedded funcrtion HW block [EMB_FUN]
 *
 * @param  hw: ST IMU MEMS hw instance
 * @return  < 0 if error, 0 otherwise
 */
int st_lsm6dso32x_embedded_function_init(struct st_lsm6dso32x_hw *hw)
{
	int err;

	mutex_lock(&hw->page_lock);
	err = st_lsm6dso32x_set_page_access(hw, true,
					    ST_LSM6DSO32X_REG_FUNC_CFG_MASK);
	if (err < 0)
		goto unlock;

	/* enable latched interrupts */
	err  = __st_lsm6dso32x_write_with_mask(hw, ST_LSM6DSO32X_PAGE_RW_ADDR,
					ST_LSM6DSO32X_REG_EMB_FUNC_LIR_MASK,
					1);

	st_lsm6dso32x_set_page_access(hw, false,
				      ST_LSM6DSO32X_REG_FUNC_CFG_MASK);
unlock:
	mutex_unlock(&hw->page_lock);

	return err;
}

