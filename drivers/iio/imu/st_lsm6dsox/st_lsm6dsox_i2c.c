// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics st_lsm6dsox i2c driver
 *
 * Copyright 2021 STMicroelectronics Inc.
 *
 * Tesi Mario <mario.tesi@st.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/of.h>

#include "st_lsm6dsox.h"

static const struct regmap_config st_lsm6dsox_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int st_lsm6dsox_i2c_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client, &st_lsm6dsox_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "Failed to register i2c regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

 	return st_lsm6dsox_probe(&client->dev, client->irq, regmap);
}

static int st_lsm6dsox_i2c_remove(struct i2c_client *client)
{
	return st_lsm6dsox_mlc_remove(&client->dev);
}

static const struct of_device_id st_lsm6dsox_i2c_of_match[] = {
	{
		.compatible = "st," ST_LSM6DSOX_DEV_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, st_lsm6dsox_i2c_of_match);

static const struct i2c_device_id st_lsm6dsox_i2c_id_table[] = {
	{ ST_LSM6DSOX_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(i2c, st_lsm6dsox_i2c_id_table);

static struct i2c_driver st_lsm6dsox_driver = {
	.driver = {
		.name = "st_" ST_LSM6DSOX_DEV_NAME "_i2c",
		.pm = &st_lsm6dsox_pm_ops,
		.of_match_table = of_match_ptr(st_lsm6dsox_i2c_of_match),
	},
	.probe = st_lsm6dsox_i2c_probe,
	.remove = st_lsm6dsox_i2c_remove,
	.id_table = st_lsm6dsox_i2c_id_table,
};
module_i2c_driver(st_lsm6dsox_driver);

MODULE_AUTHOR("Mario Tesi <mario.tesi@st.com>");
MODULE_DESCRIPTION("STMicroelectronics st_lsm6dsox i2c driver");
MODULE_LICENSE("GPL v2");
