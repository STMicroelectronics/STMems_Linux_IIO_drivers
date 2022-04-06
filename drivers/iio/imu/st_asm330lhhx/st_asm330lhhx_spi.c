/*
 * STMicroelectronics st_asm330lhhx spi driver
 *
 * Copyright 2022 STMicroelectronics Inc.
 *
 * Tesi Mario <mario.tesi@st.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/of.h>

#include "st_asm330lhhx.h"

static const struct regmap_config st_asm330lhhx_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int st_asm330lhhx_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_spi(spi, &st_asm330lhhx_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap %d\n",
			(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return st_asm330lhhx_probe(&spi->dev, spi->irq, regmap);
}

static int st_asm330lhhx_spi_remove(struct spi_device *spi)
{
#ifdef CONFIG_IIO_ST_ASM330LHHX_MLC
	return st_asm330lhhx_mlc_remove(&spi->dev);
#else /* CONFIG_IIO_ST_ASM330LHHX_MLC */
	return 0;
#endif /* CONFIG_IIO_ST_ASM330LHHX_MLC */
}

static const struct of_device_id st_asm330lhhx_spi_of_match[] = {
	{
		.compatible = "st," ST_ASM330LHHX_DEV_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(of, st_asm330lhhx_spi_of_match);

static const struct spi_device_id st_asm330lhhx_spi_id_table[] = {
	{ ST_ASM330LHHX_DEV_NAME },
	{},
};
MODULE_DEVICE_TABLE(spi, st_asm330lhhx_spi_id_table);

static struct spi_driver st_asm330lhhx_driver = {
	.driver = {
		.name = "st_" ST_ASM330LHHX_DEV_NAME "_spi",
		.pm = &st_asm330lhhx_pm_ops,
		.of_match_table = of_match_ptr(st_asm330lhhx_spi_of_match),
	},
	.probe = st_asm330lhhx_spi_probe,
	.remove = st_asm330lhhx_spi_remove,
	.id_table = st_asm330lhhx_spi_id_table,
};
module_spi_driver(st_asm330lhhx_driver);

MODULE_AUTHOR("Mario Tesi <mario.tesi@st.com>");
MODULE_DESCRIPTION("STMicroelectronics st_asm330lhhx spi driver");
MODULE_LICENSE("GPL v2");
