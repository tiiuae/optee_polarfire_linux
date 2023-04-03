// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for seL4 TEE OS communication
 *
 * Copyright (C) 2021 Unikie
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/atomic.h>

#define DT_COMM_CTRL		"ctrl"
#define DT_REE2TEE_NAME		"ree2tee"
#define DT_TEE2REE_NAME		"tee2ree"

/****  Common definitions with REE  ****/
#define COMM_MAGIC_TEE		0x87654321
#define COMM_MAGIC_REE		0xFEDCBA98

struct tee_comm_ctrl {
	uint32_t ree_magic;
	uint32_t tee_magic;
	int32_t head;
	int32_t tail;
};
/***************************************/

struct tee_comm_ch {
	struct tee_comm_ctrl *ctrl;
	size_t buf_len;
	char *buf;
};

struct sel4com {
	struct tee_comm_ch ree2tee;
	struct tee_comm_ch tee2ree;
};

static ssize_t diag_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	int pos = 0;
	struct sel4com *priv = dev_get_drvdata(dev);
	pos += sysfs_emit_at(buf, pos, "REE->TEE\n");
	pos += sysfs_emit_at(buf, pos, "ree_magic: 0x%x\n", priv->ree2tee.ctrl->ree_magic);
	pos += sysfs_emit_at(buf, pos, "tee_magic: 0x%x\n", priv->ree2tee.ctrl->tee_magic);
	pos += sysfs_emit_at(buf, pos, "head: %d\n", priv->ree2tee.ctrl->head);
	pos += sysfs_emit_at(buf, pos, "tail: %d\n", priv->ree2tee.ctrl->tail);

	pos += sysfs_emit_at(buf, pos, "TEE->REE\n");
	pos += sysfs_emit_at(buf, pos, "ree_magic: 0x%x\n", priv->tee2ree.ctrl->ree_magic);
	pos += sysfs_emit_at(buf, pos, "tee_magic: 0x%x\n", priv->tee2ree.ctrl->tee_magic);
	pos += sysfs_emit_at(buf, pos, "head: %d\n", priv->tee2ree.ctrl->head);
	pos += sysfs_emit_at(buf, pos, "tail: %d\n", priv->tee2ree.ctrl->tail);

	return pos;
}
static DEVICE_ATTR_RO(diag);

static struct attribute *sel4com_sysfs_attrs[] = {
	&dev_attr_diag.attr,
	NULL
};

static struct attribute_group sel4com_attribute_group = {
	.attrs = sel4com_sysfs_attrs
};

static int sel4com_dt_config(struct device *dev, const char *dt_name,
			     void **vaddr, size_t *len)
{
	int err = -1;
	struct device_node *np = dev_of_node(dev);
	struct device_node *dt_ch = NULL;
	struct resource res = { 0 };
	void *ch_base = NULL;

	dt_ch = of_get_child_by_name(np, dt_name);
	if (!dt_ch) {
		dev_err(dev, "channel %s not found", dt_name);
		return -ENXIO;
	}

	err = of_address_to_resource(dt_ch, 0, &res);
	if (err) {
		dev_err(dev, "%s no memory area defined", dt_name);
		return err;
	}

	ch_base = devm_ioremap_resource(dev, &res);
	err = PTR_ERR_OR_ZERO(ch_base);
	if (err) {
		dev_err(dev, "%s memory map failed %d", dt_name, err);
		return err;
	}

	*vaddr = ch_base;
	*len = resource_size(&res);

	dev_info(dev, "%s %pr", dt_name, &res);

	return err;
}

static int sel4com_ch_setup(struct device *dev, struct sel4com *priv)
{
	int err = -1;
	void *base = 0;
	size_t len = 0;

	err = sel4com_dt_config(dev, DT_COMM_CTRL, &base, &len);
	if (err)
		return err;

	if (len < (sizeof(struct tee_comm_ctrl) * 2)) {
		dev_err(dev, "ctrl buffer too small %ld", len);
		return -ENOBUFS;
	}

	/* First area is for REE->TEE buffer control data and
	 * second is for TEE->REE.
	 */
	priv->ree2tee.ctrl = (struct tee_comm_ctrl *)base;
	base += sizeof(struct tee_comm_ctrl);
	priv->tee2ree.ctrl = (struct tee_comm_ctrl *)base;

	err = sel4com_dt_config(dev, DT_REE2TEE_NAME, &base, &len);
	if (err)
		return err;

	priv->ree2tee.buf = (char *)base;
	priv->ree2tee.buf_len = len;

	err = sel4com_dt_config(dev, DT_TEE2REE_NAME, &base, &len);
	if (err)
		return err;

	priv->tee2ree.buf = (char *)base;
	priv->tee2ree.buf_len = len;

	return err;
}

static void sel4com_init_ch(struct sel4com *priv)
{
	/* writer initializes the channel */
	memset(priv->ree2tee.buf, 0x0, priv->ree2tee.buf_len);

	priv->ree2tee.ctrl->head = 0;
	priv->ree2tee.ctrl->tail = 0;

	/* Writing magic indicates channel setup is ready. SMP release
	 * semantics to ensure init operations are done prior.
	 */
	smp_store_release(&priv->tee2ree.ctrl->ree_magic, COMM_MAGIC_REE);
	priv->ree2tee.ctrl->ree_magic = COMM_MAGIC_REE;
}

static int sel4com_probe(struct platform_device *pdev)
{
	int err = -1;
	struct sel4com *priv = NULL;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	err = sel4com_ch_setup(&pdev->dev, priv);
	if (err)
		return err;

	sel4com_init_ch(priv);

	err = devm_device_add_group(&pdev->dev, &sel4com_attribute_group);
	if (err) {
		dev_err(&pdev->dev, "sysfs creation failed %d", err);
		return err;
	}

	return err;
}

static int sel4com_remove(struct platform_device *pdev)
{
	int err = 0;

	return err;
}

static const struct of_device_id sel4com_of_match[] = {
	{
		.compatible = "sel4,teeos-comm",
	},
	{}
};
MODULE_DEVICE_TABLE(of, sel4com_of_match);

static struct platform_driver sel4com_driver = {
	.driver = {
		.name = "sel4-comm",
		.of_match_table = sel4com_of_match,
	},
	.probe = sel4com_probe,
	.remove = sel4com_remove,
};

module_platform_driver(sel4com_driver);

MODULE_AUTHOR("");
MODULE_DESCRIPTION("seL4 TEE OS communication driver");
MODULE_LICENSE("GPL v2");
