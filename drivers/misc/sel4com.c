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
#include <linux/spinlock.h>
#include <linux/circ_buf.h>
#include <linux/minmax.h>

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

struct tee_comm_ch {
	struct tee_comm_ctrl *ctrl;
	int32_t buf_len;
	char *buf;
};
/***************************************/

struct sel4com {
	struct tee_comm_ch ree2tee;
	struct tee_comm_ch tee2ree;
};

static DEFINE_SPINLOCK(writer_lock);
static DEFINE_SPINLOCK(reader_lock);

/*
 * Design copied from producer example: linux/Documentation/core-api/circular-buffers.rst
 */
static int write_to_circ(struct tee_comm_ch *circ, int32_t data_len,
			 const char *data_in)
{
	int ret = -ENOSPC;
	int32_t head = 0;
	int32_t tail = 0;
	int32_t buf_end = 0;
	int32_t write_ph1 = 0;
	int32_t wrap = 0;

	spin_lock(&writer_lock);

	head = circ->ctrl->head;

	/* The spin_unlock() and next spin_lock() provide needed ordering. */
	tail = READ_ONCE(circ->ctrl->tail);

	/* Shrink consecutive writes to the buffer end */
	buf_end = CIRC_SPACE_TO_END(head, tail, circ->buf_len);
	write_ph1 = min(buf_end, data_len);

	/* Remaining data if wrap needed, otherwise zero */
	wrap = data_len - write_ph1;

	if (CIRC_SPACE(head, tail, circ->buf_len) >= data_len) {
		memcpy(&circ->buf[head], data_in, write_ph1);

		/* Head will be automatically rolled back to the beginning of the buffer */
		head = (head + write_ph1) & (circ->buf_len - 1);

		if (wrap) {
			memcpy(&circ->buf[head], &data_in[write_ph1], wrap);
			head = (head + wrap) & (circ->buf_len - 1);
		}

		/* update the head after buffer write */
		smp_store_release(&circ->ctrl->head, head);

		/* TODO: wakeup reader */
		ret = 0;
	}

	spin_unlock(&writer_lock);

	return ret;
}

/*
 * Design copied from consumer example: linux/Documentation/core-api/circular-buffers.rst
 */
static int read_from_circ(struct tee_comm_ch *circ, int32_t out_len,
			  char *out_buf, int32_t *read_len)
{
	int ret = -ENODATA;
	int32_t head = 0;
	int32_t tail = 0;
	int32_t available = 0;
	int32_t buf_end = 0;
	int32_t read_ph1 = 0;
	int32_t wrap = 0;

	spin_lock(&reader_lock);

	/* Read index before reading contents at that index. */
	head = smp_load_acquire(&circ->ctrl->head);
	tail = circ->ctrl->tail;

	/* Shrink read length to output buffer size */
	available = min(out_len, CIRC_CNT(head, tail, circ->buf_len));

	/* Shrink consecutive reads to the buffer end */
	buf_end = CIRC_CNT_TO_END(head, tail, circ->buf_len);
	read_ph1 = min(available, buf_end);

	/* Remaining data if wrap needed, otherwise zero */
	wrap = available - read_ph1;

	*read_len = 0;

	if (available >= 1) {
		memcpy(out_buf, &circ->buf[tail], read_ph1);
		tail = (tail + read_ph1) & (circ->buf_len - 1);

		*read_len = read_ph1;

		if (wrap) {
			memcpy(&out_buf[read_ph1], &circ->buf[tail], wrap);
			tail = (tail + wrap) & (circ->buf_len - 1);
			*read_len += wrap;
		}

		/* Finish reading descriptor before incrementing tail. */
		smp_store_release(&circ->ctrl->tail, tail);

		ret = 0;
	}

	spin_unlock(&reader_lock);

	return ret;
}

static ssize_t diag_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	int pos = 0;
	struct sel4com *priv = dev_get_drvdata(dev);

	pos += sysfs_emit_at(buf, pos, "\nREE->TEE\n");
	pos += sysfs_emit_at(buf, pos, "ree_magic: 0x%x\n", priv->ree2tee.ctrl->ree_magic);
	pos += sysfs_emit_at(buf, pos, "tee_magic: 0x%x\n", priv->ree2tee.ctrl->tee_magic);
	pos += sysfs_emit_at(buf, pos, "head: %d\n", priv->ree2tee.ctrl->head);
	pos += sysfs_emit_at(buf, pos, "tail: %d\n", priv->ree2tee.ctrl->tail);

	pos += sysfs_emit_at(buf, pos, "\nTEE->REE\n");
	pos += sysfs_emit_at(buf, pos, "ree_magic: 0x%x\n", priv->tee2ree.ctrl->ree_magic);
	pos += sysfs_emit_at(buf, pos, "tee_magic: 0x%x\n", priv->tee2ree.ctrl->tee_magic);
	pos += sysfs_emit_at(buf, pos, "head: %d\n", priv->tee2ree.ctrl->head);
	pos += sysfs_emit_at(buf, pos, "tail: %d\n", priv->tee2ree.ctrl->tail);

	return pos;
}
static DEVICE_ATTR_RO(diag);

static ssize_t write_tee_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	int err = -1;
	struct sel4com *priv = dev_get_drvdata(dev);

	dev_info(dev, "write %ld", count);

	err = write_to_circ(&priv->ree2tee, count, buf);
	if (err)
		count = err;

	return count;
}

static DEVICE_ATTR_WO(write_tee);

static ssize_t read_tee_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	int err = -1;
	int32_t pos = 0;
	struct sel4com *priv = dev_get_drvdata(dev);

	err = read_from_circ(&priv->tee2ree, PAGE_SIZE, buf, &pos);
	if (err)
		return err;

	dev_info(dev, "read %d", pos);

	return pos;
}
static DEVICE_ATTR_RO(read_tee);

static struct attribute *sel4com_sysfs_attrs[] = {
	&dev_attr_diag.attr,
	&dev_attr_write_tee.attr,
	&dev_attr_read_tee.attr,
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
