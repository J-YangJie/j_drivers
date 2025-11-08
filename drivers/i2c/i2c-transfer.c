/*
  I2C universal transfer driver 
  Copyright (C) 2022  YangJie <yangjie98765@yeah.net>

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along
  with this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/kref.h>
#include <linux/poll.h>
#include <linux/eventpoll.h>
#include <linux/cdev.h>
#include <linux/printk.h>
#include <linux/errno.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/i2c-dev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/mutex.h>
#include <linux/gpio/consumer.h>

#define NAME_DRIVER      "i2c-transfer"
#define NAME_CLASS_SLAVE "i2c-slave"

struct i2c_transfer_data {
	struct kref kref;
	dev_t devno;
	struct cdev cdev;
	struct device *device;
	struct i2c_client *client;
	wait_queue_head_t wait_read;
	atomic_t need_r;
	atomic_t s_removed;
	struct gpio_desc *gpio_wakeup;
};

static DEFINE_MUTEX(g_mutex_g_major);
static dev_t g_major;
static struct class *g_class;

enum {
	NEED_R_NO_NEED = 0,
	NEED_R_NEED    = 1,
	NEED_R_READING,
};

static irqreturn_t i2c_transfer_interrupt(int irq, void* dev_id)
{
	int ret;
	struct i2c_transfer_data *data = dev_id;

	if (atomic_read(&data->s_removed))
		return IRQ_NONE;

	ret = atomic_cmpxchg(&data->need_r, NEED_R_NO_NEED, NEED_R_NEED);
	if (NEED_R_NO_NEED == ret)
		wake_up_interruptible(&data->wait_read);

	return IRQ_HANDLED;
}

static void __i2c_transfer_data_free_delay(struct kref *kref)
{
	struct i2c_transfer_data *data = container_of(kref, 
						      struct i2c_transfer_data, kref);
	kfree(data);
}

static int i2c_transfer_open(struct inode *inode, struct file *filp)
{
	struct i2c_transfer_data *data = container_of(inode->i_cdev, 
						      struct i2c_transfer_data, cdev);

	filp->private_data = data;
	kref_get(&data->kref);
	return 0;
}

static int i2c_transfer_release(struct inode *inode, struct file *filp)
{
	struct i2c_transfer_data *data = filp->private_data;

	kref_put(&data->kref, __i2c_transfer_data_free_delay);
	return 0;
}

static int __i2c_transfer_wait_read(struct i2c_transfer_data* data)
{
	int ret = 0;
	DEFINE_WAIT(wait);

	for (;;) {
		if (atomic_read(&data->s_removed)) {
			ret = -ENODEV;
			break;
		}

		if (signal_pending(current)) {
			ret = -ERESTARTSYS;
			break;
		}

		prepare_to_wait_exclusive(&data->wait_read, &wait, TASK_INTERRUPTIBLE);

		if (NEED_R_NEED == atomic_cmpxchg(&data->need_r, NEED_R_NEED, 
						  NEED_R_READING)) {
			break;
		}

		schedule();
	}
	finish_wait(&data->wait_read, &wait);

	return ret;
}

static ssize_t i2c_transfer_read(struct file *filp, char __user *buf, 
				 size_t count, loff_t *ppos)
{
	int ret, need_r;
	u8 *kbuf;
	struct i2c_transfer_data *data = filp->private_data;
	bool nonblock = filp->f_flags & O_NONBLOCK;

	if (unlikely(count <= 0))
		return 0;

	kbuf = kmalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	need_r = atomic_cmpxchg(&data->need_r, NEED_R_NEED, NEED_R_READING);
	if (NEED_R_NO_NEED == need_r || NEED_R_READING == need_r) {
		if (nonblock) {
			ret = NEED_R_READING == need_r ? -EBUSY : -EAGAIN;
			goto err_only_free;
		}

		ret = __i2c_transfer_wait_read(data);
		if (ret) {
			goto err_only_free;
		}
	}

	ret = i2c_master_recv(data->client, kbuf, count);
	if (ret < 0)
		goto err;

	if (copy_to_user(buf, kbuf, ret)) {
		ret = -EFAULT;
		goto err;
	}

	need_r = data->client->irq > 0 ? NEED_R_NO_NEED : NEED_R_NEED;
	atomic_set(&data->need_r, need_r); /* atomic_cmpxchg(&data->need_r, NEED_R_READING, need_r); */
	if (NEED_R_NEED == need_r)
		wake_up_interruptible(&data->wait_read);

	kfree(kbuf);
	return ret;

err:
	atomic_set(&data->need_r, NEED_R_NEED); /* atomic_cmpxchg(&data->need_r, NEED_R_READING, NEED_R_NEED); */
	wake_up_interruptible(&data->wait_read);
err_only_free:
	kfree(kbuf);
	return ret;
}

static ssize_t i2c_transfer_write(struct file *filp, const char __user *buf, 
				  size_t count, loff_t *ppos)
{
	int ret;
	u8 *kbuf;
	struct i2c_transfer_data *data = filp->private_data;

	if (unlikely(count <= 0))
		return 0;

	kbuf = kmalloc(count, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	if (copy_from_user(kbuf, buf, count)) {
		ret = -EFAULT;
		goto err;
	}

	ret = i2c_master_send(data->client, kbuf, count);

err:
	kfree(kbuf);
	return ret;
}

static unsigned int i2c_transfer_poll(struct file *filp, struct poll_table_struct *wait)
{
	int ret;
	struct i2c_transfer_data *data = filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &data->wait_read, wait);

	if (atomic_read(&data->s_removed)) {
		mask |= POLLHUP | EPOLLHUP | POLLERR;
		goto end;
	}

	ret = atomic_read(&data->need_r);
	if (NEED_R_NEED == ret)
		mask |= POLLIN | EPOLLIN | POLLOUT | EPOLLOUT;
	else if (NEED_R_NO_NEED == ret)
		mask |= POLLOUT | EPOLLOUT;

end:
	return mask;
}

static int i2c_transfer_suspend(struct device *dev)
{
	struct i2c_transfer_data *data = dev_get_drvdata(dev);

	if (!data || atomic_read(&data->s_removed))
		return 0;

	if (data->client->irq > 0)
		disable_irq(data->client->irq);
	return 0;
}

static int i2c_transfer_resume(struct device *dev)
{
	struct i2c_transfer_data *data = dev_get_drvdata(dev);

	if (!data || atomic_read(&data->s_removed))
		return 0;

	if (data->client->irq > 0)
		enable_irq(data->client->irq);
	return 0;
}
static SIMPLE_DEV_PM_OPS(i2c_transfer_pm_ops, i2c_transfer_suspend, 
			 i2c_transfer_resume);

static const struct file_operations i2c_transfer_fops = {
	.owner          = THIS_MODULE,
	.read           = i2c_transfer_read,
	.write          = i2c_transfer_write,
	.open           = i2c_transfer_open,
	.release        = i2c_transfer_release,
	.poll           = i2c_transfer_poll,
};

static int __i2c_transfer_properties_set(struct i2c_transfer_data *data)
{
	int ret;
	struct device *dev = &data->client->dev;
	struct gpio_desc *gpio_wakeup;

	if (data->client->irq > 0)
		return 0;

	gpio_wakeup = gpiod_get_optional(dev, "wakeup", GPIOD_IN);
	if (IS_ERR_OR_NULL(gpio_wakeup)) {
		dev_dbg(dev, "No gpio for wakeup!\n");
		return gpio_wakeup ? PTR_ERR(gpio_wakeup) : 0;
	}

	ret = gpiod_to_irq(gpio_wakeup);
	if (ret <= 0) {
		dev_err(dev, "Failed to get irq number by gpio!\n");
		goto err;
	}
	data->client->irq = ret;
	data->gpio_wakeup = gpio_wakeup;

	dev_notice(dev, "Get irq number(%d) from gpio!\n", 
		   data->client->irq);
	return 0;

err:
	gpiod_put(gpio_wakeup);
	return ret;
}

static int __i2c_transfer_properties_put(struct i2c_transfer_data *data)
{
	if (!data->gpio_wakeup)
		return 0;

	gpiod_put(data->gpio_wakeup);
	data->gpio_wakeup = NULL;
	data->client->irq = 0;
	return 0;
}

static int i2c_transfer_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	struct i2c_transfer_data *data;
	struct device *dev = &client->dev;
	int minor = (client->adapter->nr << 8) | client->addr;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "Failed to alloc memory for private data!\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	kref_init(&data->kref);

	data->client = client;
	ret = __i2c_transfer_properties_set(data);
	if (ret < 0)
		goto err_properties;

	atomic_set(&data->s_removed, 0);
	atomic_set(&data->need_r, !client->irq);
	init_waitqueue_head(&data->wait_read);

	if (client->irq > 0) {
		ret = request_irq(client->irq, i2c_transfer_interrupt, 
				  IRQF_TRIGGER_FALLING, "i2c_transfer", data);
		if (ret) {
			dev_err(dev, "Failed to request irq(%d)!\n", 
				client->irq);
			goto err_irq;
		}
	}

	cdev_init(&data->cdev, &i2c_transfer_fops);

	mutex_lock(&g_mutex_g_major);
	if (!g_major) {
		ret = alloc_chrdev_region(&data->devno, minor, 1, "i2c-slave");
		if (ret) {
			dev_err(dev, "Failed to alloc cdev number!\n");
			mutex_unlock(&g_mutex_g_major);
			goto err_devno;
		}
		g_major = MAJOR(data->devno);
	} else {
		data->devno = MKDEV(g_major, minor);
		ret = register_chrdev_region(data->devno, 1, "i2c-slave");
		if (ret) {
			dev_err(dev, 
				"Failed to register cdev number(0x%x, 0x%x)!\n", 
				MAJOR(data->devno), MINOR(data->devno));
			mutex_unlock(&g_mutex_g_major);
			goto err_devno;
		}
	}
	mutex_unlock(&g_mutex_g_major);

	ret = cdev_add(&data->cdev, data->devno, 1);
	if (ret) {
		dev_err(dev, "Failed to add cdev(%x, %x) to kernel!\n", 
			MAJOR(data->devno), MINOR(data->devno));
		goto err_cdev_add;
	}

	data->device = device_create(g_class, NULL, data->devno, NULL, 
				     "i2c-slave-%03x", minor);
	if (IS_ERR(data->device)) {
		dev_err(dev, "Failed to create i2c-slave(0x%x, 0x%x) device!\n", 
			MAJOR(data->devno), MINOR(data->devno));
		ret = PTR_ERR(data->device);
		data->device = NULL;
		goto err_device_create;
	}

	i2c_set_clientdata(client, data);
	return 0;

err_device_create:
	cdev_del(&data->cdev);
err_cdev_add:
	unregister_chrdev_region(data->devno, 1);
err_devno:
	data->devno = 0;
	free_irq(client->irq, data);
err_irq:
	atomic_set(&data->s_removed, 1);
	atomic_set(&data->need_r, NEED_R_NO_NEED);
	__i2c_transfer_properties_put(data);
err_properties:
	data->client = NULL;
	kref_put(&data->kref, __i2c_transfer_data_free_delay);
err_alloc:
	return ret;
}

static int i2c_transfer_remove(struct i2c_client *client)
{
	struct i2c_transfer_data *data = i2c_get_clientdata(client);

	if (!data)
		return 0;

	atomic_set(&data->s_removed, 1);
	atomic_set(&data->need_r, NEED_R_NO_NEED);
	wake_up_all(&data->wait_read);

	if (client->irq > 0) {
		disable_irq(client->irq);
		free_irq(client->irq, data);
	}

	device_destroy(g_class, data->devno);
	cdev_del(&data->cdev);
	unregister_chrdev_region(data->devno, 1);
	__i2c_transfer_properties_put(data);

	kref_put(&data->kref, __i2c_transfer_data_free_delay);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id i2c_transfer_of_match[] = {
	{ .compatible = "universal,i2c-transfer", },
	{},
};
MODULE_DEVICE_TABLE(of, i2c_transfer_of_match);
#endif

static const struct i2c_device_id i2c_transfer_id[] = {
	{ NAME_DRIVER, 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, i2c_transfer_id);

static struct i2c_driver i2c_transfer_driver = {
	.driver = {
		.name  = NAME_DRIVER,
		.owner = THIS_MODULE,
		.of_match_table = i2c_transfer_of_match,
		.pm    = &i2c_transfer_pm_ops,
	},
	.probe  = i2c_transfer_probe,
	.remove = i2c_transfer_remove,
	.id_table = i2c_transfer_id,
};

static int __init i2c_transfer_init(void)
{
	g_major = 0;
	if (!g_class) {
		struct class *t_class = class_create(THIS_MODULE, 
						     NAME_CLASS_SLAVE);
		if (IS_ERR(t_class)) {
			pr_err("Failed to create class(%s)!\n", 
			       NAME_CLASS_SLAVE);
			return PTR_ERR(t_class);
		}
		g_class = t_class;
	}

	return i2c_add_driver(&i2c_transfer_driver);
}

static void __exit i2c_transfer_exit(void)
{
	i2c_del_driver(&i2c_transfer_driver);

	if (g_class)
		class_destroy(g_class);
	g_class = NULL;
	g_major = 0;
}

module_init(i2c_transfer_init);
module_exit(i2c_transfer_exit);

MODULE_AUTHOR("yangjie <yangjie98765@yeah.net>");
MODULE_DESCRIPTION("I2C universal transfer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");


