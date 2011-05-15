/* drivers/input/touchscreen/himax8250.c
 *
 * Copyright (C) 2011 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/himax8250.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <mach/msm_hsusb.h>
#include <mach/board.h>

#define HIMAX_I2C_RETRY_TIMES 10

struct himax_ts_data {
	int use_irq;
	struct workqueue_struct *himax_wq;
	struct input_dev *input_dev;
	int fw_ver;
	struct hrtimer timer;
	struct work_struct work;
	struct i2c_client *client;
	uint8_t debug_log_level;
	int (*power)(int on);
	struct early_suspend early_suspend;
	uint8_t *command_b3;
	uint8_t *command_c2;
	uint8_t *cable_config;
};
static struct himax_ts_data *private_ts;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void himax_ts_early_suspend(struct early_suspend *h);
static void himax_ts_late_resume(struct early_suspend *h);
#endif


int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length)
{
	int retry;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = data,
		}
	};

	for (retry = 0; retry < HIMAX_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			break;
		mdelay(10);
	}
	if (retry == HIMAX_I2C_RETRY_TIMES) {
		printk(KERN_INFO "i2c_read_block retry over %d\n",
			HIMAX_I2C_RETRY_TIMES);
		return -EIO;
	}
	return 0;

}


int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length)
{
	int retry, loop_i;
	uint8_t buf[length + 1];

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = length + 1,
			.buf = buf,
		}
	};

	buf[0] = command;
	for (loop_i = 0; loop_i < length; loop_i++)
		buf[loop_i + 1] = data[loop_i];

	for (retry = 0; retry < HIMAX_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		mdelay(10);
	}

	if (retry == HIMAX_I2C_RETRY_TIMES) {
		printk(KERN_ERR "i2c_write_block retry over %d\n",
			HIMAX_I2C_RETRY_TIMES);
		return -EIO;
	}
	return 0;

}

int i2c_himax_write_command(struct i2c_client *client, uint8_t command)
{
	return i2c_himax_write(client, command, NULL, 0);
}

static uint8_t himax_reg_addr;

static ssize_t himax_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	uint8_t ptr[1] = { 0 };
	struct himax_ts_data *ts_data;
	ts_data = private_ts;
	if (i2c_himax_read(ts_data->client, himax_reg_addr, ptr, 1) < 0) {
		printk(KERN_WARNING "%s: read fail\n", __func__);
		return ret;
	}
	ret += sprintf(buf, "addr: %d, data: %d\n", himax_reg_addr, ptr[0]);
	return ret;
}

static ssize_t himax_register_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct himax_ts_data *ts_data;
	char buf_tmp[4], length = 0;
	uint8_t write_da[100], command;

	ts_data = private_ts;
	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(write_da, 0x0, sizeof(write_da));

	if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':') {
		if (buf[2] == 'x') {
			uint8_t loop_i, base = 5;
			memcpy(buf_tmp, buf + 3, 2);
			command = simple_strtol(buf_tmp, NULL, 16);
			length++;
			for (loop_i = 0; loop_i < 100; loop_i++) {
				if (buf[base] == '\n') {
					i2c_himax_write(ts_data->client, command,
						&write_da[0], length);
					printk("CMD: %x, %x, %d\n", command,
						write_da[0], length);
					return count;
				}
				if (buf[base + 1] == 'x') {
					memcpy(buf_tmp, buf + base + 2, 2);
					write_da[loop_i] = simple_strtol(buf_tmp, NULL, 16);
					length++;
				}
				base += 4;
			}
		}
	}
	return count;
}

static DEVICE_ATTR(register, (S_IWUSR|S_IRUGO),
	himax_register_show, himax_register_store);


static ssize_t touch_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct himax_ts_data *ts_data;
	ts_data = private_ts;

	sprintf(buf, "%s_%#x\n", HIMAX8250_NAME, ts_data->fw_ver);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(vendor, 0444, touch_vendor_show, NULL);


static ssize_t himax_debug_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct himax_ts_data *ts_data;
	size_t count = 0;
	ts_data = private_ts;

	count += sprintf(buf, "%d\n", ts_data->debug_log_level);

	return count;
}

static ssize_t himax_debug_level_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct himax_ts_data *ts_data;
	ts_data = private_ts;
	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		ts_data->debug_log_level = buf[0] - '0';

	return count;
}

static DEVICE_ATTR(debug_level, (S_IWUSR|S_IRUGO),
	himax_debug_level_show, himax_debug_level_dump);

static struct kobject *android_touch_kobj;

static int himax_touch_sysfs_init(void)
{
	int ret;
	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr);
	if (ret) {
		printk(KERN_ERR "TOUCH_ERR: create_file debug_level failed\n");
		return ret;
	}
	himax_reg_addr = 0;
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_register.attr);
	if (ret) {
		printk(KERN_ERR "TOUCH_ERR: create_file register failed\n");
		return ret;
	}

	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}

	return 0 ;
}

static void himax_touch_sysfs_deinit(void)
{
/*	sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);*/
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_register.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
	kobject_del(android_touch_kobj);
}

static void himax_ts_work_func(struct work_struct *work)
{
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data, work);
	uint8_t buf[24], loop_i, finger_num, finger_pressed;

	i2c_himax_read(ts->client, 0x86, buf, 24);

	if (ts->debug_log_level & 0x1) {
		for (loop_i = 0; loop_i < 24; loop_i++) {
			printk("0x%2.2X ", buf[loop_i]);
			if (loop_i % 8 == 7)
				printk("\n");
		}
	}
	if (buf[20] == 0xFF && buf[21] == 0xFF) {
		/* finger leave */
#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
#else
		input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, 0);
		input_report_abs(ts->input_dev, ABS_MT_POSITION, 1 << 31);
#endif
		if (ts->debug_log_level & 0x2)
			printk(KERN_INFO "Finger leave\n");
	} else {
		finger_num = buf[20] & 0x0F;
		finger_pressed = buf[21];
		for (loop_i = 0; loop_i < 4; loop_i++) {
			if (((finger_pressed >> loop_i) & 1) == 1) {
				int base = loop_i * 4;
				int x = buf[base] << 8 | buf[base + 1];
				int y = buf[base + 2] << 8 | buf[base + 3];
				int w = buf[16 + loop_i];
				finger_num--;
#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
				input_mt_sync(ts->input_dev);
#else
				input_report_abs(ts->input_dev, ABS_MT_AMPLITUDE, w << 16 | w);
				input_report_abs(ts->input_dev, ABS_MT_POSITION,
					((finger_num ==  0) ? BIT(31) : 0) | x << 16 | y);
#endif
				if (ts->debug_log_level & 0x2)
					printk(KERN_INFO "Finger %d=> X:%d, Y:%d w:%d, z:%d\n",
						loop_i + 1, x, y, w, w);
				}
			}
	}
#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
	input_sync(ts->input_dev);
#endif
	enable_irq(ts->client->irq);
}

static enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer)
{
	struct himax_ts_data *ts;

	ts = container_of(timer, struct himax_ts_data, timer);
	queue_work(ts->himax_wq, &ts->work);
	return HRTIMER_NORESTART;
}
static irqreturn_t himax_ts_irq_handler(int irq, void *dev_id)
{
	struct himax_ts_data *ts = dev_id;

	disable_irq_nosync(ts->client->irq);
	queue_work(ts->himax_wq, &ts->work);
	return IRQ_HANDLED;
}

static void himax_cable_tp_status_handler_func(int connect_status)
{
	struct himax_ts_data *ts;
	uint8_t buf[3];
	printk(KERN_INFO "Touch: cable change to %d\n", connect_status);
	ts = private_ts;
	if (ts->cable_config[0]) {
		if (connect_status)
			ts->cable_config[1] = ts->cable_config[1] | 0x10;
		else
			ts->cable_config[1] = ts->cable_config[1] & 0xEF;
		i2c_master_send(ts->client, ts->cable_config, sizeof(ts->cable_config));
	}
	i2c_himax_read(ts->client, 0xF2, buf, 1);
	printk(KERN_INFO "F2: %x\n", buf[0]);
}

static struct t_usb_status_notifier himax_cable_status_handler = {
	.name = "usb_tp_connected",
	.func = himax_cable_tp_status_handler_func,
};


static int himax8250_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0, err = 0;
	struct himax_ts_data *ts;
	struct himax_i2c_platform_data *pdata;
	uint8_t data[5];

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "%s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct himax_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "%s: allocate himax_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;
	if (pdata)
		ts->power = pdata->power;
	if (ts->power) {
		ret = ts->power(1);
		msleep(10);
		if (ret < 0) {
			dev_err(&client->dev, "power on failed\n");
			goto err_power_failed;
		}
	}

	ret = i2c_himax_write_command(ts->client, 0x81);
	if (ret < 0) {
		printk(KERN_INFO "No Himax chip inside\n");
		goto err_detect_failed;
	}
	msleep(120);

	i2c_master_send(ts->client, pdata->command_76, sizeof(pdata->command_76));
	i2c_master_send(ts->client, pdata->command_6e, sizeof(pdata->command_6e));
	i2c_master_send(ts->client, pdata->command_39, sizeof(pdata->command_39));
	i2c_master_send(ts->client, pdata->command_37, sizeof(pdata->command_37));
	i2c_master_send(ts->client, pdata->command_7d, sizeof(pdata->command_7d));
	i2c_master_send(ts->client, pdata->command_c2, sizeof(pdata->command_c2));
	i2c_master_send(ts->client, pdata->command_7f, sizeof(pdata->command_7f));
	i2c_master_send(ts->client, pdata->command_c0, sizeof(pdata->command_c0));
	i2c_master_send(ts->client, pdata->command_62, sizeof(pdata->command_62));
	i2c_master_send(ts->client, pdata->command_63, sizeof(pdata->command_63));
	i2c_master_send(ts->client, pdata->command_64, sizeof(pdata->command_64));
	i2c_master_send(ts->client, pdata->command_65, sizeof(pdata->command_65));
	i2c_master_send(ts->client, pdata->command_66, sizeof(pdata->command_66));
	i2c_master_send(ts->client, pdata->command_67, sizeof(pdata->command_67));
	i2c_master_send(ts->client, pdata->command_68, sizeof(pdata->command_68));
	i2c_master_send(ts->client, pdata->command_69, sizeof(pdata->command_69));
	i2c_master_send(ts->client, pdata->command_6a, sizeof(pdata->command_6a));
	i2c_master_send(ts->client, pdata->command_6b, sizeof(pdata->command_6b));
	i2c_master_send(ts->client, pdata->command_6c, sizeof(pdata->command_6c));
	i2c_master_send(ts->client, pdata->command_6d, sizeof(pdata->command_6d));
	i2c_master_send(ts->client, pdata->command_c9, sizeof(pdata->command_c9));
	i2c_master_send(ts->client, pdata->command_cb, sizeof(pdata->command_cb));
	i2c_master_send(ts->client, pdata->command_d4, sizeof(pdata->command_d4));
	i2c_master_send(ts->client, pdata->command_b2, sizeof(pdata->command_b2));
	i2c_master_send(ts->client, pdata->command_b3, sizeof(pdata->command_b3));
	i2c_master_send(ts->client, pdata->command_c5, sizeof(pdata->command_c5));
	i2c_master_send(ts->client, pdata->command_c6, sizeof(pdata->command_c6));
	i2c_master_send(ts->client, pdata->command_7a, sizeof(pdata->command_7a));
	i2c_master_send(ts->client, pdata->command_78, sizeof(pdata->command_78));
	i2c_master_send(ts->client, pdata->command_3a, sizeof(pdata->command_3a));
	i2c_master_send(ts->client, pdata->command_e9, sizeof(pdata->command_e9));
	i2c_master_send(ts->client, pdata->command_ea, sizeof(pdata->command_ea));
	i2c_master_send(ts->client, pdata->command_eb, sizeof(pdata->command_eb));
	i2c_master_send(ts->client, pdata->command_ec, sizeof(pdata->command_ec));
	i2c_master_send(ts->client, pdata->command_ee, sizeof(pdata->command_ee));
	i2c_master_send(ts->client, pdata->command_ed, sizeof(pdata->command_ed));
	i2c_master_send(ts->client, pdata->command_ef, sizeof(pdata->command_ef));
	i2c_master_send(ts->client, pdata->command_f0, sizeof(pdata->command_f0));
	i2c_master_send(ts->client, pdata->command_f1, sizeof(pdata->command_f1));
	i2c_master_send(ts->client, pdata->command_f2, sizeof(pdata->command_f2));
	i2c_master_send(ts->client, pdata->command_f3, sizeof(pdata->command_f3));
	i2c_master_send(ts->client, pdata->command_f4, sizeof(pdata->command_f4));
	i2c_master_send(ts->client, pdata->command_f7, sizeof(pdata->command_f7));
	i2c_master_send(ts->client, pdata->command_e1, sizeof(pdata->command_e1));

	msleep(10);
	data[0] = 0x02;
	i2c_himax_write(ts->client, 0x35, &data[0], 2);
	mdelay(1);

	data[0] = 0x01;
	i2c_himax_write(ts->client, 0x36, &data[0], 1);

	mdelay(1);

	i2c_himax_write_command(ts->client, 0x83);

	msleep(100);
	i2c_himax_read(ts->client, 0x31, data, 3);
	i2c_himax_read(ts->client, 0x32, &data[3], 1);
	printk(KERN_INFO "0x31=> 0x%2.2X 0x%2.2X 0x%2.2X 0x%2.2X\n",
		data[0], data[1], data[2], data[3]);

	ts->fw_ver = data[3];
	ts->command_b3 = pdata->command_b3;
	ts->command_c2 = pdata->command_c2;

	ts->cable_config = pdata->cable_config;
	if (ts->cable_config[0])
		printk(KERN_INFO "cable_config => 0x%2.2X 0x%2.2X\n",
			ts->cable_config[0], ts->cable_config[1]);

	ts->himax_wq = create_singlethread_workqueue("himax_touch");
	if (!ts->himax_wq)
		goto err_create_wq_failed;

	INIT_WORK(&ts->work, himax_ts_work_func);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_err(&client->dev, "Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "himax-touchscreen";

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);

	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);

	printk(KERN_INFO "input_set_abs_params: mix_x %d, max_x %d, min_y %d, max_y %d\n",
		pdata->abs_x_min, pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
		pdata->abs_x_min, pdata->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
		pdata->abs_y_min, pdata->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
		pdata->abs_pressure_min, pdata->abs_pressure_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,
		pdata->abs_width_min, pdata->abs_width_max, 0, 0);
#ifndef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE,
		0, ((pdata->abs_pressure_max << 16) | pdata->abs_width_max), 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION,
		0, (BIT(31) | (pdata->abs_x_max << 16) | pdata->abs_y_max), 0, 0);
#endif

	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(&client->dev,
			"%s: Unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	if (client->irq) {
		ret = request_irq(client->irq, himax_ts_irq_handler,
				  IRQF_TRIGGER_LOW, client->name, ts);
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}

	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = himax_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING + 1;
	ts->early_suspend.suspend = himax_ts_early_suspend;
	ts->early_suspend.resume = himax_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	private_ts = ts;
	himax_touch_sysfs_init();
	if (ts->cable_config[0])
		usb_register_notifier(&himax_cable_status_handler);
	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_create_wq_failed:
err_detect_failed:
err_power_failed:
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:
	return ret;

}

static int himax8250_remove(struct i2c_client *client)
{
	struct himax_ts_data *ts = i2c_get_clientdata(client);

	himax_touch_sysfs_deinit();

	unregister_early_suspend(&ts->early_suspend);

	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);

	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;

}

static int himax8250_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	uint8_t data = 0x01;
	struct himax_ts_data *ts = i2c_get_clientdata(client);

	printk(KERN_INFO "%s: enter\n", __func__);

	disable_irq(client->irq);

	ret = cancel_work_sync(&ts->work);
	if (ret)
		enable_irq(client->irq);

	i2c_himax_write_command(ts->client, 0x82);
	msleep(5);
	i2c_himax_write_command(ts->client, 0x80);
	msleep(5);
	i2c_himax_write(ts->client, 0xD7, &data, 1);

	return 0;
}

static int himax8250_resume(struct i2c_client *client)
{
	uint8_t data[1];

	struct himax_ts_data *ts = i2c_get_clientdata(client);
	printk(KERN_INFO "%s: enter\n", __func__);

	data[0] = 0x00;
	i2c_himax_write(ts->client, 0xD7, &data[0], 1);
	usleep(100);

	i2c_himax_write_command(ts->client, 0x81);
	msleep(120);

	i2c_himax_write(ts->client, 0xB3, &ts->command_b3[1], 48);
	usleep(100);
	i2c_himax_write(ts->client, 0xC2, &ts->command_c2[1], 5);
	usleep(100);

	i2c_himax_write_command(ts->client, 0x83);

	enable_irq(client->irq);

	return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void himax_ts_early_suspend(struct early_suspend *h)
{
	struct himax_ts_data *ts;
	ts = container_of(h, struct himax_ts_data, early_suspend);
	himax8250_suspend(ts->client, PMSG_SUSPEND);
}

static void himax_ts_late_resume(struct early_suspend *h)
{
	struct himax_ts_data *ts;
	ts = container_of(h, struct himax_ts_data, early_suspend);
	himax8250_resume(ts->client);

}
#endif

static const struct i2c_device_id himax8250_ts_id[] = {
	{HIMAX8250_NAME, 0 },
	{}
};

static struct i2c_driver himax8250_driver = {
	.id_table	= himax8250_ts_id,
	.probe		= himax8250_probe,
	.remove		= himax8250_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= himax8250_suspend,
	.resume		= himax8250_resume,
#endif
	.driver		= {
		.name = HIMAX8250_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init himax8250_init(void)
{
	return i2c_add_driver(&himax8250_driver);
}

static void __exit himax8250_exit(void)
{
	i2c_del_driver(&himax8250_driver);
}

module_init(himax8250_init);
module_exit(himax8250_exit);

MODULE_DESCRIPTION("Himax8250 driver");
MODULE_LICENSE("GPL");
