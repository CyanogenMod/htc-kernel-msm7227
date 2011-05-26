/* drivers/input/keyboard/synaptics_i2c_rmi.c
 *
 * Copyright (C) 2007 Google, Inc.
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/synaptics_i2c_rmi.h>


static struct workqueue_struct *synaptics_wq;

struct synaptics_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max[2];
	int snap_state[2][2];
	int snap_down_on[2];
	int snap_down_off[2];
	int snap_up_on[2];
	int snap_up_off[2];
	int snap_down[2];
	int snap_up[2];
	uint32_t flags;
	uint8_t reported_finger_count;
	int8_t sensitivity_adjust;
	int (*power)(int on);
	struct early_suspend early_suspend;
	struct page_description page_table[18];
	uint8_t key_pressed[2];
	uint8_t debug_log_level;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif

static struct synaptics_ts_data *gl_ts;
static const char SYNAPTICSNAME[]	= "Synaptics_T1021";
static uint32_t syn_panel_version;

static ssize_t touch_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s_%#x\n", SYNAPTICSNAME, syn_panel_version);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(vendor, 0444, touch_vendor_show, NULL);

static struct kobject *android_touch_kobj;

static ssize_t debug_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct synaptics_ts_data *ts = gl_ts;

	return sprintf(buf, "%d\n", ts->debug_log_level);
}

static ssize_t debug_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct synaptics_ts_data *ts = gl_ts;

	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
		ts->debug_log_level = buf[0] - '0';

	return count;
}

static DEVICE_ATTR(debug_level, 0644, debug_level_show, debug_level_store);

static int synaptics_touch_sysfs_init(void)
{
	int ret;
	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		printk(KERN_ERR "%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		printk(KERN_ERR "touch_sysfs_init: sysfs_create_group failed\n");
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr);
	if (ret) {
		printk(KERN_ERR "%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	return 0;
}

static void synaptics_touch_sysfs_remove(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
	kobject_del(android_touch_kobj);
}

static int synaptics_init_panel(struct synaptics_ts_data *ts)
{
	int ret = 0;

	if (ts->sensitivity_adjust) {
		ret = i2c_smbus_write_byte_data(ts->client,
			ts->page_table[2].value + 54, ts->sensitivity_adjust); /* Set Sensitivity */
		if (ret < 0)
			printk(KERN_ERR "i2c_smbus_write_byte_data failed for Sensitivity Set\n");
	}
	return ret;
}

static void synaptics_ts_work_func(struct work_struct *work)
{
	int i;
	int ret;
	int bad_data = 0;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf[12];
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);

	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	start_reg = ts->page_table[9].value + 1;
	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;

	for (i = 0; i < ((ts->use_irq && !bad_data) ? 1 : 10); i++) {
		ret = i2c_transfer(ts->client->adapter, msg, 2);
		if (ret < 0) {
			printk(KERN_ERR "synaptics_ts_work_func: i2c_transfer failed\n");
			/* reset touch control */
			if (ts->power) {
				ret = ts->power(0);
				if (ret < 0)
					printk(KERN_ERR "synaptics_ts_resume power off failed\n");
				msleep(10);
				ret = ts->power(1);
				if (ret < 0)
					printk(KERN_ERR	"synaptics_ts_resume power on failed\n");
			}
			synaptics_init_panel(ts);
			if (!ts->use_irq)
				hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
			else
				i2c_smbus_write_byte_data(ts->client, 0xf1, 0x01); /* enable abs int */
		} else {

			int pos[2][2];
			int f, a;
			int base = 2;
			uint8_t pos_mask = 0x0f;
			int finger = buf[1] & 0x03;
			int finger2 = buf[1] & 0x0C;
		/*	int x = (buf[4] & 0x0f) | (uint16_t)buf[2] << 4;
			int y = (buf[4] & 0xf0) >> 4 | (uint16_t)buf[3] << 4; */
			int w = buf[5] >> 4;
			int z = buf[6];
		/*	int x2 = (buf[9] & 0x0f) | (uint16_t)buf[7] << 4;
			int y2 = (buf[9] & 0xf0) >> 4 | (uint16_t)buf[8] << 4;
			int w2 = buf[10] >> 4;
			int z2 = buf[11]; */

			bad_data = 0;

			/* debug log level 1 */
			if (ts->debug_log_level & 0x1) {
				printk("%s: raw data:", __func__);
				for (i = 0; i < sizeof(buf); i++)
					printk(" %2x", buf[i]);
				printk("\n");
			}

			for (f = 0; f < 2; f++) {
				uint32_t flip_flag = SYNAPTICS_FLIP_X;
				pos_mask = 0x0f;
				for (a = 0; a < 2; a++) {
					pos[f][a] = (buf[base+2] & pos_mask) >> (a*4) | (uint16_t)buf[base+a] << 4;
					if (ts->flags & flip_flag)
						pos[f][a] = ts->max[a] - pos[f][a];
					flip_flag <<= 1;
					pos_mask <<= 4;
				}
				base += 5;
				if (ts->flags & SYNAPTICS_SWAP_XY)
					swap(pos[f][0], pos[f][1]);
			}

#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
			if (z) {
				input_report_abs(ts->input_dev, ABS_X, pos[0][0]);
				input_report_abs(ts->input_dev, ABS_Y, pos[0][1]);
			}
			input_report_abs(ts->input_dev, ABS_PRESSURE, z);
			input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, w);
			input_report_key(ts->input_dev, BTN_TOUCH, finger);
			if (finger2) {
				input_report_abs(ts->input_dev, ABS_HAT0X, pos[1][0]);
				input_report_abs(ts->input_dev, ABS_HAT0Y, pos[1][1]);
			}
#endif
			if (!finger)
				z = 0;
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, z);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, pos[0][0]);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, pos[0][1]);
			input_mt_sync(ts->input_dev);
			if (finger2) {
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, z);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, pos[1][0]);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, pos[1][1]);
				input_mt_sync(ts->input_dev);
			}
			input_sync(ts->input_dev);

			if (!ts->key_pressed[0] && finger) {
				ts->key_pressed[0] = 1;
				printk(KERN_INFO "S1@%d, %d\n", pos[0][0], pos[0][1]);
			} else if (ts->key_pressed[0] == 1 && !finger) {
				ts->key_pressed[0] = 2;
				printk(KERN_INFO "E1@%d, %d\n", pos[0][0], pos[0][1]);
			}
			if (ts->key_pressed[0] == 1 && !ts->key_pressed[1] && finger2) {
				ts->key_pressed[1] = 1;
				printk(KERN_INFO "S2@%d, %d\n", pos[1][0], pos[1][1]);
			} else if (ts->key_pressed[1] && !finger2) {
				ts->key_pressed[1] = 0;
				printk(KERN_INFO "E2@%d, %d\n", pos[1][0], pos[1][1]);
			}
			/* debug log level 2 */
			if (ts->debug_log_level & 0x2) {
				printk("X1 %4d, Y1 %4d, Z %3d, W %2d, F1 %d, X2 %4d, Y2 %4d, F2 %d\n",
						pos[0][0], pos[0][1], z, w, finger,
						pos[1][0], pos[1][1], finger2);
			}
		}
	}

	if (ts->use_irq)
		enable_irq(ts->client->irq);

}

static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);
	/* printk("synaptics_ts_timer_func\n"); */

	queue_work(synaptics_wq, &ts->work);

	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;

	/* printk("synaptics_ts_irq_handler\n"); */
	disable_irq_nosync(ts->client->irq);
	queue_work(synaptics_wq, &ts->work);
	return IRQ_HANDLED;
}

static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	uint8_t loop_i, loop_j;
	int ret = 0;
	uint16_t max_x, max_y;
	int fuzz_x, fuzz_y, fuzz_p, fuzz_w;
	struct synaptics_i2c_rmi_platform_data *pdata;
	unsigned long irqflags;
	int inactive_area_left;
	int inactive_area_right;
	int inactive_area_top;
	int inactive_area_bottom;
	int snap_left_on;
	int snap_left_off;
	int snap_right_on;
	int snap_right_off;
	int snap_top_on;
	int snap_top_off;
	int snap_bottom_on;
	int snap_bottom_off;
	uint32_t panel_version;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	ts->client = client;
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;
	if (pdata)
		ts->power = pdata->power;
	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0) {
			printk(KERN_ERR "synaptics_ts_probe power on failed\n");
			goto err_power_failed;
		}
	}
	{
		int retry = 10;
		while (retry-- > 0) {
			ret = i2c_smbus_read_byte_data(ts->client, 0x00DD);
			if (ret >= 0)
				break;
			msleep(100);
			printk(KERN_INFO "Retry touch panel reading\n");
		}
	}
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
		goto err_detect_failed;
	}

	for (loop_i = 0x00DD, loop_j = 0; loop_i <= 0x00EE; loop_i++) {
		ts->page_table[loop_j].addr = loop_i;
		ts->page_table[loop_j].value = i2c_smbus_read_byte_data(ts->client, loop_i);
		/* printk("[0x%2.2X]%2.2X\n", ts->page_table->addr, ts->page_table->value); */
		while (ts->page_table[loop_j].value != i2c_smbus_read_byte_data(ts->client, loop_i))
			ts->page_table[loop_j].value = i2c_smbus_read_byte_data(ts->client, loop_i);
		loop_j++;
	}

	panel_version =
		i2c_smbus_read_byte_data(ts->client, ts->page_table[6].value + 3) |
		i2c_smbus_read_byte_data(ts->client, ts->page_table[6].value + 2) << 8;
	printk(KERN_INFO "%s: panel_version: %x\n", __func__, panel_version);
	syn_panel_version = panel_version;

	ret = i2c_smbus_read_byte_data(ts->client, ts->page_table[6].value);
	printk(KERN_INFO "%s: Manufacturer ID %x\n", __func__, ret);
	/* Report mode */
	/* ret = i2c_smbus_write_byte_data(ts->client, ts->page_table[8].value + 1, 4); */

	if (pdata) {
		while (pdata->version > panel_version) {
			printk(KERN_INFO "synaptics_ts_probe: old tp detected, "
					"panel version = %x\n", panel_version);
			pdata++;
		}
		ts->flags = pdata->flags;
		ts->sensitivity_adjust = pdata->sensitivity_adjust;
		irqflags = pdata->irqflags;
		inactive_area_left = pdata->inactive_left;
		inactive_area_right = pdata->inactive_right;
		inactive_area_top = pdata->inactive_top;
		inactive_area_bottom = pdata->inactive_bottom;
		snap_left_on = pdata->snap_left_on;
		snap_left_off = pdata->snap_left_off;
		snap_right_on = pdata->snap_right_on;
		snap_right_off = pdata->snap_right_off;
		snap_top_on = pdata->snap_top_on;
		snap_top_off = pdata->snap_top_off;
		snap_bottom_on = pdata->snap_bottom_on;
		snap_bottom_off = pdata->snap_bottom_off;
		fuzz_x = pdata->fuzz_x;
		fuzz_y = pdata->fuzz_y;
		fuzz_p = pdata->fuzz_p;
		fuzz_w = pdata->fuzz_w;
	} else {
		inactive_area_left = 0;
		inactive_area_right = 0;
		inactive_area_top = 0;
		inactive_area_bottom = 0;
		snap_left_on = 0;
		snap_left_off = 0;
		snap_right_on = 0;
		snap_right_off = 0;
		snap_top_on = 0;
		snap_top_off = 0;
		snap_bottom_on = 0;
		snap_bottom_off = 0;
		fuzz_x = 0;
		fuzz_y = 0;
		fuzz_p = 0;
		fuzz_w = 0;
	}
	ts->max[0] = max_x =
		i2c_smbus_read_byte_data(ts->client, ts->page_table[2].value+6) |
		i2c_smbus_read_byte_data(ts->client, ts->page_table[2].value+7) << 8;

	ts->max[1] = max_y =
		i2c_smbus_read_byte_data(ts->client, ts->page_table[2].value+8) |
		i2c_smbus_read_byte_data(ts->client, ts->page_table[2].value+9) << 8;
	printk(KERN_INFO"max_x: %x, max_y: %x\n", max_x, max_y);

	ret = synaptics_init_panel(ts);
	if (ret < 0) {
		printk(KERN_ERR "synaptics_init_panel failed\n");
		goto err_detect_failed;
	}

	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
	if (!synaptics_wq)
		goto err_create_wq_failed;
	INIT_WORK(&ts->work, synaptics_ts_work_func);

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "synaptics-rmi-touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	inactive_area_left = inactive_area_left * max_x / 0x10000;
	inactive_area_right = inactive_area_right * max_x / 0x10000;
	inactive_area_top = inactive_area_top * max_y / 0x10000;
	inactive_area_bottom = inactive_area_bottom * max_y / 0x10000;
	snap_left_on = snap_left_on * max_x / 0x10000;
	snap_left_off = snap_left_off * max_x / 0x10000;
	snap_right_on = snap_right_on * max_x / 0x10000;
	snap_right_off = snap_right_off * max_x / 0x10000;
	snap_top_on = snap_top_on * max_y / 0x10000;
	snap_top_off = snap_top_off * max_y / 0x10000;
	snap_bottom_on = snap_bottom_on * max_y / 0x10000;
	snap_bottom_off = snap_bottom_off * max_y / 0x10000;
	fuzz_x = fuzz_x * max_x / 0x10000;
	fuzz_y = fuzz_y * max_y / 0x10000;
	ts->snap_down[!!(ts->flags & SYNAPTICS_SWAP_XY)] = -inactive_area_left;
	ts->snap_up[!!(ts->flags & SYNAPTICS_SWAP_XY)] = max_x + inactive_area_right;
	ts->snap_down[!(ts->flags & SYNAPTICS_SWAP_XY)] = -inactive_area_top;
	ts->snap_up[!(ts->flags & SYNAPTICS_SWAP_XY)] = max_y + inactive_area_bottom;
	ts->snap_down_on[!!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_left_on;
	ts->snap_down_off[!!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_left_off;
	ts->snap_up_on[!!(ts->flags & SYNAPTICS_SWAP_XY)] = max_x - snap_right_on;
	ts->snap_up_off[!!(ts->flags & SYNAPTICS_SWAP_XY)] = max_x - snap_right_off;
	ts->snap_down_on[!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_top_on;
	ts->snap_down_off[!(ts->flags & SYNAPTICS_SWAP_XY)] = snap_top_off;
	ts->snap_up_on[!(ts->flags & SYNAPTICS_SWAP_XY)] = max_y - snap_bottom_on;
	ts->snap_up_off[!(ts->flags & SYNAPTICS_SWAP_XY)] = max_y - snap_bottom_off;
	printk(KERN_INFO "synaptics_ts_probe: max_x %d, max_y %d\n", max_x, max_y);
	printk(KERN_INFO "synaptics_ts_probe: inactive_x %d %d, inactive_y %d %d\n",
	       inactive_area_left, inactive_area_right,
	       inactive_area_top, inactive_area_bottom);
	printk(KERN_INFO "synaptics_ts_probe: snap_x %d-%d %d-%d, snap_y %d-%d %d-%d\n",
	       snap_left_on, snap_left_off, snap_right_on, snap_right_off,
	       snap_top_on, snap_top_off, snap_bottom_on, snap_bottom_off);
	printk(KERN_INFO "input_set_abs_params: mix_x %d, max_x %d, min_y %d, max_y %d\n",
		-inactive_area_left, max_x + inactive_area_right,
		-inactive_area_top, max_y + inactive_area_bottom);
#ifdef CONFIG_TOUCHSCREEN_COMPATIBLE_REPORT
	input_set_abs_params(ts->input_dev, ABS_X, -inactive_area_left, max_x + inactive_area_right, fuzz_x, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, -inactive_area_top, max_y + inactive_area_bottom, fuzz_y, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, fuzz_p, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 15, fuzz_w, 0);
	input_set_abs_params(ts->input_dev, ABS_HAT0X, -inactive_area_left, max_x + inactive_area_right, fuzz_x, 0);
	input_set_abs_params(ts->input_dev, ABS_HAT0Y, -inactive_area_top, max_y + inactive_area_bottom, fuzz_y, 0);
#endif
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, -inactive_area_left, max_x + inactive_area_right, fuzz_x, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, -inactive_area_top, max_y + inactive_area_bottom, fuzz_y, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, fuzz_p, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, fuzz_w, 0);
	/* ts->input_dev->name = ts->keypad_info->name; */
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "synaptics_ts_probe: "
				"Unable to register %s input device\n",
				ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	gl_ts = ts;

	if (client->irq) {
		ret = request_irq(client->irq, synaptics_ts_irq_handler, IRQF_TRIGGER_LOW,
				client->name, ts);
		if (ret == 0) {
			/* enable abs int */
			ret = i2c_smbus_write_byte_data(ts->client, ts->page_table[8].value + 1, 4);
			if (ret)
				free_irq(client->irq, ts);
		}
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}
	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = synaptics_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	synaptics_touch_sysfs_init();

	printk(KERN_INFO "synaptics_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	destroy_workqueue(synaptics_wq);

err_create_wq_failed:
err_detect_failed:
err_power_failed:
	kfree(ts);

err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);

	synaptics_touch_sysfs_remove();

	kfree(ts);
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);

	ts->key_pressed[0] = 0;

	ret = i2c_smbus_write_byte_data(client, ts->page_table[8].value, 0x01); /* sleep */
	if (ret < 0)
		printk(KERN_ERR "synaptics_ts_suspend: i2c_smbus_write_byte_data failed\n");
	if (ts->power) {
		ret = ts->power(0);
		if (ret < 0)
			printk(KERN_ERR "synaptics_ts_resume power off failed\n");
	}
	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0)
			printk(KERN_ERR "synaptics_ts_resume power on failed\n");
	} else {
		ret = i2c_smbus_write_byte_data(client, ts->page_table[7].value, 0x01);
		msleep(250);
	}

	synaptics_init_panel(ts);

	if (ts->use_irq)
		enable_irq(client->irq);

	if (!ts->use_irq)
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id synaptics_ts_id[] = {
	{ SYNAPTICS_T1021_NAME, 0 },
	{ }
};

static struct i2c_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
	.id_table	= synaptics_ts_id,
	.driver = {
		.name	= SYNAPTICS_T1021_NAME,
	},
};

static int __devinit synaptics_ts_init(void)
{
	return i2c_add_driver(&synaptics_ts_driver);
}

static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);
	if (synaptics_wq)
		destroy_workqueue(synaptics_wq);
}

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");
