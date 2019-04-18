/*****************************************************************************
	Copyright(c) 2016 FCI Inc. All Rights Reserved

	File name : fc8350.c

	Description : Driver source file

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

	History :
	----------------------------------------------------------------------
*******************************************************************************/
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/vmalloc.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/notifier.h>

#include "fc8350.h"
#include "bbm.h"
#include "fci_oal.h"
#include "fci_tun.h"
#include "fc8350_regs.h"
#include "fc8350_isr.h"
#include "fci_hal.h"

struct ISDBT_INIT_INFO_T *hInit;
#define RING_BUFFER_SIZE	(188 * 320 * 8)
#include <linux/spi/spi.h>
#include "fci_types.h"

#include <linux/of_irq.h>


/* GPIO(RESET & INTRRUPT) Setting */
#define FC8350_NAME		"isdbt"
#ifndef CONFIG_OF
#define GPIO_ISDBT_IRQ IRQ_EINT(2)
#define GPIO_ISDBT_PWR_EN EXYNOS4_GPK1(2)
#define GPIO_ISDBT_RST EXYNOS4_GPL2(6)
#else
static int ldo_gpio;
static int irq_gpio;
static int irq_gpio_num;
static int enable_gpio;
static int reset_gpio;
#define GPIO_ISDBT_LDO		ldo_gpio
#define GPIO_ISDBT_IRQ		irq_gpio
#define GPIO_ISDBT_PWR_EN	enable_gpio
#define GPIO_ISDBT_RST		reset_gpio
#endif
#define FC8350_CHIP_ID 0x8350
#define FC8350_CHIP_ID_REG 0x26

static DEFINE_MUTEX(ringbuffer_lock);
DEFINE_MUTEX(driver_mode_lock);
static DECLARE_WAIT_QUEUE_HEAD(isdbt_isr_wait);

struct ISDBT_OPEN_INFO_T hOpen_Val;
u8 static_ringbuffer[RING_BUFFER_SIZE];
enum ISDBT_MODE driver_mode = ISDBT_POWEROFF;
u32 bbm_xtal_freq;
u32 bbm_bandwidth;
u32 bbm_bandwidth_dvb;
u32 bbm_tsif_clk;
#ifndef BBM_I2C_TSIF
static u8 isdbt_isr_sig;

static struct work_struct work_tmp;
static bool dbt_charger_mitigate_enabe;
struct notifier_block	chg_nb;
struct work_struct	charger_update_work;
extern void fc8350_isr(struct work_struct *work);
static irqreturn_t isdbt_threaded_irq(int irq, void *dev_id)
{
	//struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)dev_id;

	//printk("[%s][%d]\n",__func__,__LINE__);
	mutex_lock(&driver_mode_lock);
	isdbt_isr_sig = 1;
	if (driver_mode == ISDBT_POWERON){
		//bbm_com_isr(hInit);
		schedule_work(&work_tmp);
	}
	isdbt_isr_sig = 0;
	mutex_unlock(&driver_mode_lock);

	return IRQ_HANDLED;
}


static irqreturn_t isdbt_irq(int irq, void *dev_id)
{
	return IRQ_WAKE_THREAD;
}


void isdbt_isr_check(HANDLE hDevice)
{
	u8 isr_time = 0;

	bbm_com_write(hDevice, DIV_BROADCAST, BBM_BUF_ENABLE, 0x00);
	while (isr_time < 10) {
		if (!isdbt_isr_sig)
			break;
		msWait(10);
		isr_time++;
	}

}

int data_callback(ulong hDevice, u8 bufid, u8 *data, int len)
{
	struct ISDBT_INIT_INFO_T *hInit;
	struct list_head *temp;
	hInit = (struct ISDBT_INIT_INFO_T *)hDevice;

	list_for_each(temp, &(hInit->hHead))
	{
		struct ISDBT_OPEN_INFO_T *hOpen;

		hOpen = list_entry(temp, struct ISDBT_OPEN_INFO_T, hList);

		if (hOpen->isdbttype == TS_TYPE) {
			if (fci_ringbuffer_free(&hOpen->RingBuffer) < len)
				FCI_RINGBUFFER_SKIP(&hOpen->RingBuffer, len);
			mutex_lock(&ringbuffer_lock);
			fci_ringbuffer_write(&hOpen->RingBuffer, data, len);
			mutex_unlock(&ringbuffer_lock);
		}
	}

	return 0;
}
#endif
int isdbt_hw_setting(HANDLE hDevice)
{
	int err;
	struct ISDBT_INIT_INFO_T *hInit = (struct ISDBT_INIT_INFO_T *)hDevice;

	print_log(0, "isdbt_hw_setting\n");

	err = gpio_request(GPIO_ISDBT_LDO, "isdbt_ldo");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_ldo\n");
		goto gpio_isdbt_ldo;
	}
	gpio_direction_output(GPIO_ISDBT_LDO, 0);

	err = gpio_request(GPIO_ISDBT_PWR_EN, "isdbt_en");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_en\n");
		goto gpio_isdbt_en;
	}
	gpio_direction_output(GPIO_ISDBT_PWR_EN, 0);
	err = gpio_export(GPIO_ISDBT_PWR_EN, 0);
	if (err)
		print_log(0, "%s: error %d gpio_export for %d\n",
			__func__, err, GPIO_ISDBT_PWR_EN);
	else {
		err = gpio_export_link(fc8350_misc_device.this_device,
			"isdbt_en", GPIO_ISDBT_PWR_EN);
		if (err)
			print_log(0, "%s: error %d gpio_export for %d\n",
				__func__, err, GPIO_ISDBT_PWR_EN);
	}

	err = gpio_request(GPIO_ISDBT_RST, "isdbt_rst");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_rst\n");
		//goto gpio_isdbt_rst;
	}
	gpio_direction_output(GPIO_ISDBT_RST, 0);

#ifndef BBM_I2C_TSIF
	err = gpio_request(GPIO_ISDBT_IRQ, "isdbt_irq");
	if (err) {
		print_log(0, "isdbt_hw_setting: Couldn't request isdbt_irq\n");
		//goto gpio_isdbt_rst;
	}

	gpio_direction_input(GPIO_ISDBT_IRQ);
	irq_gpio=gpio_to_irq(GPIO_ISDBT_IRQ);
	err = request_threaded_irq(irq_gpio, isdbt_irq
		, isdbt_threaded_irq, IRQF_TRIGGER_FALLING
		, FC8350_NAME, hInit);
	if (err < 0) {
		print_log(0,
			"isdbt_hw_setting: request_threaded_irq %d reason(%d)\n"
			, GPIO_ISDBT_IRQ, err);
	goto request_isdbt_irq;
	}

#endif


	return 0;

#ifndef BBM_I2C_TSIF
request_isdbt_irq:
	gpio_free(irq_gpio_num);
#endif
#if 0
gpio_isdbt_rst:
	gpio_free(GPIO_ISDBT_PWR_EN);

#endif
gpio_isdbt_en:
	gpio_free(GPIO_ISDBT_LDO);
gpio_isdbt_ldo:
	return err;
}

/*POWER_ON & HW_RESET & INTERRUPT_CLEAR */
void isdbt_hw_init(void)
{
	mutex_lock(&driver_mode_lock);
	print_log(0, "isdbt_hw_init\n");
	gpio_set_value(GPIO_ISDBT_LDO, 1);
	gpio_set_value(GPIO_ISDBT_RST, 0);
	gpio_set_value(GPIO_ISDBT_PWR_EN, 1);
	mdelay(5);
	gpio_set_value(GPIO_ISDBT_RST, 1);
	mdelay(5);
	driver_mode = ISDBT_POWERON;
	mutex_unlock(&driver_mode_lock);
#ifdef BBM_SPI_IF
	bbm_com_byte_write(hInit, DIV_BROADCAST, BBM_DM_DATA, 0x00);
#endif
}

/*POWER_OFF */
void isdbt_hw_deinit(void)
{
	mutex_lock(&driver_mode_lock);
	print_log(0, "isdbt_hw_deinit\n");
	gpio_set_value(GPIO_ISDBT_RST, 0);
	gpio_set_value(GPIO_ISDBT_PWR_EN, 0);
	gpio_set_value(GPIO_ISDBT_LDO, 0);
	driver_mode = ISDBT_POWEROFF;
	mutex_unlock(&driver_mode_lock);
}

#ifdef CONFIG_COMPAT
long isdbt_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	arg = (unsigned long) compat_ptr(arg);
	return isdbt_ioctl(filp, cmd, arg);
}
#endif

const struct file_operations isdbt_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl		= isdbt_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl		= isdbt_compat_ioctl,
#endif
	.open		= isdbt_open,
	.read		= isdbt_read,
	.release	= isdbt_release,
};

struct miscdevice fc8350_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = FC8350_NAME,
	.fops = &isdbt_fops,
};

int isdbt_open(struct inode *inode, struct file *filp)
{
	struct ISDBT_OPEN_INFO_T *hOpen;
	print_log(hInit, "isdbt open\n");
	hOpen = &hOpen_Val;
	hOpen->buf = &static_ringbuffer[0];
	hOpen->isdbttype = 0;
	if (list_empty(&(hInit->hHead)))
		list_add(&(hOpen->hList), &(hInit->hHead));
	hOpen->hInit = (HANDLE *)hInit;
	fci_ringbuffer_init(&hOpen->RingBuffer, hOpen->buf, RING_BUFFER_SIZE);
	filp->private_data = hOpen;
	return 0;
}

ssize_t isdbt_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	s32 avail;
	struct ISDBT_OPEN_INFO_T *hOpen
		= (struct ISDBT_OPEN_INFO_T *)filp->private_data;
	struct fci_ringbuffer *cibuf = &hOpen->RingBuffer;
	ssize_t len, read_len = 0;
	if (!cibuf->data || !count)	{
		/*print_log(hInit, " return 0\n"); */
		return 0;
	}
	if (fci_ringbuffer_empty(cibuf))	{
		/*print_log(hInit, "return EWOULDBLOCK\n"); */
		return -EWOULDBLOCK;
	}
	mutex_lock(&ringbuffer_lock);
	avail = fci_ringbuffer_avail(cibuf);
	mutex_unlock(&ringbuffer_lock);
	if (count >= avail)
		len = avail;
	else
		len = count - (count % 188);
	read_len = fci_ringbuffer_read_user(cibuf, buf, len);
	return read_len;
}

int isdbt_release(struct inode *inode, struct file *filp)
{
	struct ISDBT_OPEN_INFO_T *hOpen;
	print_log(hInit, "isdbt_release\n");
	isdbt_hw_deinit();
	hOpen = filp->private_data;
	hOpen->isdbttype = 0;
	if (!list_empty(&(hInit->hHead)))
		list_del(&(hOpen->hList));
	return 0;
}

#define CHG_CURRENT_LIMIT_VAL 800000
#define CHG_CURRENT_DEFAULT_VAL -22
static void chg_update_work(struct work_struct *work)
{
	int ret;
	int ctm_current;
	union power_supply_propval pval = {0, };
	struct power_supply *usb_psy;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("Couldn't get usb psy\n");
		return;
	}

	ret = power_supply_get_property(usb_psy,
			POWER_SUPPLY_PROP_PRESENT, &pval);
	if (ret < 0) {
		pr_err("Couldn't get usb presentret=%d\n", ret);
		return;
	}
	if (!pval.intval) {
		pr_info("usb not present\n");
		return;
	}

	ret = power_supply_get_property(usb_psy,
			POWER_SUPPLY_PROP_TYPEC_MODE, &pval);
	if (ret < 0) {
		pr_err("Couldn't get typeC mode=%d\n", ret);
		return;
	}
	if (pval.intval == POWER_SUPPLY_TYPEC_NONE) {
		pr_info("type c is none\n");
		return;
	}

	ret = power_supply_get_property(usb_psy,
			POWER_SUPPLY_PROP_CTM_CURRENT_MAX, &pval);
	if (ret < 0) {
		pr_err("Couldn't get ctm current maxt=%d\n", ret);
		return;
	}
	ctm_current = pval.intval;

	mutex_lock(&driver_mode_lock);
	if (driver_mode == ISDBT_POWERON
			&& (ctm_current > CHG_CURRENT_LIMIT_VAL
			|| ctm_current < 0)) {
		pr_info("start limit crrent,orig = %d\n", ctm_current);
		pval.intval = CHG_CURRENT_LIMIT_VAL;
		ret = power_supply_set_property(usb_psy,
				POWER_SUPPLY_PROP_CTM_CURRENT_MAX, &pval);
		if (ret < 0)
			pr_err("Couldn't limit CTM_CURRENT_MAX ret=%d\n", ret);
	} else if (driver_mode != ISDBT_POWERON) {
		pr_info("recovry chg current, mode =%d,orig = %d\n",
			driver_mode, ctm_current);
		pval.intval = CHG_CURRENT_DEFAULT_VAL;
		ret = power_supply_set_property(usb_psy,
			POWER_SUPPLY_PROP_CTM_CURRENT_MAX, &pval);
		if (ret < 0)
			pr_err("Couldn't recovery chg current ret=%d\n", ret);
	}
	mutex_unlock(&driver_mode_lock);
}

static int dbt_notifier_call(struct notifier_block *nb,
		unsigned long ev, void *v)
{
	struct power_supply *psy = v;

	if (!strcmp(psy->desc->name, "usb")) {
		if (ev == PSY_EVENT_PROP_CHANGED)
			schedule_work(&charger_update_work);
	}

	return NOTIFY_OK;
}

static int dbt_register_notifier(void)
{
	int rc;

	chg_nb.notifier_call = dbt_notifier_call;
	rc = power_supply_reg_notifier(&chg_nb);
	if (rc < 0) {
		pr_err("Couldn't register psy notifier rc = %d\n", rc);
		return rc;
	}

	return 0;
}

long isdbt_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	s32 res = BBM_NOK;
	s32 err = 0;
	u32 size = 0;
	struct ISDBT_OPEN_INFO_T *hOpen;

	struct ioctl_info info;

	if (_IOC_TYPE(cmd) != IOCTL_MAGIC)
		return -EINVAL;
	if (_IOC_NR(cmd) >= IOCTL_MAXNR)
		return -EINVAL;

	hOpen = filp->private_data;

	size = _IOC_SIZE(cmd);
	if (size > sizeof(struct ioctl_info))
		size = sizeof(struct ioctl_info);
	switch (cmd) {
	case IOCTL_ISDBT_RESET:
		res = bbm_com_reset(hInit, DIV_BROADCAST);
#ifdef FC8350_DEBUG
		print_log(hInit, "[FC8350] IOCTL_ISDBT_RESET\n");
#endif
		break;
	case IOCTL_ISDBT_INIT:
		res = bbm_com_i2c_init(hInit, FCI_HPI_TYPE);
		res |= bbm_com_probe(hInit, DIV_BROADCAST);
		print_log(hInit
		, "[FC8350] ISDBT_INIT BBM : %s Xtal : %d, DEV : %s\n"
		, DRIVER_VER, BBM_XTAL_FREQ, DRV_VER);

		if (res) {
			print_log(hInit, "FC8350 Initialize Fail\n");
			break;
		}
		res |= bbm_com_init(hOpen, DIV_BROADCAST);
#ifdef FC8350_DEBUG
		print_log(hInit, "[FC8350] IOCTL_ISDBT_INIT res %d\n", res);
#endif
		break;
	case IOCTL_ISDBT_BYTE_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_byte_read(hInit, DIV_BROADCAST, (u16)info.buff[0]
			, (u8 *)(&info.buff[1]));
#ifdef FC8350_DEBUG
		print_log(hInit
		, "[FC8350] IOCTL_ISDBT_BYTE_READ [0x%x][0x%x]\n"
		, (u16)info.buff[0], (u8)info.buff[1]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_WORD_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_word_read(hInit, DIV_BROADCAST, (u16)info.buff[0]
			, (u16 *)(&info.buff[1]));
#ifdef FC8350_DEBUG
		print_log(hInit
		, "[FC8350] IOCTL_ISDBT_WORD_READ [0x%x][0x%x]\n"
		, (u16)info.buff[0], (u16)info.buff[1]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_LONG_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_long_read(hInit, DIV_BROADCAST, (u16)info.buff[0]
			, (u32 *)(&info.buff[1]));
#ifdef FC8350_DEBUG
		print_log(hInit
		, "[FC8350] IOCTL_ISDBT_LONG_READ [0x%x][0x%x]\n"
		, (u16)info.buff[0], (u32)info.buff[1]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_BULK_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		if (info.buff[1] >
			(sizeof(info.buff) - sizeof(info.buff[0]) * 2)) {
			print_log(hInit, "[FC8350] BULK_READ sizeErr %d\n"
				, info.buff[1]);
			res = BBM_NOK;
			break;
		}
		res = bbm_com_bulk_read(hInit, DIV_BROADCAST, (u16)info.buff[0]
			, (u8 *)(&info.buff[2]), info.buff[1]);
#ifdef FC8350_DEBUG
		print_log(hInit
		, "[FC8350] IOCTL_ISDBT_BULK_READ [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[0], (u16)info.buff[1], info.buff[2]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_BYTE_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_byte_write(hInit, DIV_BROADCAST, (u16)info.buff[0]
			, (u8)info.buff[1]);
#ifdef FC8350_DEBUG
		print_log(hInit
		, "[FC8350] IOCTL_ISDBT_BYTE_WRITE [0x%x][0x%x]\n"
		, (u16)info.buff[0], info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_WORD_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_word_write(hInit, DIV_BROADCAST, (u16)info.buff[0]
			, (u16)info.buff[1]);
#ifdef FC8350_DEBUG
		print_log(hInit
		, "[FC8350] IOCTL_ISDBT_WORD_WRITE [0x%x][0x%x]\n"
		, (u16)info.buff[0], (u16)info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_LONG_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_long_write(hInit, DIV_BROADCAST, (u16)info.buff[0]
			, (u32)info.buff[1]);
#ifdef FC8350_DEBUG
		print_log(hInit
		, "[FC8350] IOCTL_ISDBT_LONG_WRITE [0x%x][0x%x]\n"
		, (u16)info.buff[0], (u32)info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_BULK_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		if (info.buff[1] >
			(sizeof(info.buff) - sizeof(info.buff[0]) * 2)) {
			print_log(hInit, "[FC8350] BULK_WRITE sizeErr %d\n"
				, info.buff[1]);
			res = BBM_NOK;
			break;
		}
		res = bbm_com_bulk_write(hInit, DIV_BROADCAST, (u16)info.buff[0]
			, (u8 *)(&info.buff[2]), info.buff[1]);
#ifdef FC8350_DEBUG
		print_log(hInit
		, "[FC8350] IOCTL_ISDBT_BULK_WRITE [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[0], info.buff[1], info.buff[2]);
#endif
		break;
	case IOCTL_ISDBT_TUNER_READ:
		err = copy_from_user((void *)&info, (void *)arg, size);
		if ((info.buff[1] > 1) || (info.buff[2] >
			(sizeof(info.buff) - sizeof(info.buff[0]) * 3))) {
			print_log(hInit
				, "[FC8350] TUNER_R sizeErr A[%d] D[%d]\n"
				, info.buff[1], info.buff[2]);
			res = BBM_NOK;
			break;
		}
		res = bbm_com_tuner_read(hInit, DIV_BROADCAST, (u8)info.buff[0]
			, (u8)info.buff[1],  (u8 *)(&info.buff[3])
			, (u8)info.buff[2]);
#ifdef FC8350_DEBUG
		print_log(hInit
		, "[FC8350] IOCTL_ISDBT_TUNER_READ [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[0], (u16)info.buff[2], info.buff[3]);
#endif
		err |= copy_to_user((void *)arg, (void *)&info, size);
		break;
	case IOCTL_ISDBT_TUNER_WRITE:
		err = copy_from_user((void *)&info, (void *)arg, size);
		if ((info.buff[1] > 1) || (info.buff[2] >
			(sizeof(info.buff) - sizeof(info.buff[0]) * 3))) {
			print_log(hInit
				, "[FC8350] TUNER_R sizeErr A[%d] D[%d]\n"
				, info.buff[1], info.buff[2]);
			res = BBM_NOK;
			break;
		}
		res = bbm_com_tuner_write(hInit, DIV_BROADCAST, (u8)info.buff[0]
			, (u8)info.buff[1], (u8 *)(&info.buff[3])
			, (u8)info.buff[2]);
#ifdef FC8350_DEBUG
		print_log(hInit
		, "[FC8350] IOCTL_ISDBT_TUNER_WRITE [0x%x][0x%x][0x%x]\n"
		, (u16)info.buff[0], (u16)info.buff[2], info.buff[3]);
#endif
		break;
	case IOCTL_ISDBT_TUNER_SET_FREQ:
		{
			u32 f_rf;
			u8 subch;
			err = copy_from_user((void *)&info, (void *)arg, size);

			f_rf = (u32)info.buff[0];
			subch = (u8)info.buff[1];
#ifndef BBM_I2C_TSIF
			isdbt_isr_check(hInit);
#endif
			res = bbm_com_tuner_set_freq(hInit
				, DIV_BROADCAST, f_rf, subch);
#ifdef FC8350_DEBUG
		print_log(hInit
		, "[FC8350] IOCTL_ISDBT_TUNER_SET_FREQ [%d][0x%x]\n"
		, f_rf, subch);
#endif
#ifndef BBM_I2C_TSIF
			mutex_lock(&ringbuffer_lock);
			fci_ringbuffer_flush(&hOpen->RingBuffer);
			mutex_unlock(&ringbuffer_lock);
			bbm_com_write(hInit
				, DIV_BROADCAST, BBM_BUF_ENABLE, 0x01);
#endif
		}
		break;
	case IOCTL_ISDBT_TUNER_SELECT:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_select(hOpen
			, DIV_BROADCAST, (u32)info.buff[0], (u32)info.buff[1]);
#ifdef FC8350_DEBUG
		print_log(hInit
		, "[FC8350] IOCTL_ISDBT_TUNER_SELECT [%d][0x%x]\n"
		, (u32)info.buff[0], (u32)info.buff[1]);
#endif
		break;
	case IOCTL_ISDBT_TS_START:
		hOpen->isdbttype = TS_TYPE;
#ifdef FC8350_DEBUG
		print_log(hInit, "[FC8350] IOCTL_ISDBT_TS_START\n");
#endif
		break;
	case IOCTL_ISDBT_TS_STOP:
		hOpen->isdbttype = 0;
#ifdef FC8350_DEBUG
		print_log(hInit, "[FC8350] IOCTL_ISDBT_TS_STOP\n");
#endif
		break;
	case IOCTL_ISDBT_POWER_ON:
		isdbt_hw_init();
		if (dbt_charger_mitigate_enabe)
			schedule_work(&charger_update_work);
#ifdef FC8350_DEBUG
		print_log(hInit, "[FC8350] IOCTL_ISDBT_POWER_ON\n");
#endif
		break;
	case IOCTL_ISDBT_POWER_OFF:
		isdbt_hw_deinit();
		if (dbt_charger_mitigate_enabe)
			schedule_work(&charger_update_work);
#ifdef FC8350_DEBUG
		print_log(hInit, "[FC8350] IOCTL_ISDBT_POWER_OFF\n");
#endif
		break;
	case IOCTL_ISDBT_SCAN_STATUS:
		res = bbm_com_scan_status(hInit, DIV_BROADCAST);
#ifdef FC8350_DEBUG
		print_log(hInit
		, "[FC8350] IOCTL_ISDBT_SCAN_STATUS\n");
#endif
		break;
	case IOCTL_ISDBT_TUNER_GET_RSSI:
		err = copy_from_user((void *)&info, (void *)arg, size);
		res = bbm_com_tuner_get_rssi(hInit
			, DIV_BROADCAST, (s32 *)&info.buff[0]);
		err |= copy_to_user((void *)arg, (void *)&info, size);
#ifdef FC8350_DEBUG
		print_log(hInit
		, "[FC8350] IOCTL_ISDBT_TUNER_GET_RSSI [%d]\n"
		, (u16)info.buff[0]);
#endif
		break;

	case IOCTL_ISDBT_DEINIT:
		res = bbm_com_deinit(hInit, DIV_BROADCAST);
#ifdef FC8350_DEBUG
		print_log(hInit, "[FC8350] IOCTL_ISDBT_DEINIT\n");
#endif
		break;
	case IOCTL_ISDBT_CONFIG_DRIVER:
		err = copy_from_user((void *)&info, (void *)arg, size);
		memcpy((void *)&hOpen->driver_config, (void *)&info.buff[0]
			, sizeof(struct drv_cfg));
		/*bbm_xtal_freq		= hOpen->driver_config.v_xtal_freq; */ /*fccon always send 32000*/
		bbm_xtal_freq		= BBM_XTAL_FREQ;/*use device tree xtal config*/
		bbm_bandwidth		= hOpen->driver_config.v_band_width;
		bbm_bandwidth_dvb	= hOpen->driver_config.v_band_width_dvb;
		bbm_tsif_clk		= hOpen->driver_config.v_tsif_clk;
		print_log(NULL, "CFG Set\n");
		print_log(NULL, "	 1  : null_pid_filter (%d)\n", hOpen->driver_config.b_null_pid_filter);
		print_log(NULL, "	 2  : fail_frame (%d)\n", hOpen->driver_config.b_fail_frame);
		print_log(NULL, "	 3  : ts_204 (%d)\n", hOpen->driver_config.b_ts_204);
		print_log(NULL, "	 4  : descrambler (%d)\n", hOpen->driver_config.b_descrambler);
		print_log(NULL, "	 5  : i2c_parallel_tsif (%d)\n", hOpen->driver_config.b_i2c_parallel_tsif);
		print_log(NULL, "	 6  : i2c_spi_pol_hi (%d)\n", hOpen->driver_config.b_i2c_spi_pol_hi);
		print_log(NULL, "	 7  : i2c_spi_pha_hi (%d)\n", hOpen->driver_config.b_i2c_spi_pha_hi);
		print_log(NULL, "	 8  : ext_lna (%d)\n", hOpen->driver_config.b_ext_lna);
		print_log(NULL, "	 9  : ext_lna_pol_high (%d)\n", hOpen->driver_config.b_ext_lna_pol_high);
		print_log(NULL, "	 10 : spi_30M (%d)\n", hOpen->driver_config.b_spi_30M);
		print_log(NULL, "	 11 : xtal_freq (%d)\n", hOpen->driver_config.v_xtal_freq);
		print_log(NULL, "	 12 : band_width (%d)\n", hOpen->driver_config.v_band_width);
		print_log(NULL, "	 13 : band_width_dvb (%d)\n", hOpen->driver_config.v_band_width_dvb);
		print_log(NULL, "	 14 : tsif_clk (%d)\n", hOpen->driver_config.v_tsif_clk);
		break;

	default:
		print_log(hInit, "isdbt ioctl error!\n");
		res = BBM_NOK;
		break;
	}

	if (err < 0) {
		print_log(hInit, "copy to/from user fail : %d", err);
		res = BBM_NOK;
	}
	return res;
}

#ifdef CONFIG_OF
static int fc8350_dt_init(void)
{
	struct device_node *np;
	u32 rc;

	np = of_find_compatible_node(NULL, NULL,
		fc8350_match_table[0].compatible);
	if (!np)
		return -ENODEV;

	ldo_gpio = of_get_named_gpio(np, "ldo-gpio", 0);
	if (!gpio_is_valid(ldo_gpio)) {
		print_log(hInit, "isdbt error getting ldo_gpio\n");
		return -EINVAL;
	}

	enable_gpio = of_get_named_gpio(np, "enable-gpio", 0);
	if (!gpio_is_valid(enable_gpio)) {
		print_log(hInit, "isdbt error getting enable_gpio\n");
		return -EINVAL;
	}

	reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (!gpio_is_valid(reset_gpio)) {
		print_log(hInit, "isdbt error getting reset_gpio\n");
		return -EINVAL;
	}

	irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (!gpio_is_valid(irq_gpio)) {
		print_log(hInit, "isdbt error getting irq_gpio\n");
		return -EINVAL;
	}
	irq_gpio_num = irq_gpio;

	bbm_xtal_freq = DEFAULT_BBM_XTAL_FREQ;
	rc = of_property_read_u32(np, "bbm-xtal-freq", &bbm_xtal_freq);
	if (rc)
		print_log(hInit, "no dt xtal-freq config, using default\n");

	bbm_bandwidth = DEFAULT_BBM_BAND_WIDTH;
	bbm_bandwidth_dvb = DEFAULT_BBM_BAND_WIDTH_DVB;
	bbm_tsif_clk = DEFAULT_BBM_TSIF_CLK;

	if (of_property_read_bool(np, "dbt-charger-mitigate-enable"))
		dbt_charger_mitigate_enabe = true;
	else
		dbt_charger_mitigate_enabe = false;

	return 0;
}
#else
static int fc8350_dt_init(void)
{
	bbm_xtal_freq = DEFAULT_BBM_XTAL_FREQ;
	bbm_bandwidth = DEFAULT_BBM_BAND_WIDTH;
	bbm_bandwidth_dvb = DEFAULT_BBM_BAND_WIDTH_DVB;
	bbm_tsif_clk = DEFAULT_BBM_TSIF_CLK;
	return 0;
}
#endif

s32 isdbt_chip_id(void)
{
	s32 res;
	u16 addr, data;

	isdbt_hw_init();
	addr = FC8350_CHIP_ID_REG;
	res = bbm_com_word_read(hInit, DIV_BROADCAST, addr, &data);
	if (res) {
		print_log(hInit, "%s reading chip id err %d\n", __func__, res);
		goto errout;
	}

	if (FC8350_CHIP_ID != data) {
		print_log(hInit, "%s wrong chip id %#x\n", __func__, data);
		res = -1;
	} else
		print_log(hInit, "%s reg %#x id %#x\n", __func__, addr, data);

errout:
	isdbt_hw_deinit();
	return res;
}

int isdbt_init(void)
{
	s32 res;

	print_log(NULL, "%s\n",__func__);
	res = misc_register(&fc8350_misc_device);
	if (res < 0) {
		print_log(NULL, "isdbt init fail : %d\n", res);
		return res;
	}
	hInit = kmalloc(sizeof(struct ISDBT_INIT_INFO_T), GFP_KERNEL);
	res = fc8350_dt_init();
	if (res) {
		misc_deregister(&fc8350_misc_device);
		return res;
	}
	isdbt_hw_setting(hInit);
	INIT_WORK(&work_tmp, &fc8350_isr);
#ifndef BBM_I2C_TSIF
	bbm_com_ts_callback_register((ulong)hInit, data_callback);
#endif
#if defined(BBM_I2C_TSIF) || defined(BBM_I2C_SPI)
	res = bbm_com_hostif_select(hInit, BBM_I2C);
#else
	res = bbm_com_hostif_select(hInit, BBM_SPI);
#endif
	if (res)
		print_log(hInit, "isdbt host interface select fail!\n");
	INIT_LIST_HEAD(&(hInit->hHead));
	INIT_WORK(&charger_update_work, chg_update_work);
	if (dbt_charger_mitigate_enabe)
		dbt_register_notifier();
	res = isdbt_chip_id();
	if (res)
		goto error_out;

	return 0;
error_out:
	isdbt_exit();
	return -ENODEV;
}

void isdbt_exit(void)
{
	print_log(hInit, "isdbt isdbt_exit\n");
	if (dbt_charger_mitigate_enabe)
		power_supply_unreg_notifier(&chg_nb);
	cancel_work_sync(&charger_update_work);
	isdbt_hw_deinit();
#ifndef BBM_I2C_TSIF
	free_irq(GPIO_ISDBT_IRQ, hInit);
	gpio_free(irq_gpio_num);
#endif
	gpio_free(GPIO_ISDBT_RST);
	gpio_free(GPIO_ISDBT_PWR_EN);
	gpio_free(GPIO_ISDBT_LDO);

#ifndef BBM_I2C_TSIF
	bbm_com_ts_callback_deregister();
#endif
	bbm_com_hostif_deselect(hInit);
	if (hInit != NULL)
		kfree(hInit);
	misc_deregister(&fc8350_misc_device);
}

module_init(isdbt_init);
module_exit(isdbt_exit);
MODULE_LICENSE("Dual BSD/GPL");
