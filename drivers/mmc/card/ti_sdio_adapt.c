#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/mach-types.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>

#include <linux/delay.h>

#include <linux/miscdevice.h>
//#include <linux/cdev.h>
#include <asm/uaccess.h>

/* sdio function */
#include <linux/mmc/core.h>
#include <linux/mmc/card.h>
#include <linux/mmc/sdio_func.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio.h>


/* ====  IOCTL === */
#include <linux/ioctl.h>

/* Documentation/ioctl-number.txt */
/* the same with linux/wireless.h */
#define MAGIC			0x8B

#define TI_EMAPI_IOC_CONTROL					_IOW(MAGIC, 1, unsigned long)
#define TI_EMAPI_IOC_DIRECT_READ_BYTES		_IOR(MAGIC, 2, unsigned long)
#define TI_EMAPI_IOC_DIRECT_WRITE_BYTES		_IOW(MAGIC, 3, unsigned long)
#define TI_EMAPI_IOC_DATA_SYN_READ			_IOR(MAGIC, 4, unsigned long)
#define TI_EMAPI_IOC_DATA_SYN_WRITE			_IOW(MAGIC, 5, unsigned long)

#define TI_EMAPI_IOC_SDCC_CLK		_IOW(MAGIC, 12, unsigned long)


struct sdio_request {
	unsigned long	addr;
	unsigned long	len;
	uint8_t		*buf;
};

#define SDIO_BLK_SIZE 512
#define SDIO_DATA_BUFFER_SIZE	1024  // enlarge this size from 512 to 1024 for Triscope-196
unsigned char *__emapi_data_buffer = NULL;



#ifdef CONFIG_WIFI_CONTROL_FUNC
#include <linux/platform_device.h>
#include <linux/wifi_tiwlan.h>
int msm_wifi_power(int on);
#endif


struct tisdio_platform_data {
	int (*sdio_connect_bus)(void *fCbFunc,
                            void *hCbArg,
                            unsigned int uBlkSizeShift,
                            unsigned int uSdioThreadPriority);
	int (*sdio_disconnect_bus)(void);
	int (*sdio_execute_cmd)(unsigned int uCmd,
                            unsigned int uArg,
                            unsigned int uRespType,
                            void *       pResponse,
                            unsigned int uLen);
	int (*sdio_read)(unsigned int uFunc,
                            unsigned int uHwAddr,
                            void *       pData,
                            unsigned int uLen,
                            unsigned int bBlkMode,
                            unsigned int bIncAddr,
                            unsigned int bMore);
	int (*sdio_write)(unsigned int uFunc,
                            unsigned int uHwAddr,
                            void *       pData,
                            unsigned int uLen,
                            unsigned int bBlkMode,
                            unsigned int bIncAddr,
                            unsigned int bMore);
	int (*sdio_read_direct_bytes)(unsigned int uFunc,
                            unsigned int  uHwAddr,
                            unsigned char *pData,
                            unsigned int  uLen,
                            unsigned int  bMore);
	int (*sdio_write_direct_bytes)(unsigned int uFunc,
                             unsigned int  uHwAddr,
                             unsigned char *pData,
                             unsigned int  uLen,
                             unsigned int  bMore);
};

static struct tisdio_platform_data *ptr_ti_sdio_drv_data;


//#define _SDIO_DEBUG_

#ifdef _SDIO_DEBUG_
//////////////////////////////////////////
#define HWIO_SDC1_MD_REG_ADDR	0xa86000a0
#define HWIO_SDC1_NS_REG_ADDR	0xa86000a4
#include <asm/io.h>

static int __sdcc_set_mclk(uint16_t m, uint16_t n, uint16_t d,
		uint8_t p, uint8_t mncntr_mode, uint8_t src_sel, bool mncntr_en)
{
	uint32_t sdc_ns_reg;
	uint16_t d_val, n_val;
    void __iomem *md_reg = 0;
    void __iomem *ns_reg = 0;

    md_reg = ioremap(HWIO_SDC1_MD_REG_ADDR, 4096);
    if (!md_reg) {
        return -ENOMEM;
    }
    ns_reg = ioremap(HWIO_SDC1_NS_REG_ADDR, 4096);
    if (!ns_reg) {
        iounmap(md_reg);
        return -ENOMEM;
    }

	// Calculate the value for SDCX_MD_REG register
	d_val = ~(2 * d);
	n_val = ~(n - m);
	writel((m << 16) | (d_val), md_reg);

	// Calculate the value for SDC1_NS_REG register
	sdc_ns_reg = (n_val << 16) |			//SDC1_N_VAL
			(1 << 11) |			//SDC1_ROOT_ENA
			(1 << 9) |			//SDC1_CLK_BRANCH_ENA
	    		(mncntr_en << 8) |		//MNCNTR_EN
	    		((mncntr_mode & 3) << 5) |	//MNCNTR_MODE
	    		((p & 3) << 3) |		//PRE_DIV_SEL
	    		(src_sel & 7);			//SRC_SEL

	writel(sdc_ns_reg, ns_reg);

    if (md_reg)
        iounmap(md_reg);
    if (ns_reg)
        iounmap(ns_reg);

    return ;
}

int __sdcc_dump_mclk(void)
{
	uint16_t d_val, n_val;
    void __iomem *md_reg = 0;
    void __iomem *ns_reg = 0;
    uint32_t raw1, raw2;
	uint16_t m, n, d;
	uint8_t p, src_sel;


    md_reg = ioremap(HWIO_SDC1_MD_REG_ADDR, 4096);
    if (!md_reg) {
        return -ENOMEM;
    }
    ns_reg = ioremap(HWIO_SDC1_NS_REG_ADDR, 4096);
    if (!ns_reg) {
        iounmap(md_reg);
        return -ENOMEM;
    }


	raw1 = readl(md_reg);
	raw2 = readl(ns_reg);
	m = raw1 >> 16;
	d = ~(raw1&0xffff);
	d /= 2;

	n_val= ~(raw2>>16);
	n = m+n_val;

	p = (raw2>>3)&0x3;
	src_sel = raw2&0x7;

	printk("sdc1: m: %u, n: %u, d: %u, p: %u, src %u\n", m, n, d, p, src_sel);


    if (md_reg)
        iounmap(md_reg);
    if (ns_reg)
        iounmap(ns_reg);

    return ;
}
#endif

#ifdef CONFIG_WIFI_CONTROL_FUNC
static struct wifi_platform_data *wifi_control_data = NULL;

static int wifi_probe( struct platform_device *pdev )
{
    struct wifi_platform_data *wifi_ctrl = (struct wifi_platform_data *)(pdev->dev.platform_data);

    printk("%s\n", __FUNCTION__);
    if( wifi_ctrl ) {
		wifi_control_data = wifi_ctrl;
    }
    return 0;
}

static int wifi_remove( struct platform_device *pdev )
{
    printk("%s\n", __FUNCTION__);
    wifi_control_data = NULL;
    return 0;
}

static struct platform_driver wifi_device = {
    .probe          = wifi_probe,
    .remove         = wifi_remove,
    .suspend        = NULL,
    .resume         = NULL,
    .driver         = {
        .name   = "msm_wifi",
    },
};


int msm_wifi_power( int on )
{
	printk("%s\n", __FUNCTION__);
	if( wifi_control_data && wifi_control_data->set_power ) {
		wifi_control_data->set_power(on);
	}
	return 0;
}

#endif

static int ti_sdio_probe(struct platform_device *pdev)
{
	printk("%s\n", __func__);
	ptr_ti_sdio_drv_data = (struct tisdio_platform_data *)(pdev->dev.platform_data);


	if (__emapi_data_buffer == NULL) {
		__emapi_data_buffer = kmalloc(SDIO_DATA_BUFFER_SIZE, GFP_ATOMIC);
		if (!__emapi_data_buffer)
			return -ENOMEM;
	}
	return 0;
}

static int ti_sdio_remove(struct platform_device *pdev)
{
	printk("%s\n", __func__);
	ptr_ti_sdio_drv_data = NULL;


	if (__emapi_data_buffer) {
		kfree(__emapi_data_buffer);
		__emapi_data_buffer = NULL;
	}

	return 0;
}

static struct platform_driver ti_sdio_drv = {
	.probe		= ti_sdio_probe,
	.remove		= ti_sdio_remove,
    .suspend        = NULL,
    .resume         = NULL,
    .driver         = {
        .name   = "ti_sdio_driver",
    },

};


/* calibration read func */
extern unsigned char *get_wifi_nvs_ram( void );
static int emapi_calibration_read(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
   	unsigned char *nvs;
	unsigned long len;

	nvs = get_wifi_nvs_ram();
	if (nvs) {
	   	/* htc specific, 0x0c: body len */
	   	memcpy(&len, nvs + 0x0c, 4);
		/* max size is 2048 */
		len = min(len+ 0x40, (unsigned long)2048);
		memcpy((void *)page, (void *) nvs, len);
		//memcpy(page, nvs, len);
		printk("%s - read %d bytes\n", __FUNCTION__, (unsigned int)len);
		return len;
	}
	else {
	   	printk("%s - no calibration data\n", __FUNCTION__);
		return -EIO;
	}
}

static int emapi_calibration_write(struct file *file, const char __user *buffer,
			   unsigned long count, void *data)
{
	printk("%s do nothing\n", __FUNCTION__);
   	return 0;
}


//------------------------------------------------------------------------------------
#define VDD_VOLTAGE_WINDOW	0xffffc0
#define SDIO_WIFI_FUNC_NUM	2

#define CHECK_ERR(status) { if ((status)) { printk(KERN_INFO "ERROR DETECT: STOP!\n"); while (1); } }

void sdio_drv_cb(void *dummy, int status){
	printk("%s: status: %d\n", __func__, status);
}


static int emapi_connect_bus(void)
{
	unsigned char  uByte, rByte;
	unsigned long  uLong;
	unsigned int   uBlkSizeShift = 9;
    unsigned int   uBlkSize = 1 << uBlkSizeShift; // 512
	int            iStatus;
	//int i;

	if (!wifi_control_data || !ptr_ti_sdio_drv_data) {
		printk("%s: driver not ready\n", __func__);
		return -1;
	}
	msm_wifi_power(1);

#ifdef _SDIO_DEBUG_
	//__sdcc_set_mclk(0x3, 0x64, 0x32, 3, 3, 0, 1);
	//printk("sdc1 clk: 144KHz\n");
	//mdelay(10);
#endif

    /* Init SDIO driver and HW */
    iStatus = ptr_ti_sdio_drv_data->sdio_connect_bus(sdio_drv_cb, NULL, uBlkSizeShift, 0);
	if (iStatus) { return iStatus; }


    /* Send commands sequence: 0, 5, 3, 7 */
    printk(KERN_INFO "[DEBUG] %s: Executing SD_IO_GO_IDLE_STATE\n", __func__);
	iStatus = ptr_ti_sdio_drv_data->sdio_execute_cmd(0, 0, MMC_RSP_NONE, &uByte, sizeof(uByte));
    CHECK_ERR(iStatus);

    printk(KERN_INFO "[DEBUG] %s: Executing CMD5\n", __func__);
    iStatus = ptr_ti_sdio_drv_data->sdio_execute_cmd(5, VDD_VOLTAGE_WINDOW, MMC_RSP_R4, &uByte, sizeof(uByte));
    CHECK_ERR(iStatus);

    printk(KERN_INFO "[DEBUG] %s: Executing SD_IO_SEND_RELATIVE_ADDR\n", __func__);
    iStatus = ptr_ti_sdio_drv_data->sdio_execute_cmd(3, 0, MMC_RSP_PRESENT/*MMC_RSP_R6*/, &uLong, sizeof(uLong));
    CHECK_ERR(iStatus);

    printk(KERN_INFO "CMD3 Read: %08X\n", (int)uLong);
    uLong = 0x10000;

    printk(KERN_INFO "[DEBUG] %s: Executing SD_IO_SELECT_CARD\n", __func__);
    iStatus = ptr_ti_sdio_drv_data->sdio_execute_cmd(7, uLong, MMC_RSP_PRESENT/*MMC_RSP_R6*/, &uByte, sizeof(uByte));
	CHECK_ERR(iStatus)

	/* set device side bus width to 4 bit (for 1 bit write 0x80 instead of 0x82) */
    uByte = 0x82;
    iStatus = ptr_ti_sdio_drv_data->sdio_write_direct_bytes(0, SDIO_CCCR_IF, &uByte, 1, 1);
    iStatus = ptr_ti_sdio_drv_data->sdio_read_direct_bytes(0, SDIO_CCCR_IF, &rByte, 1, 1);
    CHECK_ERR(iStatus);
    printk(KERN_INFO "CCR_BUS_INTERFACE_CONTROL = %02X (%02X)\n", uByte, rByte);


    /* allow function 2 */
    uByte = 4;
    iStatus = ptr_ti_sdio_drv_data->sdio_write_direct_bytes(0, SDIO_CCCR_IOEx, &uByte, 1, 1);
    iStatus = ptr_ti_sdio_drv_data->sdio_read_direct_bytes(0, SDIO_CCCR_IOEx, &rByte, 1, 1);
    CHECK_ERR(iStatus);
    printk(KERN_INFO "CCCR_IO_ENABLE = %02X (%02X)\n", uByte, rByte);



    uLong = uBlkSize;
    iStatus = ptr_ti_sdio_drv_data->sdio_write_direct_bytes(0, SDIO_FBR_BASE(SDIO_WIFI_FUNC_NUM) + SDIO_FBR_BLKSIZE, (unsigned char*)&uLong, 2, 1);
    if (iStatus) { return iStatus; }


#ifdef _SDIO_DEBUG_
	//__sdcc_set_mclk(6, 120, 60, 3, 3, 2, 1);// 15MHz
	//sdcc_set_mclk(10, 120, 60, 3, 3, 2, 1);// 25MHz
	//__sdcc_set_mclk(32, 500, 250, 3, 3, 2, 1); // 19.2 MHz
	//sdcc_set_mclk(8, 120, 60, 3, 3, 2, 1); // 20 MMHz
	//sdcc_set_mclk(1, 10, 5, 3, 3, 1, 1); // 24MHz
	//printk("sdc1 clk: 19.2MHz\n");
#endif

	return iStatus;
}

static int emapi_disconnect_bus(void)
{
	int iStatus;

	if (!wifi_control_data || !ptr_ti_sdio_drv_data) {
		printk("%s: driver not ready\n", __func__);
		return -1;
	}

    iStatus = ptr_ti_sdio_drv_data->sdio_disconnect_bus();
	msm_wifi_power(0);
	return iStatus;
}

// rw: 0 - read,  1 - write
static int ti_sdio_rw_direct(struct sdio_request *req, int rw)
{
	int iStatus = 0;

	if (!wifi_control_data || !ptr_ti_sdio_drv_data) {
		printk("%s: driver not ready\n", __func__);
		return -1;
	}

	if (!__emapi_data_buffer || SDIO_DATA_BUFFER_SIZE < req->len) {
		printk("%s: No buffer or reserve buffer len(%d) is not enough for %lu\n", __func__, SDIO_DATA_BUFFER_SIZE, req->len);
		return -1;
	}

	if (rw) {
		if(copy_from_user(__emapi_data_buffer, req->buf, req->len)) {
			iStatus = -EFAULT;
		} else {
			iStatus = ptr_ti_sdio_drv_data->sdio_write_direct_bytes(SDIO_WIFI_FUNC_NUM, req->addr, __emapi_data_buffer, req->len, 0);
		}
	} else {
		iStatus = ptr_ti_sdio_drv_data->sdio_read_direct_bytes(SDIO_WIFI_FUNC_NUM, req->addr, __emapi_data_buffer, req->len, 0);
		CHECK_ERR(iStatus);
		if (copy_to_user((uint8_t*)req->buf, __emapi_data_buffer, req->len)) {
			iStatus= -EFAULT;
		}
	}
	return iStatus;
}

static int ti_sdio_rw_ext(struct sdio_request *req, int rw)
{
	int iStatus = 0;
	unsigned char *ptr;
	int len;

	if (!wifi_control_data || !ptr_ti_sdio_drv_data) {
		printk("%s: driver not ready\n", __func__);
		return -1;
	}

	if (!__emapi_data_buffer || SDIO_DATA_BUFFER_SIZE < req->len) {
		printk("%s: No buffer or reserve buffer len(%d) is not enough for %lu\n", __func__, SDIO_DATA_BUFFER_SIZE, req->len);
		return -1;
	}

	ptr = __emapi_data_buffer;
	len = req->len;

	if (rw) {
		if(copy_from_user(__emapi_data_buffer, req->buf, len)) {
			iStatus = -EFAULT;
		} else {
			while (req->len > SDIO_BLK_SIZE) {
				ptr_ti_sdio_drv_data->sdio_write(SDIO_WIFI_FUNC_NUM, req->addr, ptr, SDIO_BLK_SIZE, 0, 1, 0);
				req->len -= SDIO_BLK_SIZE;
				ptr += SDIO_BLK_SIZE;
				req->addr+=SDIO_BLK_SIZE;
			}
			ptr_ti_sdio_drv_data->sdio_write(SDIO_WIFI_FUNC_NUM, req->addr, ptr, req->len, 0, 1, 0);
		}
	} else {
		while (req->len > SDIO_BLK_SIZE) {
			ptr_ti_sdio_drv_data->sdio_read(SDIO_WIFI_FUNC_NUM, req->addr, ptr, SDIO_BLK_SIZE, 0, 1, 0);
			req->len -= SDIO_BLK_SIZE;
			ptr += SDIO_BLK_SIZE;
			req->addr+=SDIO_BLK_SIZE;
		}
		iStatus = ptr_ti_sdio_drv_data->sdio_read(SDIO_WIFI_FUNC_NUM, req->addr, ptr, req->len, 0, 1, 0);
		CHECK_ERR(iStatus);
		if (copy_to_user((uint8_t*)req->buf, __emapi_data_buffer, len)) {
			iStatus= -EFAULT;
		}
	}
	return iStatus;
}


int emapi_control(unsigned long arg)
{
	unsigned long cmd;
	int ret = 0;

	get_user(cmd, (unsigned long *) arg);
	printk("%s: control cmd: %lu\n", __func__, cmd);

	switch (cmd) {
		case 0: // Unregister driver
			platform_driver_unregister(&wifi_device);
			platform_driver_unregister(&ti_sdio_drv);
			break;
		case 1: // register driver
			platform_driver_register(&wifi_device);
			platform_driver_register(&ti_sdio_drv);
			break;
		case 2: // connect the bus and Identify the card
			ret = emapi_connect_bus();
			break;
		case 3: // disconnect the bus and power off the chip
			ret = emapi_disconnect_bus();
			break;
	}
	return 0;
}

/* most content happens here */
static int emapi_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
   	int error = 0;
	int retval = 0;
	struct sdio_request req;


	if (_IOC_TYPE(cmd) != MAGIC)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		error = !access_ok(VERIFY_WRITE, (void __user*)arg, _IOC_SIZE(cmd));
	if (_IOC_DIR(cmd) & _IOC_WRITE)
		error = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if (error)
		return -ENOTTY;
	if (cmd != TI_EMAPI_IOC_CONTROL) {
		if (copy_from_user(&req, (void __user *) arg, sizeof(struct sdio_request)))
			return -EFAULT;
	}

	switch(cmd) {
		case TI_EMAPI_IOC_CONTROL:
		{
			retval = emapi_control(arg);
		}break;

		case TI_EMAPI_IOC_DIRECT_READ_BYTES:
		{
			ti_sdio_rw_direct(&req, 0);
		}break;

		case TI_EMAPI_IOC_DIRECT_WRITE_BYTES:
		{
			ti_sdio_rw_direct(&req, 1);
		}break;

		case TI_EMAPI_IOC_DATA_SYN_READ:
		{
			ti_sdio_rw_ext(&req, 0);
		}break;

		case TI_EMAPI_IOC_DATA_SYN_WRITE:
		{
			ti_sdio_rw_ext(&req, 1);
		}break;

		case TI_EMAPI_IOC_SDCC_CLK:
		{
#ifdef _SDIO_DEBUG_
			printk("[EMAPI_IOC_SDCC_CLK]  ");
			switch (req.addr)
			{
				case 15:
					printk("set 15MHz\n");
					__sdcc_set_mclk(6, 120, 60, 3, 3, 2, 1);
					break;
				case 20:
					printk("set 20MHz\n");
					__sdcc_set_mclk(8, 120, 60, 3, 3, 2, 1);
					break;
				case 25:
					__sdcc_set_mclk(10, 120, 60, 3, 3, 2, 1);
					printk("set 25MHz\n");
					break;
				case 19:
					__sdcc_set_mclk(32, 500, 250, 3, 3, 2, 1);
					printk("set 19.2MHz\n");
					break;

				default:
					printk("not support this clk rate\n");
					break;
			}
#else
			printk("no support\n");
#endif
		}break;

		default:
			printk("emapi: unknown command\n");
			return -ENOTTY;
	};

	return retval;
}

static int emapi_open(struct inode *inode, struct file *filp)
{
	printk("emapi is open\n");
	return nonseekable_open(inode, filp);
}

static int emapi_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static struct file_operations emapi_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= emapi_ioctl,
	.open		= emapi_open,
	.release	= emapi_release,
};

static struct miscdevice emapi_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "ti_emapi",
	.fops	= &emapi_fops,
};

static int ti_emapi_init_module(void)
{
	int rc;
	struct proc_dir_entry *calibration;

	rc = misc_register(&emapi_device);
	if (rc) {
		printk(KERN_ERR "emapi: register fail\n");
		return rc;
	}

	calibration = create_proc_entry("emapi_calibration", 0644, NULL);
	if (!calibration) {
	   	printk(KERN_ERR "emapi: alloc calibration error\n");
		rc = PTR_ERR(calibration);
		if (misc_deregister(&emapi_device))
			printk(KERN_ERR "emapi: deregister fail\n");
		return rc;
	}

	calibration->read_proc = emapi_calibration_read;
	calibration->write_proc = emapi_calibration_write;

	return 0;

}

static void ti_emapi_exit_module(void)
{

	if (misc_deregister(&emapi_device))
		printk("emapi: deregister fail\n");
}

module_init(ti_emapi_init_module);
module_exit(ti_emapi_exit_module);

MODULE_LICENSE("GPL");

