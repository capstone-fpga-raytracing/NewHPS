#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/interrupt.h>
#include <linux/spinlock_types.h>

#define RTINTR_BASE 0xff203050
#define RTINTR_SIZE 4
#define RTINTR_IRQ 73

void *rtmem;

static DEFINE_SEMAPHORE(intr_mutex);
static DECLARE_WAIT_QUEUE_HEAD(intr_wq);

static int intr_flag = 0;
static uint8_t end_rtstat;
static DEFINE_SPINLOCK(intr_flag_lock);

static irqreturn_t fpga_rt_interrupt(int irq, void *dev_id)
{
    (void)dev_id;
    if (irq != RTINTR_IRQ)
        return IRQ_NONE;

    spin_lock(&intr_flag_lock);
    intr_flag = 1;
    end_rtstat = ioread8(rtmem);
    spin_unlock(&intr_flag_lock);

    wake_up_interruptible(&intr_wq);

    return IRQ_HANDLED;
}

static struct device_driver fpga_rtintr_driver = {
    .name = "fpga_rtintr",
    .bus = &platform_bus_type,
};

static ssize_t fpga_rtintr_show(struct device_driver *drv, char *buf)
{
    (void)drv;
    if (down_trylock(&intr_mutex))
        return -EAGAIN;

    if (wait_event_interruptible(intr_wq, intr_flag != 0)) {
        up(&intr_mutex);
        return -ERESTART;
    }

    spin_lock(&intr_flag_lock);
    intr_flag = 0;
    spin_unlock(&intr_flag_lock);

    buf[0] = end_rtstat;
    up(&intr_mutex);
    return 1;
}

static ssize_t fpga_rtintr_store(struct device_driver *drv, const char *buf, size_t count)
{
    (void)drv; (void)buf; (void)count;
    return -EROFS;
}

static DRIVER_ATTR(fpga_rtintr, S_IRUSR, fpga_rtintr_show, fpga_rtintr_store);

static int __init fpga_rtintr_init(void)
{
    int ret;
    struct resource* res;

    ret = driver_register(&fpga_rtintr_driver);
    if (ret < 0) { return ret; }

    // make sysfs file
    ret = driver_create_file(&fpga_rtintr_driver, &driver_attr_fpga_rtintr);
    if (ret < 0) { goto fail_sysfs; }

    res = request_mem_region(RTINTR_BASE, RTINTR_SIZE, "fpga_rtintr");
    if (res == NULL) {
        ret = -EBUSY;
        goto fail_rqmem;
    }

    rtmem = ioremap(RTINTR_BASE, RTINTR_SIZE);
    if (rtmem == NULL) {
        ret = -EFAULT;
        goto fail_ioremap;
    }

    ret = request_irq(RTINTR_IRQ, fpga_rt_interrupt, 0, "fpga_rtintr", NULL);
    if (ret < 0) { goto fail_rqirq; }

    return 0;

fail_rqirq:
    iounmap(rtmem);
fail_ioremap:
    release_mem_region(RTINTR_BASE, RTINTR_SIZE);
fail_rqmem:
    driver_remove_file(&fpga_rtintr_driver, &driver_attr_fpga_rtintr);
fail_sysfs:
    driver_unregister(&fpga_rtintr_driver);
fail_driver_register:
    return ret;
}

static void __exit fpga_rtintr_exit(void)
{
    free_irq(RTINTR_IRQ, NULL);
    iounmap(rtmem);
    release_mem_region(RTINTR_BASE, RTINTR_SIZE);
    driver_remove_file(&fpga_rtintr_driver, &driver_attr_fpga_rtintr);
    driver_unregister(&fpga_rtintr_driver);
}

MODULE_LICENSE("BSD");

module_init(fpga_rtintr_init);
module_exit(fpga_rtintr_exit);
