#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#define LW_BRIDGE_BASE 0xFF200000
#define SDR_IRQ_OFF 0x00000010

void *sdr_irq_mem;
static uint8_t input_state;

static irqreturn_t sdr_irq_interrupt(int irq, void *dev_id)
{
	if (irq != UINPUT_INT_NUM)
		return IRQ_NONE;

	input_state = ioread8(sdr_irq_mem);

	return IRQ_HANDLED;
}

static int __init sdr_irq_init(void)
{
	int ret;
	struct resource *res;

	res = request_mem_region(LW_BRIDGE_BASE + SDR_IRQ_OFF, 1, "sdr_irq");
	if (res == NULL) {
		ret = -EBUSY;
		goto fail_request_mem;
	}

	sdr_irq_mem = ioremap(LW_BRIDGE_BASE + SDR_IRQ_OFF, 1);
	if (sdr_irq_mem == NULL) {
		ret = -EFAULT;
		goto fail_ioremap;
	}

	ret = request_irq(73, sdr_irq_interrupt,
			0, "sdr_irq", NULL);
	if (ret < 0)
		goto fail_request_irq;

	return 0;

fail_request_irq:
	iounmap(sdr_irq_mem);
fail_ioremap:
	release_mem_region(LW_BRIDGE_BASE + SDR_IRQ_OFF, 1);
fail_request_mem:
	return ret;
}

static void __exit sdr_irq_exit(void)
{
	free_irq(73, NULL);
	iounmap(sdr_irq_mem);
	release_mem_region(LW_BRIDGE_BASE + SDR_IRQ_OFF, 1);
	driver_remove_file(&sdr_irq_driver, &driver_attr_sdr_irq);
	driver_unregister(&sdr_irq_driver);
}

MODULE_LICENSE("Dual BSD/GPL");

module_init(sdr_irq_init);
module_exit(sdr_irq_exit);