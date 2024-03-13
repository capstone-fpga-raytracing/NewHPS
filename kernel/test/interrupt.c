#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <asm/io.h>

#define RAYTRACE_BASEOFF 0x00003050

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Altera University Program");
MODULE_DESCRIPTION("DE1SoC Test IRQ");

void *lwbridgebase;

irq_handler_t irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	// Increment the value on the LEDs
	printk(KERN_INFO "Interrupt!\n");
	ioread8(lwbridgebase + RAYTRACE_BASEOFF); 
	iowrite32(ioread32(lwbridgebase) + 1, lwbridgebase);
	

	return (irq_handler_t)IRQ_HANDLED;
}

static int __init initialize_handler(void)
{
	// get the virtual addr that maps to 0xff200000
	lwbridgebase = ioremap_nocache(0xff200000, 0x200000);
	// Set LEDs to 0x200 (the leftmost LED will turn on)
	iowrite32(0x200, lwbridgebase);
	// Register the interrupt handler.
	return request_irq(73, (irq_handler_t)irq_handler, IRQF_SHARED,
					   "writeback_irq_handler", (void *)(irq_handler));
}

static void __exit cleanup_pushbutton_handler(void)
{
	// Turn off LEDs and de-register irq handler
	iowrite32(0x0, lwbridgebase);
	free_irq(73, (void *)irq_handler);
}

module_init(initialize_handler);
module_exit(cleanup_pushbutton_handler);