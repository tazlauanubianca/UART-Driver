// SPDX-License-Identifier: GPL-2.0+

/*
 * uart16550.c - UART Linux Driver
 *
 * Author: Bianca Tazlauanu tazlaunubianca30@gmail.com
 */
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/cdev.h>
#include <asm/io.h>

#include "uart16550.h"

#define DEFAULT_MAJOR			42
#define MODULE_NAME			"uart16550"
#define FIFO_SIZE			512
#define BASE_PORT_COM1			0x3f8
#define BASE_PORT_COM2			0x2f8
#define NUM_PORTS			8
#define COM1				"com1"
#define COM2				"com2"
#define IRQ_NUM_COM1			4
#define IRQ_NUM_COM2			3

#define FCR_OFFSET			2
#define MCR_OFFSET			4
#define LCR_OFFSET			3
#define IIR_OFFSET			2
#define IER_OFFSET			1
#define ENABLE_FIFO			0x07
#define IER_ENABLE_READ_WRITE		0x03
#define MCR_ENABLE_INTERRUPTS		0x08
#define DISABLE_WRITES_MASK		0xfd
#define DISABLE_READS_MASK		0xfe
#define ENABLE_READS_MASK		0x01
#define ENABLE_WRITES_MASK		0x02
#define LOW_NIBBLE_MASK			0x0f
#define TIMEOUT_INTERRUPT_PENDING	12
#define TRANSMITTER_INTERRUPT		2
#define	RECEIVED_DATA_INTERRUPT		4
#define LATCH_LOW_MASK			0xff
#define LATCH_HIGH_MASK			0x08
#define DLAB_SET			0x80
#define DLAB_CLEAR			0x00

MODULE_DESCRIPTION("UART Driver");
MODULE_AUTHOR("Tazlauanu Bianca");
MODULE_LICENSE("GPL");

static int major = DEFAULT_MAJOR;
static int option = OPTION_BOTH;

module_param(major, int, 0);
module_param(option, int, 0);

static int uart16550_open(struct inode *inode, struct file *file);
static int uart16550_read(struct file *file, char __user *user_buffer,
		size_t size, loff_t *offset);
static int uart16550_write(struct file *file, const char *user_buffer,
		size_t size, loff_t *offset);
static int uart16550_release(struct inode *inode, struct file *file);
static long uart16550_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg);
/*
 * Structure used for managing one serial port.
 */
struct uart16550_cdev {
	struct cdev cdev;
	char activated;
	char *name;
	int base_port;
	int irq_num;
	atomic_t can_read;
	atomic_t can_write;
	wait_queue_head_t wq_read;
	wait_queue_head_t wq_write;
	DECLARE_KFIFO(fifo_write, unsigned char, FIFO_SIZE);
	DECLARE_KFIFO(fifo_read, unsigned char, FIFO_SIZE);
};

/*
 * Structure with operations for the device.
 */
static const struct file_operations uart16550_fops = {
	.owner		= THIS_MODULE,
	.open		= uart16550_open,
	.read		= uart16550_read,
	.write		= uart16550_write,
	.release	= uart16550_release,
	.unlocked_ioctl	= uart16550_ioctl
};

static struct uart16550_cdev devices[MAX_NUMBER_DEVICES];
static int number_minors;

/*
 * Interrupt handler for the serial port.
 */
irqreturn_t irq_handler(int irq_no, void *dev_id)
{
	struct uart16550_cdev *device = (struct uart16550_cdev *)dev_id;
	char irq = inb(device->base_port + IIR_OFFSET);
	char low_nibble_irq = irq & LOW_NIBBLE_MASK;
	char data_read, data_write;

	/* Request to write to UART */
	if (low_nibble_irq == TRANSMITTER_INTERRUPT) {
		/* If the fifo buffer is not empty, write one character
		 * from the write buffer to UART
		 */
		if (!kfifo_is_empty(&device->fifo_write)) {
			kfifo_out(&device->fifo_write, &data_write, 1);
			outb(data_write, device->base_port);
			atomic_set(&device->can_write, 1);
			wake_up_interruptible(&device->wq_write);
		} else {
			outb(inb(device->base_port + IER_OFFSET) &
				DISABLE_WRITES_MASK,
				device->base_port + IER_OFFSET);
		}
	}

	/* Request to read from UART */
	if ((low_nibble_irq == TIMEOUT_INTERRUPT_PENDING) ||
		(low_nibble_irq == RECEIVED_DATA_INTERRUPT)) {
		/* If the fifo buffer is not full, read one character
		 * from UART and store it in the buffer
		 */
		if (!kfifo_is_full(&device->fifo_read)) {
			data_read = inb(device->base_port);
			kfifo_in(&device->fifo_read, &data_read, 1);
			atomic_set(&device->can_read, 1);
			wake_up_interruptible(&device->wq_read);
		} else {
			outb(inb(device->base_port + IER_OFFSET) &
					DISABLE_READS_MASK,
					device->base_port + IER_OFFSET);
		}
	}

	return IRQ_HANDLED;
}

static int uart16550_open(struct inode *inode, struct file *file)
{
	struct uart16550_cdev *device = container_of(inode->i_cdev,
					struct uart16550_cdev, cdev);
	file->private_data = device;

	return 0;
}

/*
 * Read function called from user space to
 * get data that came from UART.
 */
static int uart16550_read(struct file *file, char __user *user_buffer,
		size_t size, loff_t *offset)
{
	struct uart16550_cdev *device =
		(struct uart16550_cdev *)file->private_data;
	int size_fifo;
	int num_read_bytes = 0;
	char buffer[FIFO_SIZE] = {0};

	/* If there is no data to read, wait */
	if (kfifo_is_empty(&device->fifo_read)) {
		outb(inb(device->base_port + IER_OFFSET) | ENABLE_READS_MASK,
				device->base_port + IER_OFFSET);
		wait_event_interruptible(device->wq_read,
				atomic_read(&device->can_read) != 0);
	}

	/* Send as much data as possible to the user */
	size_fifo = kfifo_len(&device->fifo_read);
	num_read_bytes = size > size_fifo ? size_fifo : size;
	kfifo_out(&device->fifo_read, buffer, num_read_bytes);

	if (copy_to_user(user_buffer, buffer, num_read_bytes))
		return -EFAULT;
	atomic_set(&device->can_read, 0);

	return num_read_bytes;
}

/*
 * Write function called from user space to store
 * data that will be transferred to UART.
 */
static int uart16550_write(struct file *file, const char *user_buffer,
			size_t size, loff_t *offset)
{
	struct uart16550_cdev *device =
		(struct uart16550_cdev *)file->private_data;
	int size_fifo;
	int num_write_bytes = 0;
	char buffer[FIFO_SIZE] = {0};

	/* If there is no space to write the data, wait */
	if (kfifo_is_full(&device->fifo_write)) {
		wait_event_interruptible(device->wq_write,
				atomic_read(&device->can_write) != 0);
	}

	/* Write ar much data from the user in the buffer */
	size_fifo = kfifo_avail(&device->fifo_write);
	num_write_bytes = size > size_fifo ? size_fifo : size;
	if (copy_from_user(buffer, user_buffer, num_write_bytes))
		return -EFAULT;
	kfifo_in(&device->fifo_write, buffer, num_write_bytes);

	atomic_set(&device->can_write, 0);
	outb(inb(device->base_port + IER_OFFSET) | ENABLE_WRITES_MASK,
			device->base_port + IER_OFFSET);

	return num_write_bytes;
}

static int uart16550_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long uart16550_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct uart16550_cdev *device =
		(struct uart16550_cdev *)file->private_data;
	struct uart16550_line_info info;

	switch (cmd) {
	case UART16550_IOCTL_SET_LINE:
		if (copy_from_user(&info, (void *)arg,
				sizeof(struct uart16550_line_info)))
			return -EFAULT;

		/* Set the parameters for the transfer for UART */
		outb(DLAB_SET, device->base_port + LCR_OFFSET);
		outb(info.baud >> LATCH_HIGH_MASK,
					device->base_port + IER_OFFSET);
		outb(info.baud & LATCH_LOW_MASK, device->base_port);
		outb(DLAB_CLEAR, device->base_port + LCR_OFFSET);
		outb(info.len | info.par | info.stop,
					device->base_port + LCR_OFFSET);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * Function to set up the device structure.
 * @device_index: the index in the devices array
 * @base_port: base port addr of the device
 * @name: name of the device
 * @irq_num: number of the IRQ for the device
 */
static void setup_device_structure(int device_index,
		int base_port, char *name, int irq_num)
{
	devices[device_index].activated = 1;
	devices[device_index].base_port = base_port;
	devices[device_index].name = name;
	devices[device_index].irq_num = irq_num;
	number_minors++;
}

static int uart16550_init(void)
{
	int err, i, j;

	number_minors = 0;
	for (i = 0; i < MAX_NUMBER_DEVICES; i++)
		devices[i].activated = 0;

	if ((option & OPTION_COM1) != 0)
		setup_device_structure(0, BASE_PORT_COM1, COM1, IRQ_NUM_COM1);

	if ((option & OPTION_COM2) != 0)
		setup_device_structure(1, BASE_PORT_COM2, COM2, IRQ_NUM_COM2);

	if (option == OPTION_COM2)
		err = register_chrdev_region(MKDEV(major, 1),
				number_minors, MODULE_NAME);
	else
		err = register_chrdev_region(MKDEV(major, 0),
				number_minors, MODULE_NAME);
	if (err != 0)
		return err;

	for (i = 0; i < MAX_NUMBER_DEVICES; i++) {
		if (devices[i].activated) {
			/* Register device */
			cdev_init(&devices[i].cdev, &uart16550_fops);
			cdev_add(&devices[i].cdev, MKDEV(major, i), 1);

			/* Initialise structures */
			INIT_KFIFO(devices[i].fifo_read);
			INIT_KFIFO(devices[i].fifo_write);
			init_waitqueue_head(&devices[i].wq_read);
			init_waitqueue_head(&devices[i].wq_write);
			atomic_set(&devices[i].can_read, 0);
			atomic_set(&devices[i].can_write, 0);

			/* Request access to ports */
			if (!request_region(devices[i].base_port,
						NUM_PORTS, MODULE_NAME)) {
				err = -ENODEV;
				goto release_devices;
			}

			/* Request IRQ handler */
			err = request_irq(devices[i].irq_num, irq_handler,
					IRQF_SHARED, MODULE_NAME, &devices[i]);
			if (err < 0) {
				release_region(devices[i].base_port, NUM_PORTS);
				goto release_devices;
			}

			/* Enable handler for UART */
			outb(MCR_ENABLE_INTERRUPTS,
					devices[i].base_port + MCR_OFFSET);
			outb(IER_ENABLE_READ_WRITE,
					devices[i].base_port + IER_OFFSET);

			/* Enable and clear FIFO for UART */
			outb(ENABLE_FIFO, devices[i].base_port + FCR_OFFSET);
		}
	}

	return 0;

release_devices:
	if (option == OPTION_COM2)
		unregister_chrdev_region(MKDEV(major, 1), number_minors);
	else
		unregister_chrdev_region(MKDEV(major, 0), number_minors);

	for (j = 0; j < i; j++) {
		if (devices[j].activated) {
			cdev_del(&devices[j].cdev);
			release_region(devices[j].base_port, NUM_PORTS);
			free_irq(devices[j].irq_num, &devices[i]);
		}
	}

	return err;
}

static void uart16550_exit(void)
{
	int i;

	/* Unregister device */
	if (option == OPTION_COM2)
		unregister_chrdev_region(MKDEV(major, 1), number_minors);
	else
		unregister_chrdev_region(MKDEV(major, 0), number_minors);

	/* Release ports and device, IRQ handlers */
	for (i = 0; i < MAX_NUMBER_DEVICES; i++) {
		if (devices[i].activated) {
			cdev_del(&devices[i].cdev);
			release_region(devices[i].base_port, NUM_PORTS);
			free_irq(devices[i].irq_num, &devices[i]);
		}
	}
}

module_init(uart16550_init);
module_exit(uart16550_exit);
