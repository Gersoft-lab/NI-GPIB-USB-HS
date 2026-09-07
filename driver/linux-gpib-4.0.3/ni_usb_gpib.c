/***************************************************************************
                          ni_usb/ni_usb_gpib.c
                             -------------------
 driver for National Instruments usb to gpib adapters

    begin                : 2004-05-29
    copyright            : (C) 2004 by Frank Mori Hess
    email                : fmhess@users.sourceforge.net
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/bitrev.h>
#include "ni_usb_gpib.h"
#include "gpibP.h"
#include "nec7210.h"
#include "tnt4882_registers.h"

MODULE_LICENSE("GPL");

#define MAX_NUM_NI_USB_INTERFACES 128
static struct usb_interface *ni_usb_driver_interfaces[MAX_NUM_NI_USB_INTERFACES];

static int ni_usb_parse_status_block(const uint8_t *buffer, struct ni_usb_status_block *status);
static int ni_usb_firmware_gts(gpib_board_t *board);
int ni_usb_command_chunk(gpib_board_t *board, uint8_t *buffer, size_t length, size_t *command_bytes_written);
static int ni_usb_set_interrupt_monitor(gpib_board_t *board, unsigned int monitored_bits);
static void ni_usb_stop(ni_usb_private_t *ni_priv);

static DEFINE_MUTEX(ni_usb_hotplug_lock);

/* Skip-ppoll workaround: when nonzero, ni_usb_parallel_poll returns this
 * value without touching the bus.  HPDir tests a drive's PP line
 * microseconds after addressing it -- winnable over PCI, structurally lost
 * over USB (one round trip later the drive has already released its line).
 * Against an always-ready emulated drive (Gesswein), forcing the response
 * is semantically sound.  Runtime tunable (drive at address 2):
 * echo 32 | sudo tee /sys/module/ni_usb_gpib/parameters/rpp_force */
static int rpp_force = 0;
module_param(rpp_force, int, 0644);
MODULE_PARM_DESC(rpp_force, "forced parallel poll response (0 = real poll)");

/* with rpp_force: make the first poll after a data write return 0 (busy),
 * then the forced mask — emulates the busy->ready transition that some
 * tools (e.g. HPDir's write loop) wait for and that a constant forced
 * value can never produce. */
static int rpp_force_transition = 0;
module_param(rpp_force_transition, int, 0644);
MODULE_PARM_DESC(rpp_force_transition, "emulate busy->ready ppoll transition after data writes");

//calculates a reasonable timeout in that can be passed to usb functions
static inline unsigned long ni_usb_timeout_msecs(unsigned int usec)
{
	if(usec == 0) return 0;
	return 2000 + usec / 500;
};
// returns timeout code byte for use in ni-usb-b instructions
static unsigned short ni_usb_timeout_code(unsigned int usec)
{
	if( usec == 0 ) return 0xf0;
	else if( usec <= 10 ) return 0xf1;
	else if( usec <= 30 ) return 0xf2;
	else if( usec <= 100 ) return 0xf3;
	else if( usec <= 300 ) return 0xf4;
	else if( usec <= 1000 ) return 0xf5;
	else if( usec <= 3000 ) return 0xf6;
	else if( usec <= 10000 ) return 0xf7;
	else if( usec <= 30000 ) return 0xf8;
	else if( usec <= 100000 ) return 0xf9;
	else if( usec <= 300000 ) return 0xfa;
	else if( usec <= 1000000 ) return 0xfb;
	else if( usec <= 3000000 ) return 0xfc;
	else if( usec <= 10000000 ) return 0xfd;
	else if( usec <= 30000000 ) return 0xfe;
	else if( usec <= 100000000 ) return 0xff;
	else if( usec <= 300000000 ) return 0x01;
	/* NI driver actually uses 0xff for timeout T1000s, which is a bug in their code.
	 * I've verified on a usb-b that a code of 0x2 is correct for a 1000 sec timeout */
	else if( usec <= 1000000000 ) return 0x02;
	else
	{
		printk("%s: bug? usec is greater than 1e9\n", __FILE__);
		return 0xf0;
	}
};

static void ni_usb_bulk_complete(struct urb *urb PT_REGS_ARG)
{
	ni_usb_urb_context_t *context = urb->context;

//	printk("debug: %s: %s: status=0x%x, error_count=%i, actual_length=%i\n", __FILE__, __FUNCTION__,
//		urb->status, urb->error_count, urb->actual_length);
	up(&context->complete);
}

static void ni_usb_timeout_handler(unsigned long arg)
{
	ni_usb_urb_context_t *context = (ni_usb_urb_context_t *) arg;
	context->timed_out = 1;
	up(&context->complete);
};

// I'm using nonblocking loosely here, it only means -EAGAIN can be returned in certain cases
int ni_usb_nonblocking_send_bulk_msg(ni_usb_private_t *ni_priv, void *data, int data_length, int *actual_data_length, int timeout_msecs)
{
	struct usb_device *usb_dev;
	int retval;
	unsigned int out_pipe;
	ni_usb_urb_context_t context;
	struct timer_list timer;

	*actual_data_length = 0;
	mutex_lock(&ni_priv->bulk_transfer_lock);
	if(ni_priv->bus_interface == NULL)
	{
		mutex_unlock(&ni_priv->bulk_transfer_lock);
		return -ENODEV;
	}
	if(ni_priv->bulk_urb)
	{
		mutex_unlock(&ni_priv->bulk_transfer_lock);
		return -EAGAIN;
	}
	ni_priv->bulk_urb = usb_alloc_urb(0, GFP_KERNEL);
	if(ni_priv->bulk_urb == NULL)
	{
		mutex_unlock(&ni_priv->bulk_transfer_lock);
		return -ENOMEM;
	}
	usb_dev = interface_to_usbdev(ni_priv->bus_interface);
	out_pipe = usb_sndbulkpipe(usb_dev, ni_priv->bulk_out_endpoint);
	sema_init(&context.complete, 0);
	context.timed_out = 0;
	usb_fill_bulk_urb(ni_priv->bulk_urb, usb_dev, out_pipe, data, data_length,
		&ni_usb_bulk_complete, &context);
	init_timer(&timer);
	if(timeout_msecs)
	{
		timer.expires = jiffies + msecs_to_jiffies(timeout_msecs);
		timer.function = ni_usb_timeout_handler;
		timer.data = (unsigned long) &context;
		add_timer(&timer);
	}
	//printk("%s: submitting urb\n", __FUNCTION__);
	retval = usb_submit_urb(ni_priv->bulk_urb, GFP_KERNEL);
	if(retval)
	{
		if(timer_pending(&timer))
			del_timer_sync(&timer);
		usb_free_urb(ni_priv->bulk_urb);
		ni_priv->bulk_urb = NULL;
		printk("%s: failed to submit bulk out urb, retval=%i\n", __FILE__, retval);
		mutex_unlock(&ni_priv->bulk_transfer_lock);
		return retval;
	}
	mutex_unlock(&ni_priv->bulk_transfer_lock);
	down(&context.complete);
	if(context.timed_out)
	{
		usb_kill_urb(ni_priv->bulk_urb);
		printk("%s: killed urb due to timeout\n", __FUNCTION__);
		retval = -ETIMEDOUT;
	}else
		retval = ni_priv->bulk_urb->status;
	if(timer_pending(&timer))
		del_timer_sync(&timer);
	*actual_data_length = ni_priv->bulk_urb->actual_length;
	mutex_lock(&ni_priv->bulk_transfer_lock);
	usb_free_urb(ni_priv->bulk_urb);
	ni_priv->bulk_urb = NULL;
	mutex_unlock(&ni_priv->bulk_transfer_lock);
	return retval;
}

/* When a response was orphaned or misparsed, the firmware's bulk-in queue is
 * shifted and every subsequent transfer reads the wrong reply (poisoning all
 * following operations, e.g. ppoll reading a register-read response).
 * Before the next request goes out, ask the firmware to stop whatever is
 * pending and scoop all stale bulk-in data. */
static void ni_usb_resync_pipe(ni_usb_private_t *ni_priv)
{
	struct usb_device *usb_dev;
	unsigned int in_pipe;
	uint8_t *scratch;
	int n, got, r, total = 0;

	if(ni_priv->bus_interface == NULL) return;
	usb_dev = interface_to_usbdev(ni_priv->bus_interface);
	in_pipe = usb_rcvbulkpipe(usb_dev, ni_priv->bulk_in_endpoint);
	ni_usb_stop(ni_priv);
	scratch = kmalloc(0x200, GFP_KERNEL);
	if(scratch == NULL) return;
	for(n = 0; n < 8; n++)
	{
		got = 0;
		r = usb_bulk_msg(usb_dev, in_pipe, scratch, 0x200, &got, 300);
		if(r || got <= 0) break;
		total += got;
	}
	kfree(scratch);
	ni_priv->pipe_dirty = 0;
	printk("ni_usb_gpib: pipe resync: drained %i stale byte(s)\n", total);
}

static int ni_usb_send_bulk_msg(ni_usb_private_t *ni_priv, void *data, int data_length, int *actual_data_length, int timeout_msecs)
{
	int retval;
	int timeout_msecs_remaining = timeout_msecs;
	if(ni_priv->pipe_dirty)
		ni_usb_resync_pipe(ni_priv);
	retval = ni_usb_nonblocking_send_bulk_msg(ni_priv, data, data_length, actual_data_length, timeout_msecs_remaining);
	while(retval == -EAGAIN && (timeout_msecs == 0 || timeout_msecs_remaining > 0))
	{
		msleep(1);
		retval = ni_usb_nonblocking_send_bulk_msg(ni_priv, data, data_length, actual_data_length, timeout_msecs_remaining);
		if(timeout_msecs != 0) --timeout_msecs_remaining;
	}
	if(timeout_msecs != 0 && timeout_msecs_remaining <= 0) return -ETIMEDOUT;
	return retval;
}

// I'm using nonblocking loosely here, it only means -EAGAIN can be returned in certain cases
int ni_usb_nonblocking_receive_bulk_msg(ni_usb_private_t *ni_priv,
	void *data, int data_length, int *actual_data_length, int timeout_msecs,
	int interruptible)
{
	struct usb_device *usb_dev;
	int retval;
	unsigned int in_pipe;
	ni_usb_urb_context_t context;
	struct timer_list timer;
	int urb_killed = 0;

	*actual_data_length = 0;
	mutex_lock(&ni_priv->bulk_transfer_lock);
	if(ni_priv->bus_interface == NULL)
	{
		mutex_unlock(&ni_priv->bulk_transfer_lock);
		return -ENODEV;
	}
	if(ni_priv->bulk_urb)
	{
		mutex_unlock(&ni_priv->bulk_transfer_lock);
		return -EAGAIN;
	}
	ni_priv->bulk_urb = usb_alloc_urb(0, GFP_KERNEL);
	if(ni_priv->bulk_urb == NULL)
	{
		mutex_unlock(&ni_priv->bulk_transfer_lock);
		return -ENOMEM;
	}
	usb_dev = interface_to_usbdev(ni_priv->bus_interface);
	in_pipe = usb_rcvbulkpipe(usb_dev, ni_priv->bulk_in_endpoint);
	sema_init(&context.complete, 0);
	context.timed_out = 0;
	usb_fill_bulk_urb(ni_priv->bulk_urb, usb_dev, in_pipe, data, data_length,
		&ni_usb_bulk_complete, &context);
	init_timer(&timer);
	if(timeout_msecs)
	{
		timer.expires = jiffies + msecs_to_jiffies(timeout_msecs);
		timer.function = ni_usb_timeout_handler;
		timer.data = (unsigned long) &context;
		add_timer(&timer);
	}
	//printk("%s: submitting urb\n", __FUNCTION__);
	retval = usb_submit_urb(ni_priv->bulk_urb, GFP_KERNEL);
	if(retval)
	{
		if(timer_pending(&timer))
			del_timer_sync(&timer);
		usb_free_urb(ni_priv->bulk_urb);
		ni_priv->bulk_urb = NULL;
		printk("%s: failed to submit bulk out urb, retval=%i\n", __FILE__, retval);
		mutex_unlock(&ni_priv->bulk_transfer_lock);
		return retval;
	}
	mutex_unlock(&ni_priv->bulk_transfer_lock);
	if(interruptible)
	{
		if(down_interruptible(&context.complete))
		{
			/* If we got interrupted by a signal while waiting for the usb gpib
				to respond, we should send a stop command so it will finish
				up with whatever it was doing and send its response now. */ 
			ni_usb_stop(ni_priv);
			retval = -ERESTARTSYS;
			/* now do an uninterruptible wait, it shouldn't take long
				for the board to respond now. */
			down(&context.complete);
		}
	}else
	{
		down(&context.complete);
	}
	if(context.timed_out)
	{
		/* Ask the firmware to abort and post its (aborted) response now,
		 * instead of killing the urb and leaving an orphaned response in
		 * the bulk-in pipe — that desyncs every transfer that follows
		 * (responses shifted by one transaction).  Mirrors the signal
		 * path above. */
		ni_usb_stop(ni_priv);
		if(down_timeout(&context.complete, msecs_to_jiffies(1000)) == 0)
		{
			printk("%s: timeout, firmware aborted cleanly (bulk-in pipe kept in sync)\n",
				__FUNCTION__);
		}else
		{
			usb_kill_urb(ni_priv->bulk_urb);
			printk("%s: killed urb due to timeout\n", __FUNCTION__);
			urb_killed = 1;
			ni_priv->pipe_dirty = 1;
		}
		retval = -ETIMEDOUT;
	}else if(ni_priv->bulk_urb->status)
		retval = ni_priv->bulk_urb->status;
	if(timer_pending(&timer))
		del_timer_sync(&timer);
	*actual_data_length = ni_priv->bulk_urb->actual_length;
	mutex_lock(&ni_priv->bulk_transfer_lock);
	usb_free_urb(ni_priv->bulk_urb);
	ni_priv->bulk_urb = NULL;
	mutex_unlock(&ni_priv->bulk_transfer_lock);
	if(urb_killed)
	{
		/* the firmware may still deliver the orphaned response later;
		 * scoop it synchronously so the pipe stays usable */
		uint8_t *scratch = kmalloc(0x200, GFP_KERNEL);
		if(scratch)
		{
			int n, got, r;
			for(n = 0; n < 4; n++)
			{
				got = 0;
				r = usb_bulk_msg(usb_dev, in_pipe, scratch, 0x200, &got, 300);
				if(r || got <= 0) break;
				printk("ni_usb_gpib: drained %i byte(s) of late response after killed urb\n", got);
			}
			kfree(scratch);
		}
	}
	return retval;
}

static int ni_usb_receive_bulk_msg(ni_usb_private_t *ni_priv, void *data, 
	int data_length, int *actual_data_length, int timeout_msecs, int interruptible)
{
	int retval;
	int timeout_msecs_remaining = timeout_msecs;
	retval = ni_usb_nonblocking_receive_bulk_msg(ni_priv, data, data_length, 
		actual_data_length, timeout_msecs_remaining, interruptible);
	while(retval == -EAGAIN && (timeout_msecs == 0 || timeout_msecs_remaining > 0))
	{
		msleep(1);
		retval = ni_usb_nonblocking_receive_bulk_msg(ni_priv, data, data_length, 
			actual_data_length, timeout_msecs_remaining, interruptible);
		if(timeout_msecs != 0) --timeout_msecs_remaining;
	}
	if(timeout_msecs && timeout_msecs_remaining <= 0) return -ETIMEDOUT;
	return retval;
}

int ni_usb_receive_control_msg(ni_usb_private_t *ni_priv, __u8 request, __u8 requesttype, __u16 value, __u16 index,
	void *data, __u16 size, int timeout_msecs)
{
	struct usb_device *usb_dev;
	int retval;
	unsigned int in_pipe;

	mutex_lock(&ni_priv->control_transfer_lock);
	if(ni_priv->bus_interface == NULL)
	{
		mutex_unlock(&ni_priv->control_transfer_lock);
		return -ENODEV;
	}
	usb_dev = interface_to_usbdev(ni_priv->bus_interface);
	in_pipe = usb_rcvctrlpipe(usb_dev, 0);
	retval = USB_CONTROL_MSG(usb_dev, in_pipe, request, requesttype, value, index, data, size, timeout_msecs);
	mutex_unlock(&ni_priv->control_transfer_lock);
	return retval;
}

void ni_usb_soft_update_status(gpib_board_t *board, unsigned int ni_usb_ibsta, unsigned int clear_mask)
{
	ni_usb_private_t *ni_priv = board->private_data;
	static const unsigned int ni_usb_ibsta_mask = SRQI | ATN | CIC | REM | LACS | TACS | LOK;
	unsigned int need_monitoring_bits;
	unsigned long flags;

	board->status &= ~clear_mask;
	board->status &= ~ni_usb_ibsta_mask;
	board->status |= ni_usb_ibsta & ni_usb_ibsta_mask;
//	if(ni_usb_ibsta & ~ni_usb_ibsta_mask)
//	{
//		printk("%s: debug: ibsta from ni gpib usb adapter is 0x%x\n", __FILE__, ni_usb_ibsta);
//	}
	//FIXME should generate events on DTAS and DCAS
	need_monitoring_bits = ni_usb_ibsta_monitor_mask;
	spin_lock_irqsave(&board->spinlock, flags);
	ni_priv->monitored_ibsta_bits &= ~ni_usb_ibsta;
	need_monitoring_bits &= ~ni_priv->monitored_ibsta_bits;
	spin_unlock_irqrestore(&board->spinlock, flags);
	if(need_monitoring_bits & ~ni_usb_ibsta)
	{
		ni_usb_set_interrupt_monitor(board, ni_usb_ibsta_monitor_mask);
	}else if(need_monitoring_bits & ni_usb_ibsta)
	{
		wake_up_interruptible( &board->wait );
	}
// 	printk("%s: ni_usb_ibsta=0x%x\n", __FUNCTION__, ni_usb_ibsta);
	return;
}

static int ni_usb_parse_status_block(const uint8_t *buffer, struct ni_usb_status_block *status)
{
	uint16_t count;

	status->id = buffer[0];
	status->ibsta = (buffer[1] << 8) | buffer[2];
	status->error_code = buffer[3];
	count = buffer[4] | (buffer[5] << 8);
	count = ~count;
	count++;
	status->count = count;
	return 8;
};

static void ni_usb_dump_raw_block(const uint8_t *raw_data, int length)
{
	int i;

	printk("%s:", __FUNCTION__);
	for(i = 0; i < length; i++)
	{
		if(i % 8 == 0)
			printk("\n");
		printk(" %2x", raw_data[i]);
	}
	printk("\n");
}

int ni_usb_parse_register_read_block(const uint8_t *raw_data, unsigned int *results, int num_results)
{
	int i = 0;
	int j;
	int unexpected = 0;
	static const int results_per_chunk = 3;
	for(j = 0; j < num_results;)
	{
		int k;
		if(raw_data[i++] != NIUSB_REGISTER_READ_DATA_START_ID)
		{
			printk("%s: %s: parse error: wrong start id\n", __FILE__, __FUNCTION__);
			unexpected = 1;
		}
		for(k = 0; k < results_per_chunk && j < num_results; ++k)
		{
			results[j++] = raw_data[i++];
		}
	}
	while(i % 4)
	{
		i++;
	}
	if(raw_data[i++] != NIUSB_REGISTER_READ_DATA_END_ID)
	{
		printk("%s: %s: parse error: wrong end id\n", __FILE__, __FUNCTION__);
		unexpected = 1;
	}
	if(raw_data[i++] % results_per_chunk != num_results % results_per_chunk)
	{
		printk("%s: %s: parse error: wrong count=%i for NIUSB_REGISTER_READ_DATA_END\n",
			__FILE__, __FUNCTION__, (int)raw_data[i - 1]);
		unexpected = 1;
	}
	while(i % 4)
	{
		if(raw_data[i++] != 0)
		{
			printk("%s: %s: unexpected data: raw_data[%i]=0x%x, expected 0\n",
				__FILE__, __FUNCTION__, i - 1, (int)raw_data[i - 1]);
			unexpected = 1;
		}
	}
	if(unexpected)
		ni_usb_dump_raw_block(raw_data, i);
	return i;
}

int ni_usb_parse_termination_block(const uint8_t *buffer)
{
	int i = 0;

	if(buffer[i++] != NIUSB_TERM_ID ||
		buffer[i++] != 0x0 ||
		buffer[i++] != 0x0 ||
		buffer[i++] != 0x0)
	{
		printk("%s: received unexpected termination block\n" , __FILE__);
		printk(" expected: 0x%x 0x%x 0x%x 0x%x\n",
			NIUSB_TERM_ID, 0x0, 0x0, 0x0);
		printk(" received: 0x%x 0x%x 0x%x 0x%x\n",
			buffer[i - 4], buffer[i - 3], buffer[i - 2], buffer[i - 1]);
	}
	return i;
};

int parse_board_ibrd_readback(const uint8_t *raw_data, struct ni_usb_status_block *status,
	uint8_t *parsed_data, int parsed_data_length, int *actual_bytes_read)
{
	static const int ibrd_data_block_length = 0xf;
	static const int ibrd_extended_data_block_length = 0x1e;
	int data_block_length = 0;
	int i = 0;
	int j = 0;
	int k;
	unsigned int adr1_bits;
	int num_data_blocks = 0;
	struct ni_usb_status_block register_write_status;
	int unexpected = 0;

	while(raw_data[i] == NIUSB_IBRD_DATA_ID || raw_data[i] == NIUSB_IBRD_EXTENDED_DATA_ID)
	{
		if(raw_data[i] == NIUSB_IBRD_DATA_ID)
		{
			data_block_length = ibrd_data_block_length;
		}else if(raw_data[i] == NIUSB_IBRD_EXTENDED_DATA_ID)
		{
			data_block_length = ibrd_extended_data_block_length;
			if(raw_data[++i] != 0)
			{
				printk("%s: %s: unexpected data: raw_data[%i]=0x%x, expected 0\n",
					__FILE__, __FUNCTION__, i, (int)raw_data[i]);
				unexpected = 1;
			}
		}else
		{
			printk("%s: logic bug!\n", __FILE__);
			return -EINVAL;
		}
		++i;
		for(k = 0; k < data_block_length; k++)
		{
			if(j < parsed_data_length)
				parsed_data[j++] = raw_data[i++];
			else
				++i;
		}
		++num_data_blocks;
	}
	i += ni_usb_parse_status_block(&raw_data[i], status);
	if(status->id != NIUSB_IBRD_STATUS_ID)
	{
		printk("%s: bug: status->id=%i, != ibrd_status_id\n", __FILE__, status->id);
		return -EIO;
	}
	adr1_bits = raw_data[i++];
	if(num_data_blocks)
		*actual_bytes_read = (num_data_blocks - 1) * data_block_length + raw_data[i++];
	else
	{
		++i;
		*actual_bytes_read = 0;
	}
	if(*actual_bytes_read > j)
	{
		printk("%s: bug: discarded data. actual_bytes_read=%i, j=%i (firmware garbage count? clamping)\n",
			__FILE__, *actual_bytes_read, j);
		/* New firmware fills the bytes-in-last-block byte with 0xaa filler
		 * when the read ends without END/EOI; trusting it overruns the
		 * caller's buffer (user-space segfault). Clamp to what was
		 * actually parsed. */
		*actual_bytes_read = j;
	}
	for(k = 0; k < 2; k++)
		if(raw_data[i++] != 0)
		{
			printk("%s: %s: unexpected data: raw_data[%i]=0x%x, expected 0\n",
				__FILE__, __FUNCTION__, i - 1, (int)raw_data[i - 1]);
			unexpected = 1;
		}
	/* Quirk 2: the adapter omits the register-write status
	 * block in the ibrd response; the termination block follows the pad
	 * bytes directly.  Accept that layout instead of overrunning into
	 * garbage (which made ni_usb_read discard a successful read). */
	if(raw_data[i] == NIUSB_TERM_ID)
	{
		static int quirk_reported = 0;
		if(quirk_reported == 0)
		{
			printk("%s: %s: no register-write status block in ibrd response (new firmware quirk), accepting\n",
				__FILE__, __FUNCTION__);
			quirk_reported = 1;
		}
		i += ni_usb_parse_termination_block(&raw_data[i]);
		if(unexpected)
			ni_usb_dump_raw_block(raw_data, i);
		return i;
	}
	i += ni_usb_parse_status_block(&raw_data[i], &register_write_status);
	if(register_write_status.id != NIUSB_REG_WRITE_ID)
	{
		printk("%s: %s: unexpected data: register write status id=0x%x, expected 0x%x\n",
			__FILE__, __FUNCTION__, register_write_status.id, NIUSB_REG_WRITE_ID);
		unexpected = 1;
	}
	if(raw_data[i++] != 2)
	{
		printk("%s: %s: unexpected data: register write count=%i, expected 2\n",
			__FILE__, __FUNCTION__, (int)raw_data[i - 1]);
		unexpected = 1;
	}
	for(k = 0; k < 3; k++)
		if(raw_data[i++] != 0)
		{
			printk("%s: %s: unexpected data: raw_data[%i]=0x%x, expected 0\n",
				__FILE__, __FUNCTION__, i - 1, (int)raw_data[i - 1]);
			unexpected = 1;
		}
	i += ni_usb_parse_termination_block(&raw_data[i]);
	if(unexpected)
		ni_usb_dump_raw_block(raw_data, i);
	return i;
}

int ni_usb_parse_reg_write_status_block(const uint8_t *raw_data, struct ni_usb_status_block *status, int *writes_completed)
{
	int i = 0;

	i += ni_usb_parse_status_block(raw_data, status);
	*writes_completed = raw_data[i++];
	while(i % 4) i++;
	return i;
}
int ni_usb_write_registers(ni_usb_private_t *ni_priv, const struct ni_usb_register *writes, int num_writes,
	unsigned int *ibsta)
{
	int retval;
	uint8_t *out_data, *in_data;
	int out_data_length, in_data_length;
	int bytes_written = 0, bytes_read = 0;
	int i = 0;
	int j;
	struct ni_usb_status_block status;
	static const int bytes_per_write = 3;
	int reg_writes_completed;

	out_data_length = num_writes * bytes_per_write + 0x10;
	out_data = kmalloc(out_data_length, GFP_KERNEL);
	if(out_data == NULL)
	{
		printk("%s: %s: kmalloc failed\n", __FILE__, __FUNCTION__);
		return -ENOMEM;
	}
	i += ni_usb_bulk_register_write_header(&out_data[i], num_writes);
	for(j = 0; j < num_writes; j++)
	{
		i += ni_usb_bulk_register_write(&out_data[i], writes[j]);
	}
	while(i % 4)
		out_data[i++] = 0x00;
	i += ni_usb_bulk_termination(&out_data[i]);
	if(i > out_data_length)
	{
		printk("%s: bug! buffer overrun\n", __FUNCTION__);
	}
	retval = ni_usb_send_bulk_msg(ni_priv, out_data, i, &bytes_written, 1000);
	kfree(out_data);
	if(retval)
	{
		printk("%s: %s: ni_usb_send_bulk_msg returned %i, bytes_written=%i, i=%i\n", __FILE__, __FUNCTION__,
			retval, bytes_written, i);
		return retval;
	}
	in_data_length = 0x20;
	in_data = kmalloc(in_data_length, GFP_KERNEL);
	if(in_data == NULL)
	{
		printk("%s: kmalloc failed\n", __FILE__);
		return -ENOMEM;
	}
	retval = ni_usb_receive_bulk_msg(ni_priv, in_data, in_data_length, &bytes_read, 1000, 0);
	if(retval || (bytes_read != 12 && bytes_read != 16))
	{
		printk("%s: %s: ni_usb_receive_bulk_msg returned %i, bytes_read=%i\n", __FILE__, __FUNCTION__, retval, bytes_read);
		ni_usb_dump_raw_block(in_data, bytes_read);
		kfree(in_data);
		return retval;
	}
	ni_usb_parse_reg_write_status_block(in_data, &status, &reg_writes_completed);
	//FIXME parse extra 09 status bits and termination
	kfree(in_data);
	if(status.id != NIUSB_REG_WRITE_ID)
	{
		printk("%s: %s: parse error, id=0x%x != NIUSB_REG_WRITE_ID\n", __FILE__, __FUNCTION__, status.id);
		ni_priv->pipe_dirty = 1;
		return -EIO;
	}
	if(status.error_code)
	{
		printk("%s: %s: nonzero error code 0x%x\n", __FILE__, __FUNCTION__, status.error_code);
		return -EIO;
	}
	if(reg_writes_completed != num_writes)
	{
		printk("%s: %s: reg_writes_completed=%i, num_writes=%i\n", __FILE__, __FUNCTION__,
			reg_writes_completed, num_writes);
		return -EIO;
	}
	if(ibsta) *ibsta = status.ibsta;
	return 0;
}

// interface functions
int ni_usb_read(gpib_board_t *board, uint8_t *buffer, size_t length, int *end, size_t *bytes_read)
{
	int retval, parse_retval;
	ni_usb_private_t *ni_priv = board->private_data;
	uint8_t *out_data, *in_data;
	int out_data_length, in_data_length;
	int usb_bytes_written = 0, usb_bytes_read = 0;
	int i = 0;
	int complement_count;
	int actual_length;
	struct ni_usb_status_block status;
	static const int max_read_length = 0xffff;
	struct ni_usb_register reg;

	*bytes_read = 0;
	if(length > max_read_length)
	{
		length = max_read_length;
		printk("%s: read length too long\n", __FILE__);
	}
	out_data_length = 0x20;
	out_data = kmalloc(out_data_length, GFP_KERNEL);
	if(out_data == NULL) return -ENOMEM;
	out_data[i++] = 0x0a;
	out_data[i++] = ni_priv->eos_mode >> 8;
	out_data[i++] = ni_priv->eos_char;
	out_data[i++] = ni_usb_timeout_code(board->usec_timeout);
	complement_count = length - 1;
	complement_count = ~complement_count;
	out_data[i++] = complement_count & 0xff;
	out_data[i++] = (complement_count >> 8) & 0xff;
	out_data[i++] = 0x0;
	out_data[i++] = 0x0;
	i += ni_usb_bulk_register_write_header(&out_data[i], 2);
	reg.device = NIUSB_SUBDEV_TNT4882;
	reg.address = nec7210_to_tnt4882_offset(AUXMR);
	reg.value = AUX_HLDI;
	i += ni_usb_bulk_register_write(&out_data[i], reg);
	reg.value = AUX_CLEAR_END;
	i += ni_usb_bulk_register_write(&out_data[i], reg);
	while(i % 4)	// pad with zeros to 4-byte boundary
		out_data[i++] = 0x0;
	i += ni_usb_bulk_termination(&out_data[i]);
	mutex_lock(&ni_priv->addressed_transfer_lock);
	retval = ni_usb_send_bulk_msg(ni_priv, out_data, i, &usb_bytes_written, 1000);
	kfree(out_data);
	if(retval || usb_bytes_written != i)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		if(retval == 0) retval = -EIO;
		printk("%s: %s: ni_usb_send_bulk_msg returned %i, usb_bytes_written=%i, i=%i\n", __FILE__, __FUNCTION__, retval, usb_bytes_written, i);
		return retval;
	}
	in_data_length = (length / 30 + 1) * 0x20 + 0x20;
	in_data = kmalloc(in_data_length, GFP_KERNEL);
	if(in_data == NULL)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		return -ENOMEM;
	}
	retval = ni_usb_receive_bulk_msg(ni_priv, in_data, in_data_length, &usb_bytes_read,
		ni_usb_timeout_msecs(board->usec_timeout), 1);
	mutex_unlock(&ni_priv->addressed_transfer_lock);
	if(retval == -ERESTARTSYS)
	{}
	else if(retval)
	{
		printk("%s: %s: ni_usb_receive_bulk_msg returned %i, usb_bytes_read=%i\n", 
			__FILE__, __FUNCTION__, retval, usb_bytes_read);
		kfree(in_data);
		return retval;
	}
	parse_retval = parse_board_ibrd_readback(in_data, &status, buffer, length, &actual_length);
	if(parse_retval != usb_bytes_read)
	{
		if(parse_retval >= 0) parse_retval = -EIO;
		printk("%s: %s: retval=%i usb_bytes_read=%i\n", __FILE__, __FUNCTION__, parse_retval, usb_bytes_read);
		ni_priv->pipe_dirty = 1;
		kfree(in_data);
		return parse_retval;
	}
	if(actual_length != length - status.count)
	{
		printk("%s: %s: actual_length=%i expected=%li\n", __FILE__, __FUNCTION__, actual_length, (long)(length - status.count));
		ni_usb_dump_raw_block(in_data, usb_bytes_read);
	}
	kfree(in_data);	/* moved below the dump (was a use-after-free) */
	switch(status.error_code)
	{
	case NIUSB_NO_ERROR:
		retval = 0;
		break;
	case NIUSB_ABORTED_ERROR:
		/*this is expected if ni_usb_receive_bulk_msg got interrupted by a signal 
			and returned -ERESTARTSYS */
		break;
	case NIUSB_ADDRESSING_ERROR:
		retval = -EIO;
		break;
	case NIUSB_TIMEOUT_ERROR:
		retval = -ETIMEDOUT;
		break;
	case NIUSB_EOSMODE_ERROR:
		printk("%s: %s: driver bug, we should have been able to avoid NIUSB_EOSMODE_ERROR.\n", __FILE__, __FUNCTION__);
		retval = -EINVAL;
		break;
	default:
		printk("%s: %s: unknown error code=%i\n", __FILE__, __FUNCTION__, status.error_code);
		retval = -EIO;
		break;
	}
	ni_usb_soft_update_status(board, status.ibsta, 0);
	if(status.ibsta & END) *end = 1;
	else *end = 0;
	*bytes_read = actual_length;
	printk("NIDBG rd: len=%i got=%i err=%i ibsta=0x%x data=%02x %02x %02x %02x\n",
		(int)length, actual_length, status.error_code, status.ibsta,
		actual_length > 0 ? buffer[0] : 0xee, actual_length > 1 ? buffer[1] : 0xee,
		actual_length > 2 ? buffer[2] : 0xee, actual_length > 3 ? buffer[3] : 0xee);
	return retval;
}

static int ni_usb_write(gpib_board_t *board, uint8_t *buffer, size_t length, int send_eoi, size_t *bytes_written)
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	uint8_t *out_data, *in_data;
	int out_data_length, in_data_length;
	int usb_bytes_written = 0, usb_bytes_read = 0;
	int i = 0, j;
	int complement_count;
	int recv_timeout;
	struct ni_usb_status_block status;
	static const int max_write_length = 0xffff;

	*bytes_written = 0;
	printk("NIDBG wrt: entry len=%i eoi=%i\n", (int)length, send_eoi);
	/* a firmware gts (sent after parallel polls) wiped the engine's
	 * addressing state: replay the last addressing sequence so this write
	 * finds the engine in TACS again (idempotent on the bus side). */
	if(ni_priv->needs_readdress)
	{
		ni_priv->needs_readdress = 0;
		if(ni_priv->last_addressing_len > 0)
		{
			size_t dummy;

			printk("NIDBG wrt: replaying %i addressing byte(s) after gts\n",
				ni_priv->last_addressing_len);
			ni_usb_command_chunk(board, ni_priv->last_addressing,
				ni_priv->last_addressing_len, &dummy);
		}
	}
	if(length > max_write_length)
	{
		length = max_write_length;
		send_eoi = 0;
		printk("%s: write length too long\n", __FILE__);
	}
	out_data_length = length + 0x10;
	out_data = kmalloc(out_data_length, GFP_KERNEL);
	if(out_data == NULL) return -ENOMEM;
	out_data[i++] = 0x0d;
	complement_count = length;
	complement_count = length - 1;
	complement_count = ~complement_count;
	out_data[i++] = complement_count & 0xff;
	out_data[i++] = (complement_count >> 8) & 0xff;
	out_data[i++] = ni_usb_timeout_code(board->usec_timeout);
	out_data[i++] = 0x0;
	out_data[i++] = 0x0;
	if(send_eoi)
		out_data[i++] = 0x8;
	else
		out_data[i++] = 0x0;
	out_data[i++] = 0x0;
	for(j = 0; j < length; j++)
		out_data[i++] = buffer[j];
	while(i % 4)	// pad with zeros to 4-byte boundary
		out_data[i++] = 0x0;
	i += ni_usb_bulk_termination(&out_data[i]);
	mutex_lock(&ni_priv->addressed_transfer_lock);
	retval = ni_usb_send_bulk_msg(ni_priv, out_data, i, &usb_bytes_written,
		ni_usb_timeout_msecs(board->usec_timeout));
	kfree(out_data);
	if(retval || usb_bytes_written != i)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		printk("%s: %s: ni_usb_send_bulk_msg returned %i, usb_bytes_written=%i, i=%i\n", __FILE__, __FUNCTION__, retval, usb_bytes_written, i);
		return retval;
	}
	ni_priv->wrote_since_rpp = 1;
	/* quirk 6: a data write WITHOUT EOI termination is
	 * executed on the bus (the bytes demonstrably reach the listener) but
	 * its status block is never sent back.  Once that behaviour has been
	 * confirmed on this unit, don't wait for an ack that never comes. */
	if(send_eoi == 0 && ni_priv->eoiless_ack_missing)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		*bytes_written = length;
		return 0;
	}
	in_data_length = 0x10;
	in_data = kmalloc(in_data_length, GFP_KERNEL);
	if(in_data == NULL)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		return -ENOMEM;
	}
	recv_timeout = ni_usb_timeout_msecs(board->usec_timeout);
	if(send_eoi == 0 && (recv_timeout == 0 || recv_timeout > 500))
		recv_timeout = 500;	/* quirk 6 probe: short wait on first EOI-less write */
	retval = ni_usb_receive_bulk_msg(ni_priv, in_data, in_data_length, &usb_bytes_read,
		recv_timeout, 1);
	mutex_unlock(&ni_priv->addressed_transfer_lock);
	if(send_eoi == 0 && retval && retval != -ERESTARTSYS && usb_bytes_read == 0)
	{
		/* quirk 6 confirmed: emulate the missing acknowledgement; the
		 * pipe_dirty resync absorbs any straggler response. */
		printk("NIDBG wrt: quirk6 - no ack for EOI-less write, emulating success from now on\n");
		ni_priv->eoiless_ack_missing = 1;
		ni_priv->pipe_dirty = 1;
		kfree(in_data);
		*bytes_written = length;
		return 0;
	}
	if((retval && retval != -ERESTARTSYS) || usb_bytes_read != 12)
	{
		printk("%s: %s: ni_usb_receive_bulk_msg returned %i, usb_bytes_read=%i\n", __FILE__, __FUNCTION__, retval, usb_bytes_read);
		kfree(in_data);
		return retval;
	}
	ni_usb_parse_status_block(in_data, &status);
	kfree(in_data);
	switch(status.error_code)
	{
	case NIUSB_NO_ERROR:
		retval = 0;
		break;
	case NIUSB_ABORTED_ERROR:
		/*this is expected if ni_usb_receive_bulk_msg got interrupted by a signal 
			and returned -ERESTARTSYS */
		break;
	case NIUSB_ADDRESSING_ERROR:
		retval = -EIO;
		break;
	case NIUSB_NO_LISTENER_ERROR:
		retval = -EIO;
		break;
	case NIUSB_TIMEOUT_ERROR:
		retval = -ETIMEDOUT;
		break;
	default:
		printk("%s: %s: unknown error code=%i\n", __FILE__, __FUNCTION__, status.error_code);
		retval = -EIO;
		break;
	}
	/* diagnostic: make the firmware's verdict visible (error codes 3 and 8
	 * are otherwise indistinguishable from user space) */
	if(status.error_code != NIUSB_NO_ERROR)
	{
		printk("%s: %s: status block on error: id=0x%x ibsta=0x%x error_code=%i count=%i length=%i\n",
			__FILE__, __FUNCTION__, status.id, status.ibsta, status.error_code,
			status.count, (int)length);
	}
	if(status.id != 0xd)	/* response shifted: not our write's status */
		ni_priv->pipe_dirty = 1;
	ni_usb_soft_update_status(board, status.ibsta, 0);
	if((size_t)status.count > length)
	{
		printk("%s: %s: BOGUS status block: id=0x%x ibsta=0x%x error=%i count=%i length=%i (pipe desync?)\n",
			__FILE__, __FUNCTION__, status.id, status.ibsta, status.error_code, status.count, (int)length);
		ni_priv->pipe_dirty = 1;
		*bytes_written = 0;
		return -EIO;
	}
	*bytes_written = length - status.count;
	return retval;
}

int ni_usb_command_chunk(gpib_board_t *board, uint8_t *buffer, size_t length, size_t *command_bytes_written)
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	uint8_t *out_data, *in_data;
	int out_data_length, in_data_length;
	int bytes_written = 0, bytes_read = 0;
	int i = 0, j;
	unsigned complement_count;
	struct ni_usb_status_block status;
	// usb-b gives error 4 if you try to send more than 16 command bytes at once
	static const int max_command_length = 0x10;

	*command_bytes_written = 0;
	if(length > max_command_length) length = max_command_length;
	out_data_length = length + 0x10;
	out_data = kmalloc(out_data_length, GFP_KERNEL);
	if(out_data == NULL) return -ENOMEM;
	out_data[i++] = 0x0c;
	complement_count = length - 1;
	complement_count = ~complement_count;
	out_data[i++] = complement_count;
	out_data[i++] = 0x0;
	out_data[i++] = ni_usb_timeout_code(board->usec_timeout);
	for(j = 0; j < length; j++)
		out_data[i++] = buffer[j];
	while(i % 4)	// pad with zeros to 4-byte boundary
		out_data[i++] = 0x0;
	i += ni_usb_bulk_termination(&out_data[i]);
	mutex_lock(&ni_priv->addressed_transfer_lock);
	retval = ni_usb_send_bulk_msg(ni_priv, out_data, i, &bytes_written,
		ni_usb_timeout_msecs(board->usec_timeout));
	kfree(out_data);
	if(retval || bytes_written != i)
	{
		int k;
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		printk("%s: %s: ni_usb_send_bulk_msg returned %i, bytes_written=%i, i=%i\n", __FILE__, __FUNCTION__, retval, bytes_written, i);
		printk("\t was attempting to write command bytes:\n");
		for(k = 0; k < length; ++k)
		{
			printk(" 0x%2x", buffer[k]);
		}
		printk("\n");
		return retval;
	}
	in_data_length = 0x10;
	in_data = kmalloc(in_data_length, GFP_KERNEL);
	if(in_data == NULL)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		return -ENOMEM;
	}
	retval = ni_usb_receive_bulk_msg(ni_priv, in_data, in_data_length, &bytes_read,
		ni_usb_timeout_msecs(board->usec_timeout), 1);
	mutex_unlock(&ni_priv->addressed_transfer_lock);
	if((retval && retval != -ERESTARTSYS) || bytes_read != 12)
	{
		if(retval == 0) retval = -EIO;
		printk("%s: %s: ni_usb_receive_bulk_msg returned %i, bytes_read=%i\n", __FILE__, __FUNCTION__, retval, bytes_read);
		ni_usb_dump_raw_block(in_data, bytes_read);
		kfree(in_data);
		return retval;
	}
	ni_usb_parse_status_block(in_data, &status);
	kfree(in_data);
	if((size_t)status.count > length)
	{
		printk("%s: %s: BOGUS status block: id=0x%x ibsta=0x%x error=%i count=%i length=%i (pipe desync?)\n",
			__FILE__, __FUNCTION__, status.id, status.ibsta, status.error_code, status.count, (int)length);
		ni_priv->pipe_dirty = 1;
		*command_bytes_written = 0;
		return -EIO;
	}
	if(status.id != 0xc)	/* response shifted: not our command's status */
		ni_priv->pipe_dirty = 1;
	*command_bytes_written = length - status.count;
	printk("NIDBG cmd: len=%i wrote=%i err=%i ibsta=0x%x first=0x%02x 0x%02x 0x%02x 0x%02x\n",
		(int)length, (int)*command_bytes_written, status.error_code, status.ibsta,
		buffer[0], length > 1 ? buffer[1] : 0, length > 2 ? buffer[2] : 0,
		length > 3 ? buffer[3] : 0);
	switch(status.error_code)
	{
	case NIUSB_NO_ERROR:
		break;
	case NIUSB_ABORTED_ERROR:
		/*this is expected if ni_usb_receive_bulk_msg got interrupted by a signal 
			and returned -ERESTARTSYS */
		break;
	case NIUSB_NO_BUS_ERROR:
		return -EIO;
		break;
	case NIUSB_EOSMODE_ERROR:
		printk("%s: %s: got eosmode error.  Driver bug?\n", __FILE__, __FUNCTION__);
		return -EIO;
		break;
	default:
		printk("%s: %s: unknown error code=%i\n", __FILE__, __FUNCTION__, status.error_code);
		return -EIO;
		break;
	}
	ni_usb_soft_update_status(board, status.ibsta, 0);
	return 0;
}

int ni_usb_command(gpib_board_t *board, uint8_t *buffer, size_t length, size_t *bytes_written)
{
	size_t count;
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;

	*bytes_written = 0;
	while(*bytes_written < length)
	{
		retval = ni_usb_command_chunk(board, buffer + *bytes_written, length - *bytes_written, &count);
		*bytes_written += count;
		if(retval < 0) return retval;
	}
	/* remember the last addressing sequence (it contains our MTA): it is
	 * replayed before the next write if a firmware gts wiped the engine's
	 * addressing state (see ni_usb_parallel_poll / ni_usb_write). */
	{
		uint8_t mta = 0x40 | (board->pad & 0x1f);
		size_t k;

		for(k = 0; k < length; k++)
		{
			if((buffer[k] & 0x7f) == mta)
			{
				size_t copy = length;
				if(copy > sizeof(ni_priv->last_addressing))
					copy = sizeof(ni_priv->last_addressing);
				memcpy(ni_priv->last_addressing, buffer, copy);
				ni_priv->last_addressing_len = copy;
				ni_priv->needs_readdress = 0;
				break;
			}
		}
	}
	return 0;
}

int ni_usb_take_control(gpib_board_t *board, int synchronous)
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	uint8_t *out_data, *in_data;
	int out_data_length, in_data_length;
	int bytes_written = 0, bytes_read = 0;
	int i = 0;
	struct ni_usb_status_block status;

	out_data_length = 0x10;
	out_data = kmalloc(out_data_length, GFP_KERNEL);
	if(out_data == NULL)
	{
		printk("%s: kmalloc failed\n", __FILE__);
		return -ENOMEM;
	}
	out_data[i++] = NIUSB_IBCAC_ID;
	if(synchronous)
		out_data[i++] = 0x1;
	else
		out_data[i++] = 0x0;
	out_data[i++] = 0x0;
	out_data[i++] = 0x0;
	i += ni_usb_bulk_termination(&out_data[i]);
	mutex_lock(&ni_priv->addressed_transfer_lock);
	retval = ni_usb_send_bulk_msg(ni_priv, out_data, i, &bytes_written, 1000);
	kfree(out_data);
	if(retval || bytes_written != i)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		printk("%s: %s: ni_usb_send_bulk_msg returned %i, bytes_written=%i, i=%i\n", __FILE__, __FUNCTION__, retval, bytes_written, i);
		return retval;
	}
	in_data_length = 0x10;
	in_data = kmalloc(in_data_length, GFP_KERNEL);
	if(in_data == NULL)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		printk("%s: kmalloc failed\n", __FILE__);
		return -ENOMEM;
	}
	retval = ni_usb_receive_bulk_msg(ni_priv, in_data, in_data_length, &bytes_read, 1000, 1);
	mutex_unlock(&ni_priv->addressed_transfer_lock);
	if((retval && retval != -ERESTARTSYS) || bytes_read != 12)
	{
		if(retval == 0) retval = -EIO;
		printk("%s: %s: ni_usb_receive_bulk_msg returned %i, bytes_read=%i\n", __FILE__, __FUNCTION__, retval, bytes_read);
		kfree(in_data);
		return retval;
	}
	ni_usb_parse_status_block(in_data, &status);
	kfree(in_data);
	printk("NIDBG cac: err=%i ibsta=0x%x\n", status.error_code, status.ibsta);
	ni_usb_soft_update_status(board, status.ibsta, 0);
	return retval;
}
/* the real firmware IBGTS request — used only after parallel polls to
 * release the IDY state (it wipes the firmware's addressing state, which is
 * then replayed before the next write; see ni_usb_parallel_poll). */
static int ni_usb_firmware_gts(gpib_board_t *board)
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	uint8_t *out_data, *in_data;
	int out_data_length, in_data_length;
	int bytes_written = 0, bytes_read = 0;
	int i = 0;
	struct ni_usb_status_block status;

	out_data_length = 0x10;
	out_data = kmalloc(out_data_length, GFP_KERNEL);
	if(out_data == NULL)
	{
		printk("%s: kmalloc failed\n", __FILE__);
		return -ENOMEM;
	}
	out_data[i++] = NIUSB_IBGTS_ID;
	out_data[i++] = 0x0;
	out_data[i++] = 0x0;
	out_data[i++] = 0x0;
	i += ni_usb_bulk_termination(&out_data[i]);
	mutex_lock(&ni_priv->addressed_transfer_lock);
	retval = ni_usb_send_bulk_msg(ni_priv, out_data, i, &bytes_written, 1000);
	kfree(out_data);
	if(retval || bytes_written != i)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		printk("%s: %s: ni_usb_send_bulk_msg returned %i, bytes_written=%i, i=%i\n", __FILE__, __FUNCTION__, retval, bytes_written, i);
		return retval;
	}
	in_data_length = 0x20;
	in_data = kmalloc(in_data_length, GFP_KERNEL);
	if(in_data == NULL)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		printk("%s: kmalloc failed\n", __FILE__);
		return -ENOMEM;
	}
	retval = ni_usb_receive_bulk_msg(ni_priv, in_data, in_data_length, &bytes_read, 1000, 0);
	mutex_unlock(&ni_priv->addressed_transfer_lock);
	if(retval || bytes_read != 12)
	{
		printk("%s: %s: ni_usb_receive_bulk_msg returned %i, bytes_read=%i\n", __FILE__, __FUNCTION__, retval, bytes_read);
		kfree(in_data);
		return retval;
	}
	ni_usb_parse_status_block(in_data, &status);
	kfree(in_data);
	if(status.id != NIUSB_IBGTS_ID)
	{
		printk("%s: %s: status.id 0x%x != IBGTS (pipe shift?)\n", __FILE__, __FUNCTION__, status.id);
		ni_priv->pipe_dirty = 1;
	}
	ni_usb_soft_update_status(board, status.ibsta, 0);
	return 0;
}

int ni_usb_go_to_standby(gpib_board_t *board)
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	uint8_t *out_data, *in_data;
	int out_data_length, in_data_length;
	int bytes_written = 0, bytes_read = 0;
	int i = 0;
	struct ni_usb_status_block status;

	/* Quirk 3: the IBGTS request clears the
	 * firmware's addressing state (TACS/LACS observed dropping from the
	 * status block), so any board write right after addressing fails with
	 * addressing error 3.  A chip-level AUX_GTS here breaks writes too
	 * (verified).  So this stays a no-op for the library path; the
	 * firmware's own read/write requests manage ATN themselves.  The one
	 * place that really needs an ATN/EOI release is the tail of
	 * ni_usb_parallel_poll (see there). */
	{
		static int gts_skip_reported = 0;
		if(gts_skip_reported == 0)
		{
			printk("ni_usb_gpib: go_to_standby is a no-op (new firmware clears addressing state on gts)\n");
			gts_skip_reported = 1;
		}
	}
	return 0;

	out_data_length = 0x10;
	out_data = kmalloc(out_data_length, GFP_KERNEL);
	if(out_data == NULL)
	{
		printk("%s: kmalloc failed\n", __FILE__);
		return -ENOMEM;
	}
	out_data[i++] = NIUSB_IBGTS_ID;
	out_data[i++] = 0x0;
	out_data[i++] = 0x0;
	out_data[i++] = 0x0;
	i += ni_usb_bulk_termination(&out_data[i]);
	mutex_lock(&ni_priv->addressed_transfer_lock);
	retval = ni_usb_send_bulk_msg(ni_priv, out_data, i, &bytes_written, 1000);
	kfree(out_data);
	if(retval || bytes_written != i)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		printk("%s: %s: ni_usb_send_bulk_msg returned %i, bytes_written=%i, i=%i\n", __FILE__, __FUNCTION__, retval, bytes_written, i);
		return retval;
	}
	in_data_length = 0x20;
	in_data = kmalloc(in_data_length, GFP_KERNEL);
	if(in_data == NULL)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		printk("%s: kmalloc failed\n", __FILE__);
		return -ENOMEM;
	}
	retval = ni_usb_receive_bulk_msg(ni_priv, in_data, in_data_length, &bytes_read, 1000, 0);
	mutex_unlock(&ni_priv->addressed_transfer_lock);
	if(retval || bytes_read != 12)
	{
		printk("%s: %s: ni_usb_receive_bulk_msg returned %i, bytes_read=%i\n", __FILE__, __FUNCTION__, retval, bytes_read);
		kfree(in_data);
		return retval;
	}
	ni_usb_parse_status_block(in_data, &status);
	kfree(in_data);
	if(status.id != NIUSB_IBGTS_ID)
	{
		printk("%s: %s: bug: status.id 0x%x != INUSB_IBGTS_ID\n", __FILE__, __FUNCTION__, status.id);
	}
	printk("NIDBG gts: err=%i ibsta=0x%x\n", status.error_code, status.ibsta);
	ni_usb_soft_update_status(board, status.ibsta, 0);
	return 0;
}
//FIXME should change prototype to return int
void ni_usb_request_system_control( gpib_board_t *board, int request_control )
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	int i = 0;
	struct ni_usb_register writes[4];
	unsigned int ibsta;

	if(request_control)
	{
		writes[i].device = NIUSB_SUBDEV_TNT4882;
		writes[i].address = CMDR;
		writes[i].value = SETSC;
		i++;
		writes[i].device = NIUSB_SUBDEV_TNT4882;
		writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
		writes[i].value = AUX_CIFC;
		i++;
	}else
	{
		writes[i].device = NIUSB_SUBDEV_TNT4882;
		writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
		writes[i].value = AUX_CREN;
		i++;
		writes[i].device = NIUSB_SUBDEV_TNT4882;
		writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
		writes[i].value = AUX_CIFC;
		i++;
		writes[i].device = NIUSB_SUBDEV_TNT4882;
		writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
		writes[i].value = AUX_DSC;
		i++;
		writes[i].device = NIUSB_SUBDEV_TNT4882;
		writes[i].address = CMDR;
		writes[i].value = CLRSC;
		i++;
	}
	retval = ni_usb_write_registers(ni_priv, writes, i, &ibsta);
	if(retval < 0)
	{
		printk("%s: %s: register write failed, retval=%i\n", __FILE__, __FUNCTION__, retval);
		return;// retval;
	}
	ni_usb_soft_update_status(board, ibsta, 0);
	return;// 0;
}
//FIXME maybe the interface should have a "pulse interface clear" function that can return an error?
void ni_usb_interface_clear(gpib_board_t *board, int assert)
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	uint8_t *out_data, *in_data;
	int out_data_length, in_data_length;
	int bytes_written = 0, bytes_read = 0;
	int i = 0;
	struct ni_usb_status_block status;

	// FIXME: we are going to pulse when assert is true, and ignore otherwise
	if(assert == 0) return;
	out_data_length = 0x10;
	out_data = kmalloc(out_data_length, GFP_KERNEL);
	if(out_data == NULL)
	{
		printk("%s: kmalloc failed\n", __FILE__);
		return;
	}
	out_data[i++] = NIUSB_IBSIC_ID;
	out_data[i++] = 0x0;
	out_data[i++] = 0x0;
	out_data[i++] = 0x0;
	i += ni_usb_bulk_termination(&out_data[i]);
	retval = ni_usb_send_bulk_msg(ni_priv, out_data, i, &bytes_written, 1000);
	kfree(out_data);
	if(retval || bytes_written != i)
	{
		printk("%s: %s: ni_usb_send_bulk_msg returned %i, bytes_written=%i, i=%i\n", __FILE__, __FUNCTION__, retval, bytes_written, i);
		return;
	}
	in_data_length = 0x10;
	in_data = kmalloc(in_data_length, GFP_KERNEL);
	if(in_data == NULL)
	{
		printk("%s: kmalloc failed\n", __FILE__);
		return;
	}
	retval = ni_usb_receive_bulk_msg(ni_priv, in_data, in_data_length, &bytes_read, 1000, 0);
	if(retval || bytes_read != 12)
	{
		printk("%s: %s: ni_usb_receive_bulk_msg returned %i, bytes_read=%i\n", __FILE__, __FUNCTION__, retval, bytes_read);
		kfree(in_data);
		return;
	}
	ni_usb_parse_status_block(in_data, &status);
	kfree(in_data);
	ni_usb_soft_update_status(board, status.ibsta, 0);
	return;
}
void ni_usb_remote_enable(gpib_board_t *board, int enable)
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	struct ni_usb_register reg;
	unsigned int ibsta;

	reg.device = NIUSB_SUBDEV_TNT4882;
	reg.address = nec7210_to_tnt4882_offset(AUXMR);
	if(enable)
	{
		reg.value = AUX_SREN;
	}else
	{
		reg.value = AUX_CREN;
	}
	retval = ni_usb_write_registers(ni_priv, &reg, 1, &ibsta);
	if(retval < 0)
	{
		printk("%s: %s: register write failed, retval=%i\n", __FILE__, __FUNCTION__, retval);
		return; //retval;
	}
	ni_usb_soft_update_status(board, ibsta, 0);
	return;// 0;
}

int ni_usb_enable_eos(gpib_board_t *board, uint8_t eos_byte, int compare_8_bits)
{
	ni_usb_private_t *ni_priv = board->private_data;

	ni_priv->eos_char = eos_byte;
	ni_priv->eos_mode |= REOS;
	if(compare_8_bits)
		ni_priv->eos_mode |= BIN;
	else
		ni_priv->eos_mode &= ~BIN;
	return 0;
}

void ni_usb_disable_eos(gpib_board_t *board)
{
	ni_usb_private_t *ni_priv = board->private_data;
	/* adapter gets unhappy if you don't zero all the bits
		for the eos mode and eos char (returns error 4 on reads). */
	ni_priv->eos_mode = 0;
	ni_priv->eos_char = 0;
}

unsigned int ni_usb_update_status( gpib_board_t *board, unsigned int clear_mask )
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	static const int bufferLength = 8;
	uint8_t *buffer;
	struct ni_usb_status_block status;

	//printk("%s: receive control pipe is %i\n", __FILE__, pipe);
	buffer = kmalloc(bufferLength, GFP_KERNEL);
	if(buffer == NULL)
	{
		printk("%s: kmalloc failed!\n", __FILE__);
		return board->status;
	}
	retval = ni_usb_receive_control_msg(ni_priv, NI_USB_WAIT_REQUEST, USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		0x200, 0x0, buffer, bufferLength, 1000);
	if(retval != bufferLength)
	{
		printk("%s: usb_control_msg returned %i\n", __FILE__, retval);
		kfree(buffer);
		return board->status;
	}
	ni_usb_parse_status_block(buffer, &status);
	kfree(buffer);
	ni_usb_soft_update_status(board, status.ibsta, clear_mask);
	return board->status;
}

// tells ni-usb to immediately stop an ongoing i/o operation
static void ni_usb_stop(ni_usb_private_t *ni_priv)
{
	int retval;
	static const int bufferLength = 8;
	uint8_t *buffer;
	struct ni_usb_status_block status;

	//printk("%s: receive control pipe is %i\n", __FILE__, pipe);
	buffer = kmalloc(bufferLength, GFP_KERNEL);
	if(buffer == NULL)
	{
		printk("%s: kmalloc failed!\n", __FILE__);
		return;
	}
	retval = ni_usb_receive_control_msg(ni_priv, NI_USB_STOP_REQUEST, USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		0x0, 0x0, buffer, bufferLength, 1000);
	if(retval != bufferLength)
	{
		printk("%s: usb_control_msg returned %i\n", __FILE__, retval);
		kfree(buffer);
		return;
	}
	ni_usb_parse_status_block(buffer, &status);
	kfree(buffer);
}

//FIXME: prototype should return int
void ni_usb_primary_address(gpib_board_t *board, unsigned int address)
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	int i = 0;
	struct ni_usb_register writes[2];
	unsigned int ibsta;

	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(ADR);
	writes[i].value = address;
	i++;
	writes[i].device = NIUSB_SUBDEV_UNKNOWN2;
	writes[i].address = 0x0;
	writes[i].value = address;
	i++;
	retval = ni_usb_write_registers(ni_priv, writes, i, &ibsta);
	if(retval < 0)
	{
		printk("%s: %s: register write failed, retval=%i\n", __FILE__, __FUNCTION__, retval);
		return;// retval;
	}
	ni_usb_soft_update_status(board, ibsta, 0);
	return;// 0;
}

int ni_usb_write_sad(struct ni_usb_register *writes, int address, int enable)
{
	unsigned int adr_bits, admr_bits;
	int i = 0;

	adr_bits = HR_ARS;
	admr_bits = HR_TRM0 | HR_TRM1;
	if(enable)
	{
		adr_bits |= address;
		admr_bits |= HR_ADM1;
	}else
	{
		adr_bits |= HR_DT | HR_DL;
		admr_bits |= HR_ADM0;
	}
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(ADR);
	writes[i].value = adr_bits;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(ADMR);
	writes[i].value = admr_bits;
	i++;
	writes[i].device = NIUSB_SUBDEV_UNKNOWN2;
	writes[i].address = 0x1;
	writes[i].value = enable ? MSA(address) : 0x0;
	i++;
	return i;
}

void ni_usb_secondary_address(gpib_board_t *board, unsigned int address, int enable)
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	int i = 0;
	struct ni_usb_register writes[3];
	unsigned int ibsta;

	i += ni_usb_write_sad(writes, address, enable);
	retval = ni_usb_write_registers(ni_priv, writes, i, &ibsta);
	if(retval < 0)
	{
		printk("%s: %s: register write failed, retval=%i\n", __FILE__, __FUNCTION__, retval);
		return; // retval;
	}
	ni_usb_soft_update_status(board, ibsta, 0);
	return; // 0;
}
/* The firmware's IBRPP request leaves ATN+EOI asserted (permanent IDY state):
 * a device polled in a loop can never make progress, so e.g. an AMIGO drive
 * never reaches "response ready" and callers (HPDir) poll forever.  Worse,
 * releasing IDY afterwards with AUX_GTS breaks the firmware's addressing
 * state for the next write.  Instead, execute the parallel poll entirely at
 * chip level like the nec7210/tnt4882 drivers do: AUXMR=AUX_EPP runs the
 * hardware-timed IDY pulse and latches the DIO image into CPTR.  No
 * controller state transition, the firmware is none the wiser. */
static int ni_usb_parallel_poll_epp(gpib_board_t *board, uint8_t *result)
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	uint8_t *out_data, *in_data;
	int out_data_length, in_data_length;
	int bytes_written = 0, bytes_read = 0;
	int i = 0;
	unsigned int cptr_bits = 0;
	struct ni_usb_register reg;
	unsigned int reg_ibsta;

	reg.device = NIUSB_SUBDEV_TNT4882;
	reg.address = nec7210_to_tnt4882_offset(AUXMR);
	reg.value = AUX_EPP;
	retval = ni_usb_write_registers(ni_priv, &reg, 1, &reg_ibsta);
	if(retval)
	{
		printk("%s: %s: AUX_EPP register write failed, retval=%i\n",
			__FILE__, __FUNCTION__, retval);
		return retval;
	}
	/* the USB round trip above dwarfs the poll itself */
	out_data_length = 0x20;
	out_data = kmalloc(out_data_length, GFP_KERNEL);
	if(out_data == NULL) return -ENOMEM;
	i += ni_usb_bulk_register_read_header(&out_data[i], 1);
	i += ni_usb_bulk_register_read(&out_data[i], NIUSB_SUBDEV_TNT4882,
		nec7210_to_tnt4882_offset(CPTR));
	while(i % 4)
		out_data[i++] = 0x0;
	i += ni_usb_bulk_termination(&out_data[i]);
	mutex_lock(&ni_priv->addressed_transfer_lock);
	retval = ni_usb_send_bulk_msg(ni_priv, out_data, i, &bytes_written, 1000);
	kfree(out_data);
	if(retval || bytes_written != i)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		printk("%s: %s: ni_usb_send_bulk_msg returned %i, bytes_written=%i, i=%i\n",
			__FILE__, __FUNCTION__, retval, bytes_written, i);
		return retval;
	}
	in_data_length = 0x20;
	in_data = kmalloc(in_data_length, GFP_KERNEL);
	if(in_data == NULL)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		return -ENOMEM;
	}
	retval = ni_usb_receive_bulk_msg(ni_priv, in_data, in_data_length, &bytes_read, 1000, 1);
	mutex_unlock(&ni_priv->addressed_transfer_lock);
	if(retval && retval != -ERESTARTSYS)
	{
		printk("%s: %s: ni_usb_receive_bulk_msg returned %i, bytes_read=%i\n",
			__FILE__, __FUNCTION__, retval, bytes_read);
		kfree(in_data);
		return retval;
	}
	ni_usb_parse_register_read_block(in_data, &cptr_bits, 1);
	kfree(in_data);
	*result = cptr_bits & 0xff;
	printk("NIDBG rpp(epp): result=0x%02x bytes_read=%i\n", *result, bytes_read);
	return 0;
}

int ni_usb_parallel_poll(gpib_board_t *board, uint8_t *result)
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	uint8_t *out_data, *in_data;
	int out_data_length, in_data_length;
	int bytes_written = 0, bytes_read = 0;
	int i = 0;
	int j = 0;
	struct ni_usb_status_block status;

	if(rpp_force)
	{
		ni_priv->rpp_poll_count++;
		if(rpp_force_transition && ni_priv->wrote_since_rpp)
		{
			ni_priv->wrote_since_rpp = 0;
			*result = 0;	/* busy : la transition viendra au poll suivant */
			printk("NIDBG rpp: forced transition busy (poll #%i)\n",
				ni_priv->rpp_poll_count);
			return 0;
		}
		if((ni_priv->rpp_poll_count % 100) == 1)
			printk("NIDBG rpp: forced 0x%02x (poll #%i)\n",
				rpp_force, ni_priv->rpp_poll_count);
		*result = (uint8_t)rpp_force;
		return 0;
	}

	out_data_length = 0x10;
	out_data = kmalloc(out_data_length, GFP_KERNEL);
	if(out_data == NULL)
	{
		printk("%s: kmalloc failed\n", __FILE__);
		return -ENOMEM;
	}
	out_data[i++] = NIUSB_IBRPP_ID;
	/* 0xf0 is the only byte the firmware accepts here (0xf3 wedged it
	 * completely — send timeouts).  The poll's ATN+EOI then linger until
	 * the next request; do NOT try to release them out-of-band: IBGTS,
	 * take_control, AUXMR and BCR pokes all corrupt the engine's
	 * addressing model (commands stop latching TACS, writes die). */
	out_data[i++] = 0xf0;
	out_data[i++] = 0x0;
	out_data[i++] = 0x0;
	i += ni_usb_bulk_termination(&out_data[i]);
	/* serialize the send/receive pair like the other addressed transfers
	 * (this function was the only one left without the lock) */
	mutex_lock(&ni_priv->addressed_transfer_lock);
	retval = ni_usb_send_bulk_msg(ni_priv, out_data, i, &bytes_written, 1000 /*FIXME: should use parallel poll timeout (not supported yet)*/);
	kfree(out_data);
	if(retval || bytes_written != i)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		printk("%s: %s: ni_usb_send_bulk_msg returned %i, bytes_written=%i, i=%i\n", __FILE__, __FUNCTION__, retval, bytes_written, i);
		return retval;
	}
	in_data_length = 0x20;
	in_data = kmalloc(in_data_length, GFP_KERNEL);
	if(in_data == NULL)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		printk("%s: kmalloc failed\n", __FILE__);
		return -ENOMEM;
	}
	retval = ni_usb_receive_bulk_msg(ni_priv, in_data, in_data_length,
		&bytes_read, 1000 /*FIXME: should use parallel poll timeout (not supported yet)*/, 1);
	mutex_unlock(&ni_priv->addressed_transfer_lock);
	if(retval && retval != -ERESTARTSYS)
	{
		printk("%s: %s: ni_usb_receive_bulk_msg returned %i, bytes_read=%i\n", __FILE__, __FUNCTION__, retval, bytes_read);
		kfree(in_data);
		return retval;
	}
	j += ni_usb_parse_status_block(in_data, &status);
	/* fw quirk: the result byte contains raw DIO line levels, active-low
	 * AND bit-mirrored (DIO1 in bit 7) — e.g. drives at addresses 0 and 2
	 * responding read as raw 0x5f.  Standard convention (register-level
	 * drivers, hpdir): active-high, DIO1 in bit 0, i.e.
	 * device at address A asserts bit (1 << A).  raw 0x5f -> 0x05. */
	/* fw quirk: raw DIO line levels, active low (a responding device reads
	 * as 0).  Invert only — callers (hpdir) use the HP DIO(8-A) bit
	 * orientation as delivered by register-level drivers: device at
	 * address A responds on bit (0x80 >> A); e.g. drive at address 2 =
	 * 0x20.  (A bitrev8 to the 1<<A convention made hpdir's poll gate
	 * spin forever — its -query display remaps, its internal mask does not.) */
	*result = (uint8_t)(~in_data[j]);
	j++;
	printk("NIDBG rpp: err=%i ibsta=0x%x raw=0x%02x result=0x%02x bytes_read=%i id=0x%x\n",
		status.error_code, status.ibsta, in_data[j - 1], *result, bytes_read, status.id);
	if(status.id != NIUSB_IBRPP_ID)	/* response shifted: not an rpp status */
		ni_priv->pipe_dirty = 1;
	kfree(in_data);
	ni_usb_soft_update_status(board, status.ibsta, 0);
	/* The firmware holds the poll's IDY (ATN+EOI) until the NEXT request
	 * arrives — milliseconds on USB versus microseconds on PCI.  An AMIGO
	 * drive that has a response pending ABORTS it when it sees such an endless
	 * parallel poll (hpdir's status read then gets 1 byte of 0x00).
	 * Terminate the IDY quickly with the one benign request type that
	 * never corrupted the engine: a register READ (BSR, as line_status
	 * does).  IBGTS / take_control / AUXMR / BCR pokes all broke the
	 * engine's addressing model — do not use them here. */
	{
		uint8_t *rr_out, *rr_in;
		int rr_i = 0;
		int rr_written = 0, rr_read = 0;
		int rr_retval;

		rr_out = kmalloc(0x20, GFP_KERNEL);
		rr_in = kmalloc(0x20, GFP_KERNEL);
		if(rr_out && rr_in)
		{
			rr_i += ni_usb_bulk_register_read_header(&rr_out[rr_i], 1);
			rr_i += ni_usb_bulk_register_read(&rr_out[rr_i], NIUSB_SUBDEV_TNT4882, BSR);
			while(rr_i % 4)
				rr_out[rr_i++] = 0x0;
			rr_i += ni_usb_bulk_termination(&rr_out[rr_i]);
			mutex_lock(&ni_priv->addressed_transfer_lock);
			rr_retval = ni_usb_send_bulk_msg(ni_priv, rr_out, rr_i, &rr_written, 1000);
			if(rr_retval == 0 && rr_written == rr_i)
				ni_usb_receive_bulk_msg(ni_priv, rr_in, 0x20, &rr_read, 1000, 0);
			mutex_unlock(&ni_priv->addressed_transfer_lock);
		}
		kfree(rr_out);
		kfree(rr_in);
	}
	return retval;
}
void ni_usb_parallel_poll_configure(gpib_board_t *board, uint8_t config)
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	int i = 0;
	struct ni_usb_register writes[1];
	unsigned int ibsta;

	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
	writes[i].value = PPR | config;
	i++;
	retval = ni_usb_write_registers(ni_priv, writes, i, &ibsta);
	if(retval < 0)
	{
		printk("%s: %s: register write failed, retval=%i\n", __FILE__, __FUNCTION__, retval);
		return;// retval;
	}
	ni_usb_soft_update_status(board, ibsta, 0);
	return;// 0;
}
void ni_usb_parallel_poll_response(gpib_board_t *board, int ist)
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	int i = 0;
	struct ni_usb_register writes[1];
	unsigned int ibsta;

	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
	if(ist)
		writes[i].value = AUX_SPPF;
	else
		writes[i].value = AUX_CPPF;
	i++;
	retval = ni_usb_write_registers(ni_priv, writes, i, &ibsta);
	if(retval < 0)
	{
		printk("%s: %s: register write failed, retval=%i\n", __FILE__, __FUNCTION__, retval);
		return;// retval;
	}
	ni_usb_soft_update_status(board, ibsta, 0);
	return;// 0;
}
void ni_usb_serial_poll_response(gpib_board_t *board, uint8_t status)
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	int i = 0;
	struct ni_usb_register writes[1];
	unsigned int ibsta;

	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(SPMR);
	writes[i].value = status;
	i++;
	retval = ni_usb_write_registers(ni_priv, writes, i, &ibsta);
	if(retval < 0)
	{
		printk("%s: %s: register write failed, retval=%i\n", __FILE__, __FUNCTION__, retval);
		return;// retval;
	}
	ni_usb_soft_update_status(board, ibsta, 0);
	return;// 0;
}
uint8_t ni_usb_serial_poll_status( gpib_board_t *board )
{
	return 0;
}
void ni_usb_return_to_local( gpib_board_t *board )
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	int i = 0;
	struct ni_usb_register writes[1];
	unsigned int ibsta;

	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
	writes[i].value = AUX_RTL;
	i++;
	retval = ni_usb_write_registers(ni_priv, writes, i, &ibsta);
	if(retval < 0)
	{
		printk("%s: %s: register write failed, retval=%i\n", __FILE__, __FUNCTION__, retval);
		return;// retval;
	}
	ni_usb_soft_update_status(board, ibsta, 0);
	return;// 0;
}
int ni_usb_line_status( const gpib_board_t *board )
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	uint8_t *out_data, *in_data;
	int out_data_length, in_data_length;
	int bytes_written = 0, bytes_read = 0;
	int i = 0;
	unsigned int bsr_bits;
	int line_status = ValidALL;
	// NI windows driver reads 0xd(HSSEL), 0xc (ARD0), 0x1f (BSR)

	out_data_length = 0x20;
	out_data = kmalloc(out_data_length, GFP_KERNEL);
	if(out_data == NULL)
	{
		printk("%s: kmalloc failed\n", __FILE__);
		return -ENOMEM;
	}
	i += ni_usb_bulk_register_read_header(&out_data[i], 1);
	i += ni_usb_bulk_register_read(&out_data[i], NIUSB_SUBDEV_TNT4882, BSR);
	while(i % 4)
		out_data[i++] = 0x0;
	i += ni_usb_bulk_termination(&out_data[i]);
	/* Blocking lock (modern driver uses trylock+-EBUSY, but the 4.0.3
	 * userspace lib doesn't tolerate -EBUSY from iblines). The point is
	 * the same: never interleave this register read with an addressed
	 * transfer's send/receive pair. */
	mutex_lock(&ni_priv->addressed_transfer_lock);
	retval = ni_usb_nonblocking_send_bulk_msg(ni_priv, out_data, i, &bytes_written, 1000);
	kfree(out_data);
	if(retval || bytes_written != i)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		if(retval != -EAGAIN)
			printk("%s: %s: ni_usb_send_bulk_msg returned %i, bytes_written=%i, i=%i\n", __FILE__, __FUNCTION__, retval, bytes_written, i);
		return retval;
	}
	in_data_length = 0x20;
	in_data = kmalloc(in_data_length, GFP_KERNEL);
	if(in_data == NULL)
	{
		mutex_unlock(&ni_priv->addressed_transfer_lock);
		printk("%s: kmalloc failed\n", __FILE__);
		return -ENOMEM;
	}
	retval = ni_usb_nonblocking_receive_bulk_msg(ni_priv, in_data, in_data_length, &bytes_read, 1000, 0);
	mutex_unlock(&ni_priv->addressed_transfer_lock);
	if(retval)
	{
		if(retval != -EAGAIN)
			printk("%s: %s: ni_usb_receive_bulk_msg returned %i, bytes_read=%i\n", __FILE__, __FUNCTION__, retval, bytes_read);
		kfree(in_data);
		return retval;
	}
#if 0
{
int k;
printk("reg read response:\n");
for(k=0;k<bytes_read;k++)
	printk("%.2x ", in_data[k]);
printk("\n");
}
#endif
	ni_usb_parse_register_read_block(in_data, &bsr_bits, 1);
	kfree(in_data);
	if(bsr_bits & BCSR_REN_BIT)
		line_status |= BusREN;
	if(bsr_bits & BCSR_IFC_BIT)
		line_status |= BusIFC;
	if(bsr_bits & BCSR_SRQ_BIT)
		line_status |= BusSRQ;
	if(bsr_bits & BCSR_EOI_BIT)
		line_status |= BusEOI;
	if(bsr_bits & BCSR_NRFD_BIT)
		line_status |= BusNRFD;
	if(bsr_bits & BCSR_NDAC_BIT)
		line_status |= BusNDAC;
	if(bsr_bits & BCSR_DAV_BIT)
		line_status |= BusDAV;
	if(bsr_bits & BCSR_ATN_BIT)
		line_status |= BusATN;
	return line_status;
}
unsigned int ni_usb_t1_delay( gpib_board_t *board, unsigned int nano_sec )
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	int i = 0;
	struct ni_usb_register writes[3];
	unsigned int ibsta;
	unsigned int actual_ns = 2000;

	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
	if(nano_sec <= 1100)
	{
		writes[i].value = AUXRI | USTD | SISB;
		actual_ns = 1100;
	}else
		writes[i].value = AUXRI | SISB;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
	if(nano_sec <= 500)
	{
		writes[i].value = AUXRB | HR_TRI;
		actual_ns = 500;
	}else
		writes[i].value = AUXRB;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = KEYREG;
	if(nano_sec <= 350)
	{
		writes[i].value = MSTD;
		actual_ns = 350;
	}else
		writes[i].value = 0x0;
	i++;
	retval = ni_usb_write_registers(ni_priv, writes, i, &ibsta);
	if(retval < 0)
	{
		printk("%s: %s: register write failed, retval=%i\n", __FILE__, __FUNCTION__, retval);
		return -1;	//FIXME should change return type to int for error reporting
	}
	ni_usb_soft_update_status(board, ibsta, 0);
	return actual_ns;
}

static int ni_usb_allocate_private(gpib_board_t *board)
{
	ni_usb_private_t *ni_priv;

	board->private_data = kmalloc(sizeof(ni_usb_private_t), GFP_KERNEL);
	if(board->private_data == NULL)
		return -ENOMEM;
	ni_priv = board->private_data;
	memset(ni_priv, 0, sizeof(ni_usb_private_t));
	mutex_init(&ni_priv->bulk_transfer_lock);
	mutex_init(&ni_priv->control_transfer_lock);
	mutex_init(&ni_priv->interrupt_transfer_lock);
	mutex_init(&ni_priv->addressed_transfer_lock);
	return 0;
}

static void ni_usb_free_private(ni_usb_private_t *ni_priv)
{
	if(ni_priv->interrupt_urb)
		usb_free_urb(ni_priv->interrupt_urb);
	kfree(ni_priv);
	return;
}

static void ni_usb_set_address_mode(gpib_board_t *board, int address_mode, int sad);

static int ni_usb_init(gpib_board_t *board)
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	int i = 0;
	struct ni_usb_register *writes;
	unsigned int ibsta;
	static const int writes_length = 24;

	writes = kmalloc(sizeof(struct ni_usb_register) * writes_length, GFP_KERNEL);
	if(writes == NULL)
	{
		printk("%s: %s: kmalloc failed\n", __FILE__, __FUNCTION__);
		return -ENOMEM;
	}
	writes[i].device = NIUSB_SUBDEV_UNKNOWN3;
	writes[i].address = 0x10;
	writes[i].value = 0x0;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = CMDR;
	writes[i].value = SOFT_RESET;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
	writes[i].value = AUX_7210;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = SWAPPED_AUXCR;
	writes[i].value = AUX_7210;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = HSSEL;
	writes[i].value = TNT_ONE_CHIP_BIT;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
	writes[i].value = AUX_CR;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = IMR0;
	writes[i].value = TNT_IMR0_ALWAYS_BITS;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(IMR1);
	writes[i].value = 0x0;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(IMR2);
	writes[i].value = 0x0;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = IMR3;
	writes[i].value = 0x0;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
	writes[i].value = AUX_HLDI;
	i++;
	/* the following three writes should share code with set_t1_delay */
	board->t1_nano_sec = 500;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
	writes[i].value = AUXRI | SISB | USTD;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
	writes[i].value = AUXRB | HR_TRI;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = KEYREG;
	writes[i].value = 0x0;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
	writes[i].value = AUXRG | NTNL_BIT;
	i++;
#if 0	// request system control
	i += ni_usb_bulk_register_write(&out_data[i], NIUSB_SUBDEV_TNT4882, CMDR, SETSC);
	i += ni_usb_bulk_register_write(&out_data[i], NIUSB_SUBDEV_TNT4882, nec7210_to_tnt4882_offset(AUXMR), AUX_CIFC);
#endif
	// primary address
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(ADR);
	writes[i].value = board->pad;
	i++;
	writes[i].device = NIUSB_SUBDEV_UNKNOWN2;
	writes[i].address = 0x0;
	writes[i].value = board->pad;
	i++;
	// secondary address
	i += ni_usb_write_sad(&writes[i], board->sad, board->sad >= 0);
	// is this a timeout?
	writes[i].device = NIUSB_SUBDEV_UNKNOWN2;
	writes[i].address = 0x2;
	writes[i].value = 0xfd;
	i++;
	// what is this?  There is no documented tnt4882 register at offset 0xf
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = 0xf;
	writes[i].value = 0x11;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
	writes[i].value = AUX_PON;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
	writes[i].value = AUX_CPPF;
	i++;
	if(i > writes_length)
	{
		printk("%s: %s: bug!, buffer overrun, i=%i\n", __FILE__, __FUNCTION__, i);
		return -EINVAL;
	}
	retval = ni_usb_write_registers(ni_priv, writes, i, &ibsta);
	kfree(writes);
	if(retval)
	{
		printk("%s: %s: register write failed, retval=%i\n", __FILE__, __FUNCTION__, retval);
		return retval;
	}
	ni_usb_soft_update_status(board, ibsta, 0);
	/* Ansgar's patch put the chip in address mode 3 (major/minor, secondary
	 * validated by software via APT) at init, for the device-side HP
	 * identify hack.  In mode 3 the firmware never validates secondaries,
	 * so the chip never reaches TACS as a controller and every board write
	 * fails with addressing error 3.  Leave the firmware's default mode;
	 * callers that need the identify hack can still invoke
	 * SetAddressMode() explicitly. */
	/* ni_usb_set_address_mode(board, 3, 31); */
	return 0;
}

void ni_usb_interrupt_complete(struct urb *urb PT_REGS_ARG)
{
	gpib_board_t *board = urb->context;
	ni_usb_private_t *ni_priv = board->private_data;
	int retval;
	struct ni_usb_status_block status;
	unsigned long flags;

// 	printk("debug: %s: %s: status=0x%x, error_count=%i, actual_length=%i\n", __FILE__, __FUNCTION__,
// 		urb->status, urb->error_count, urb->actual_length);

	// don't resubmit if urb was unlinked
	if(urb->status) return;
	ni_usb_parse_status_block(urb->transfer_buffer, &status);
// 	printk("debug: ibsta=0x%x\n", status.ibsta);

	spin_lock_irqsave(&board->spinlock, flags);
//	ni_priv->monitored_ibsta_bits &= ~status.ibsta;
	ni_priv->monitored_ibsta_bits = 0;
// 	printk("debug: monitored_ibsta_bits=0x%x\n", ni_priv->monitored_ibsta_bits);
	spin_unlock_irqrestore(&board->spinlock, flags);

	retval = usb_submit_urb(ni_priv->interrupt_urb, GFP_ATOMIC);
	if(retval)
	{
		printk("%s: failed to resubmit interrupt urb\n", __FUNCTION__);
	}
	wake_up_interruptible( &board->wait );
}

static int ni_usb_set_interrupt_monitor(gpib_board_t *board, unsigned int monitored_bits)
{
	int retval;
	ni_usb_private_t *ni_priv = board->private_data;
	static const int bufferLength = 8;
	uint8_t *buffer;
	struct ni_usb_status_block status;
	unsigned long flags;
	//printk("%s: receive control pipe is %i\n", __FILE__, pipe);
	buffer = kmalloc(bufferLength, GFP_KERNEL);
	if(buffer == NULL)
	{
		printk("%s: kmalloc failed!\n", __FILE__);
		return -ENOMEM;
	}
	spin_lock_irqsave(&board->spinlock, flags);
	ni_priv->monitored_ibsta_bits = ni_usb_ibsta_monitor_mask & monitored_bits;
// 	printk("debug: %s: monitored_ibsta_bits=0x%x\n", __FUNCTION__, ni_priv->monitored_ibsta_bits);
	spin_unlock_irqrestore(&board->spinlock, flags);
	retval = ni_usb_receive_control_msg(ni_priv, NI_USB_WAIT_REQUEST, USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		0x300, ni_usb_ibsta_monitor_mask & monitored_bits, buffer, bufferLength, 1000);
	if(retval != bufferLength)
	{
		printk("%s: usb_control_msg returned %i\n", __FILE__, retval);
		kfree(buffer);
		return -1;
	}
	ni_usb_parse_status_block(buffer, &status);
	kfree(buffer);
	return 0;
}

static int ni_usb_setup_urbs(gpib_board_t *board)
{
	ni_usb_private_t *ni_priv = board->private_data;
	struct usb_device *usb_dev;
	int int_pipe;
	int retval;
	mutex_lock(&ni_priv->interrupt_transfer_lock);
	if(ni_priv->bus_interface == NULL)
	{
		mutex_unlock(&ni_priv->interrupt_transfer_lock);
		return -ENODEV;
	}
	ni_priv->interrupt_urb = usb_alloc_urb(0, GFP_KERNEL);
	if(ni_priv->interrupt_urb == NULL)
	{
		mutex_unlock(&ni_priv->interrupt_transfer_lock);
		return -ENOMEM;
	}
	usb_dev = interface_to_usbdev(ni_priv->bus_interface);
	int_pipe = usb_rcvintpipe(usb_dev, ni_priv->interrupt_in_endpoint);
	usb_fill_int_urb(ni_priv->interrupt_urb, usb_dev, int_pipe, ni_priv->interrupt_buffer,
		sizeof(ni_priv->interrupt_buffer), &ni_usb_interrupt_complete, board, 1);
	retval = usb_submit_urb(ni_priv->interrupt_urb, GFP_KERNEL);
	mutex_unlock(&ni_priv->interrupt_transfer_lock);
	if(retval)
	{
		printk("%s: failed to submit first interrupt urb, retval=%i\n", __FILE__, retval);
		return retval;
	}
	return 0;
}


static void ni_usb_cleanup_urbs(ni_usb_private_t *ni_priv)
{
	if(ni_priv && ni_priv->bus_interface)
	{
		if(ni_priv->interrupt_urb)
			usb_kill_urb(ni_priv->interrupt_urb);
		if(ni_priv->bulk_urb)
			usb_kill_urb(ni_priv->bulk_urb);
	}
};

static int ni_usb_b_read_serial_number(ni_usb_private_t *ni_priv)
{
	int retval;
	uint8_t out_data[0x20];
	uint8_t in_data[0x20];
	int bytes_written = 0, bytes_read = 0;
	int i = 0;
	static const int num_reads = 4;
	unsigned results[4];
	int j;
	unsigned serial_number;
// 	printk("%s: %s\n", __FILE__, __FUNCTION__);
	i += ni_usb_bulk_register_read_header(&out_data[i], num_reads);
	i += ni_usb_bulk_register_read(&out_data[i], NIUSB_SUBDEV_UNKNOWN3, SERIAL_NUMBER_1_REG);
	i += ni_usb_bulk_register_read(&out_data[i], NIUSB_SUBDEV_UNKNOWN3, SERIAL_NUMBER_2_REG);
	i += ni_usb_bulk_register_read(&out_data[i], NIUSB_SUBDEV_UNKNOWN3, SERIAL_NUMBER_3_REG);
	i += ni_usb_bulk_register_read(&out_data[i], NIUSB_SUBDEV_UNKNOWN3, SERIAL_NUMBER_4_REG);
	while(i % 4)
		out_data[i++] = 0x0;
	i += ni_usb_bulk_termination(&out_data[i]);
	retval = ni_usb_send_bulk_msg(ni_priv, out_data, sizeof(out_data), &bytes_written, 1000);
	if(retval)
	{
		printk("%s: %s: ni_usb_send_bulk_msg returned %i, bytes_written=%i, i=%li\n", __FILE__, __FUNCTION__,
			retval, bytes_written, (long)(sizeof(out_data)));
		return retval;
	}
	retval = ni_usb_receive_bulk_msg(ni_priv, in_data, sizeof(in_data), &bytes_read, 1000, 0);
	if(retval)
	{
		printk("%s: %s: ni_usb_receive_bulk_msg returned %i, bytes_read=%i\n", __FILE__, __FUNCTION__, retval, bytes_read);
		ni_usb_dump_raw_block(in_data, bytes_read);
		return retval;
	}
	if(sizeof(results) / sizeof(results[0]) < num_reads) BUG();
	ni_usb_parse_register_read_block(in_data, results, num_reads);
	serial_number = 0;
	for(j = 0; j < num_reads; ++j)
	{
		serial_number |= (results[j] & 0xff) << (8 * j);
	}
	printk("%s: board serial number is 0x%x\n", __FUNCTION__, serial_number);
	return 0;
}

static int ni_usb_hs_wait_for_ready(ni_usb_private_t *ni_priv)
{
	uint8_t buffer[0x10] = {0};
	int retval;
	static const int timeout = 50;
	static const int msec_sleep_duration = 100;
	int i;
	int j;
	int unexpected = 0;
	unsigned serial_number;
	retval = ni_usb_receive_control_msg(ni_priv, NI_USB_SERIAL_NUMBER_REQUEST, USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		0x0, 0x0, buffer, sizeof(buffer), 1000);
	if(retval < 0)
	{
		printk("%s: usb_control_msg request 0x%x returned %i\n", __FILE__, NI_USB_SERIAL_NUMBER_REQUEST, retval);
		return retval;
	}
	j = 0;
	if(buffer[j] != NI_USB_SERIAL_NUMBER_REQUEST)
	{
		printk("%s: %s: unexpected data: buffer[%i]=0x%x, expected 0x%x\n",
			__FILE__, __FUNCTION__, j, (int)buffer[j], NI_USB_SERIAL_NUMBER_REQUEST);
		unexpected = 1;
	}
	if(unexpected)
		ni_usb_dump_raw_block(buffer, retval);
	if(retval != 5)
	{
		printk("%s: %s: received unexpected number of bytes = %i, expected 5\n",
			__FILE__, __FUNCTION__, retval);
		ni_usb_dump_raw_block(buffer, retval);
	}
	serial_number = 0;
	serial_number |= buffer[++j];
	serial_number |= (buffer[++j] << 8);
	serial_number |= (buffer[++j] << 16);
	serial_number |= (buffer[++j] << 24);
	printk("%s: board serial number is 0x%x\n", __FUNCTION__, serial_number);
	for(i = 0; i < timeout; ++i)
	{
		int ready = 0;
		retval = ni_usb_receive_control_msg(ni_priv, NI_USB_POLL_READY_REQUEST, USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0x0, 0x0, buffer, sizeof(buffer), 100);
		if(retval < 0)
		{
			printk("%s: usb_control_msg request 0x%x returned %i\n", __FILE__, NI_USB_POLL_READY_REQUEST, retval);
			return retval;
		}
		j = 0;
		unexpected = 0;
		if(buffer[j] != NI_USB_POLL_READY_REQUEST)
		{
			printk("%s: %s: unexpected data: buffer[%i]=0x%x, expected 0x%x\n",
				__FILE__, __FUNCTION__, j, (int)buffer[j], NI_USB_POLL_READY_REQUEST);
			unexpected = 1;
		}
		if(buffer[++j] != 0x1)
		{
			printk("%s: %s: unexpected data: buffer[%i]=0x%x, expected 0x%x\n",
				__FILE__, __FUNCTION__, j, (int)buffer[j], 0x1);
			unexpected = 1;
		}
		if(buffer[++j] != 0x0)
		{
			printk("%s: %s: unexpected data: buffer[%i]=0x%x, expected 0x%x\n",
				__FILE__, __FUNCTION__, j, (int)buffer[j], 0x0);
			unexpected = 1;
		}
		++j;
		// MC usb-488 (and sometimes NI-USB-HS?) sends 0x8 here; MC usb-488A sends 0x7 here
		if(buffer[j] != 0x1 && buffer[j] != 0x8 && buffer[j] != 0x7 )
		{
			printk("%s: %s: unexpected data: buffer[%i]=0x%x, expected 0x1, 0x7 or 0x8\n",
				__FILE__, __FUNCTION__, j, (int)buffer[j]);
			unexpected = 1;
		}
		if(buffer[++j] != 0x30)
		{
			printk("%s: %s: unexpected data: buffer[%i]=0x%x, expected 0x%x\n",
				__FILE__, __FUNCTION__, j, (int)buffer[j], 0x30);
			unexpected = 1;
		}
		++j;
		if(buffer[j] != 0x1 && buffer[j] != 0x0) // MC usb-488 (and sometimes NI-USB-HS?) sends 0x0 here
		{
			printk("%s: %s: unexpected data: buffer[%i]=0x%x, expected 0x1 or 0x0\n",
				__FILE__, __FUNCTION__, j, (int)buffer[j]);
			unexpected = 1;
		}
		if(buffer[++j] != 0x0)
		{
			ready = 1;
			if(buffer[j] != 0x2 && buffer[j] != 0xe)
			{
				printk("%s: %s: unexpected data: buffer[%i]=0x%x, expected 0x%x\n",
					__FILE__, __FUNCTION__, j, (int)buffer[j], 0x2);
				unexpected = 1;
			}
		}
		if(buffer[++j] != 0x0)
		{
			ready = 1;
			// MC usb-488 sends 0x5 here; MC usb-488A sends 0x6 here
			if(buffer[j] != 0x3 && buffer[j] != 0x5 && buffer[j] != 0x6)
			{
				printk("%s: %s: unexpected data: buffer[%i]=0x%x, expected 0x3 or 0x5 or 0x6\n",
					__FILE__, __FUNCTION__, j, (int)buffer[j]);
				unexpected = 1;
			}
		}
		++j;
		if(buffer[j] != 0x0 && buffer[j] != 0x2) // MC usb-488 sends 0x2 here
		{
			printk("%s: %s: unexpected data: buffer[%i]=0x%x, expected 0x0 ox 0x2\n",
				__FILE__, __FUNCTION__, j, (int)buffer[j]);
			unexpected = 1;
		}
		++j;
		// MC usb-488A and NI-USB-HS sends 0x3 here
		if(buffer[j] != 0x0 && buffer[j] != 0x3)
		{
			printk("%s: %s: unexpected data: buffer[%i]=0x%x, expected 0x0 or 0x3\n",
				__FILE__, __FUNCTION__, j, (int)buffer[j]);
			unexpected = 1;
		}
		if(buffer[++j] != 0x0)
		{
			ready = 1;
			if(buffer[j] != 0x96 && buffer[j] != 0x7) // MC usb-488 sends 0x7 here
			{
				printk("%s: %s: unexpected data: buffer[%i]=0x%x, expected 0x96 or 0x07\n",
					__FILE__, __FUNCTION__, j, (int)buffer[j]);
				unexpected = 1;
			}
		}
		if(unexpected)
			ni_usb_dump_raw_block(buffer, retval);
		if(ready) break;
		retval = msleep_interruptible(msec_sleep_duration);
		if(retval)
		{
			printk("ni_usb_gpib: msleep interrupted\n");
			return -ERESTARTSYS;
		}
	}
	return 0;
}

int ni_usb_attach(gpib_board_t *board, gpib_board_config_t config)
{
	int retval;
	int i;
	ni_usb_private_t *ni_priv;
	int product_id;

	printk("ni_usb_gpib: attach\n");
	mutex_lock(&ni_usb_hotplug_lock);
	retval = ni_usb_allocate_private(board);
	if(retval < 0)
	{
		mutex_unlock(&ni_usb_hotplug_lock);
		return retval;
	}
	ni_priv = board->private_data;
	/*FIXME: should allow user to specifiy which device he wants to attach.
	 Use usb_make_path() */
	for(i = 0; i < MAX_NUM_NI_USB_INTERFACES; i++)
	{
		if(ni_usb_driver_interfaces[i] && usb_get_intfdata(ni_usb_driver_interfaces[i]) == NULL)
		{
			ni_priv->bus_interface = ni_usb_driver_interfaces[i];
			usb_set_intfdata(ni_usb_driver_interfaces[i], board);
			printk("\tattached to bus interface %i, address 0x%p\n", i, ni_priv->bus_interface);
			break;
		}
	}
	if(i == MAX_NUM_NI_USB_INTERFACES)
	{
		mutex_unlock(&ni_usb_hotplug_lock);
		printk("No supported NI usb gpib adapters found, have you loaded its firmware?\n");
		return -ENODEV;
	}
	if(usb_reset_configuration(interface_to_usbdev(ni_priv->bus_interface)))
	{
		printk("ni_usb_gpib: usb_reset_configuration() failed.\n");
	}
	product_id = USBID_TO_CPU(interface_to_usbdev(ni_priv->bus_interface)->descriptor.idProduct);
	printk("\tproduct id=0x%x\n", product_id);
	if(product_id == USB_DEVICE_ID_NI_USB_B)
	{
		ni_priv->bulk_out_endpoint = NIUSB_B_BULK_OUT_ENDPOINT;
		ni_priv->bulk_in_endpoint = NIUSB_B_BULK_IN_ENDPOINT;
		ni_priv->interrupt_in_endpoint = NIUSB_B_INTERRUPT_IN_ENDPOINT;
		ni_usb_b_read_serial_number(ni_priv);
	}else if(product_id == USB_DEVICE_ID_NI_USB_HS || product_id == USB_DEVICE_ID_MC_USB_488 || product_id == USB_DEVICE_ID_KUSB_488A)
	{
		ni_priv->bulk_out_endpoint = NIUSB_HS_BULK_OUT_ENDPOINT;
		ni_priv->bulk_in_endpoint = NIUSB_HS_BULK_IN_ENDPOINT;
		ni_priv->interrupt_in_endpoint = NIUSB_HS_INTERRUPT_IN_ENDPOINT;
		retval = ni_usb_hs_wait_for_ready(ni_priv);
		if(retval < 0)
		{
			mutex_unlock(&ni_usb_hotplug_lock);
			return retval;
		}
	}else
	{
		mutex_unlock(&ni_usb_hotplug_lock);
		printk("\tDriver bug: unknown endpoints for usb device id\n");
		return -EINVAL;
	}
	/* Drain stale responses possibly left in the bulk-in pipe by an
	 * earlier desync (orphaned response after a killed urb), so that a
	 * simple reattach (gpib_config) resyncs the pipe without having to
	 * physically replug the adapter. */
	{
		uint8_t *scratch;
		int drained;
		int n;

		scratch = kmalloc(0x200, GFP_KERNEL);
		if(scratch)
		{
			for(n = 0; n < 8; n++)
			{
				if(ni_usb_receive_bulk_msg(ni_priv, scratch, 0x200,
					&drained, 100, 0) < 0)
					break;
				if(drained <= 0) break;
				printk("ni_usb_gpib: attach: drained %i stale byte(s) from bulk-in pipe\n",
					drained);
			}
			kfree(scratch);
		}
	}
	retval = ni_usb_setup_urbs(board);
	if(retval < 0)
	{
		mutex_unlock(&ni_usb_hotplug_lock);
		return retval;
	}
	retval = ni_usb_set_interrupt_monitor(board, 0);
	if(retval < 0)
	{
		mutex_unlock(&ni_usb_hotplug_lock);
		return retval;
	}
	retval = ni_usb_init(board);
	if(retval < 0)
	{
		mutex_unlock(&ni_usb_hotplug_lock);
		return retval;
	}
	retval = ni_usb_set_interrupt_monitor(board, ni_usb_ibsta_monitor_mask);
	if(retval < 0)
	{
		mutex_unlock(&ni_usb_hotplug_lock);
		return retval;
	}
	mutex_unlock(&ni_usb_hotplug_lock);
	return retval;
}

static int ni_usb_shutdown_hardware(ni_usb_private_t *ni_priv)
{
	int retval;
	int i = 0;
	struct ni_usb_register writes[2];
	static const int writes_length = sizeof(writes) / sizeof(writes[0]);
	unsigned int ibsta;

// 	printk("%s: %s\n", __FILE__, __FUNCTION__);
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(AUXMR);
	writes[i].value = AUX_CR;
	i++;
	writes[i].device = NIUSB_SUBDEV_UNKNOWN3;
	writes[i].address = 0x10;
	writes[i].value = 0x0;
	i++;
	if(i > writes_length)
	{
		printk("%s: %s: bug!, buffer overrun, i=%i\n", __FILE__, __FUNCTION__, i);
		return -EINVAL;
	}
	retval = ni_usb_write_registers(ni_priv, writes, i, &ibsta);
	if(retval)
	{
		printk("%s: %s: register write failed, retval=%i\n", __FILE__, __FUNCTION__, retval);
		return retval;
	}
	return 0;
}

void ni_usb_detach(gpib_board_t *board)
{
	ni_usb_private_t *ni_priv;

	mutex_lock(&ni_usb_hotplug_lock);
//	printk("%s: enter\n", __FUNCTION__);
// under windows, software unplug does chip_reset nec7210 aux command, then writes 0x0 to address 0x10 of device 3
	ni_priv = board->private_data;
	if(ni_priv)
	{
		if(ni_priv->bus_interface)
		{
			ni_usb_set_interrupt_monitor(board, 0);
			ni_usb_shutdown_hardware(ni_priv);
			usb_set_intfdata(ni_priv->bus_interface, NULL);
		}
		mutex_lock(&ni_priv->bulk_transfer_lock);
		mutex_lock(&ni_priv->control_transfer_lock);
		mutex_lock(&ni_priv->interrupt_transfer_lock);
		ni_usb_cleanup_urbs(ni_priv);
		ni_usb_free_private(ni_priv);
	}
//	printk("%s: exit\n", __FUNCTION__);
	mutex_unlock(&ni_usb_hotplug_lock);
}

static void ni_usb_set_address_mode(gpib_board_t *board, int address_mode, int sad)
{
	ni_usb_private_t *ni_priv = board->private_data;
	struct ni_usb_register writes[2];
	unsigned int ibsta;
	int i = 0;
	if(!ni_priv->bus_interface) return;
	ni_priv->address_mode = address_mode;
	ni_priv->hp_sad = sad & 0x1f;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(ADMR);
	writes[i].value = HR_TRM0 | HR_TRM1 | HR_ADM0 | HR_ADM1;
	i++;
	writes[i].device = NIUSB_SUBDEV_TNT4882;
	writes[i].address = nec7210_to_tnt4882_offset(ADR);
	writes[i].value = HR_ARS | (sad & 0x1f);
	i++;
	ni_usb_write_registers(ni_priv, writes, i, &ibsta);
	ni_usb_soft_update_status(board, ibsta, 0);
}

gpib_interface_t ni_usb_gpib_interface =
{
	name: "ni_usb_b",
	attach: ni_usb_attach,
	detach: ni_usb_detach,
	read: ni_usb_read,
	write: ni_usb_write,
	command: ni_usb_command,
	take_control: ni_usb_take_control,
	go_to_standby: ni_usb_go_to_standby,
	request_system_control: ni_usb_request_system_control,
	interface_clear: ni_usb_interface_clear,
	remote_enable: ni_usb_remote_enable,
	enable_eos: ni_usb_enable_eos,
	disable_eos: ni_usb_disable_eos,
	parallel_poll: ni_usb_parallel_poll,
	parallel_poll_configure: ni_usb_parallel_poll_configure,
	parallel_poll_response: ni_usb_parallel_poll_response,
	line_status: ni_usb_line_status,
	update_status: ni_usb_update_status,
	primary_address: ni_usb_primary_address,
	secondary_address: ni_usb_secondary_address,
	serial_poll_response: ni_usb_serial_poll_response,
	serial_poll_status: ni_usb_serial_poll_status,
	t1_delay: ni_usb_t1_delay,
	return_to_local: ni_usb_return_to_local,
	set_address_mode: ni_usb_set_address_mode,
};

// Table with the USB-devices: just now only testing IDs
static struct usb_device_id ni_usb_driver_device_table [] =
{
	{USB_DEVICE(USB_VENDOR_ID_NI, USB_DEVICE_ID_NI_USB_B)},
	{USB_DEVICE(USB_VENDOR_ID_NI, USB_DEVICE_ID_NI_USB_HS)},
	{USB_DEVICE(USB_VENDOR_ID_NI, USB_DEVICE_ID_KUSB_488A)},
	{USB_DEVICE(USB_VENDOR_ID_NI, USB_DEVICE_ID_MC_USB_488)},
	{} /* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, ni_usb_driver_device_table);

static int ni_usb_driver_probe(struct usb_interface *interface,
	const struct usb_device_id *id)
{
	int i;
	char *path;
	static const int pathLength = 1024;

//	printk("ni_usb_driver_probe\n");
	mutex_lock(&ni_usb_hotplug_lock);
	usb_get_dev(interface_to_usbdev(interface));
	for(i = 0; i < MAX_NUM_NI_USB_INTERFACES; i++)
	{
		if(ni_usb_driver_interfaces[i] == NULL)
		{
			ni_usb_driver_interfaces[i] = interface;
			usb_set_intfdata(interface, NULL);
//			printk("set bus interface %i to address 0x%p\n", i, interface);
			break;
		}
	}
	if(i == MAX_NUM_NI_USB_INTERFACES)
	{
		usb_put_dev(interface_to_usbdev(interface));
		mutex_unlock(&ni_usb_hotplug_lock);
		printk("ni_usb_gpib: out of space in ni_usb_driver_interfaces[]\n");
		return -1;
	}
	path = kmalloc(pathLength, GFP_KERNEL);
	if(path == NULL)
	{
		usb_put_dev(interface_to_usbdev(interface));
		mutex_unlock(&ni_usb_hotplug_lock);
		return -ENOMEM;
	}
	usb_make_path(interface_to_usbdev(interface), path, pathLength);
	printk("ni_usb_gpib: probe succeeded for path: %s\n", path);
	kfree(path);
	mutex_unlock(&ni_usb_hotplug_lock);
	return 0;
}

static void ni_usb_driver_disconnect(struct usb_interface *interface)
{
	int i;

	mutex_lock(&ni_usb_hotplug_lock);
//	printk("%s: enter\n", __FUNCTION__);
	for(i = 0; i < MAX_NUM_NI_USB_INTERFACES; i++)
	{
		if(ni_usb_driver_interfaces[i] == interface)
		{
			gpib_board_t *board = usb_get_intfdata(interface);

			if(board)
			{
				ni_usb_private_t *ni_priv = board->private_data;
				if(ni_priv)
				{
					mutex_lock(&ni_priv->bulk_transfer_lock);
					mutex_lock(&ni_priv->control_transfer_lock);
					mutex_lock(&ni_priv->interrupt_transfer_lock);
					ni_usb_cleanup_urbs(ni_priv);
					ni_priv->bus_interface = NULL;
					mutex_unlock(&ni_priv->interrupt_transfer_lock);
					mutex_unlock(&ni_priv->control_transfer_lock);
					mutex_unlock(&ni_priv->bulk_transfer_lock);
				}
			}
//			printk("nulled ni_usb_driver_interfaces[%i]\n", i);
			ni_usb_driver_interfaces[i] = NULL;
			break;
		}
	}
	if(i == MAX_NUM_NI_USB_INTERFACES)
	{
		printk("unable to find interface in ni_usb_driver_interfaces[]? bug?\n");
	}
	usb_put_dev(interface_to_usbdev(interface));
//	printk("%s: exit\n", __FUNCTION__);
	mutex_unlock(&ni_usb_hotplug_lock);
}

static struct usb_driver ni_usb_bus_driver =
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
	.owner = THIS_MODULE,
#endif
	.name = "ni_usb_gpib",
	.probe = ni_usb_driver_probe,
	.disconnect = ni_usb_driver_disconnect,
	.id_table = ni_usb_driver_device_table,
};

static int __init ni_usb_init_module(void)
{
	int i;

	pr_info("ni_usb_gpib driver loading\n");
	for(i = 0; i < MAX_NUM_NI_USB_INTERFACES; i++)
		ni_usb_driver_interfaces[i] = NULL;
	usb_register(&ni_usb_bus_driver);
	gpib_register_driver(&ni_usb_gpib_interface, THIS_MODULE);

	return 0;
}

static void __exit ni_usb_exit_module(void)
{
	pr_info("ni_usb_gpib driver unloading\n");
//	printk("%s: enter\n", __FUNCTION__);
	gpib_unregister_driver(&ni_usb_gpib_interface);
	usb_deregister(&ni_usb_bus_driver);
//	printk("%s: exit\n", __FUNCTION__);
}

module_init(ni_usb_init_module);
module_exit(ni_usb_exit_module);

