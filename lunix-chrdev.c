/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * < Your name here >
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;
//global variable for chrdev state
struct lunix_chrdev_state_struct *lunix_chrdev_state;
/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	
	WARN_ON ( !(sensor = state->sensor));
	/* ? */

	//check timestamps
	if(state->buf_timestamp < sensor->msr_data[state->type]->last_update){
		//then it needs update
		debug("Update is needed\n");
		return 1;
	}
	//else it doen't need to update



	/* The following return is bogus, just for the stub to compile */
	debug("Data is up to date\n");
	return 0; /* ? */
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	uint32_t msr_data; 
	uint32_t timestamp;
	int integer_part;
	int float_part;
	long new_data;
	int ret = 1;
	int refresh;

        sensor = state->sensor;

	debug("Entering Update\n");


	/*
	 * Any new data available?
	 */

	//spin_lock(&sensor->lock);
	//refresh = lunix_chrdev_state_needs_refresh(state);
	//spin_unlock(state->lock);
	//if(refresh)


	if(lunix_chrdev_state_needs_refresh(state)){

		/*
		 * Grab the raw data quickly, hold the
		 * spinlock for as little as possible.
		 */

		/* Why use spinlocks? See LDD3, p. 119 */
		spin_lock(&sensor->lock);

		msr_data = *(sensor->msr_data[state->type]->values);
		timestamp = sensor->msr_data[state->type]->last_update;
		
		spin_unlock(&sensor->lock);
		
		/*
		 * Now we can take our time to format them,
		 * holding only the private state semaphore
		 */

//		if(ret = down_interruptible(&state->lock) < 0)
//		{
//			debug("Failed to lock state->lock\n");
//			goto out;
//		}else{
//			debug("Lock Aquired\n");
//		}

		if(state->type == TEMP){
			new_data = lookup_temperature[msr_data];
			debug("Temperature data = %ld\n", new_data);
		}else if(state->type == BATT){
			new_data = lookup_voltage[msr_data];
			debug("Battery data = %ld\n", new_data);
		}else if(state->type == LIGHT){
			new_data = lookup_light[msr_data];
			debug("Light data = %ld\n", new_data);
		}else{
			debug("Wrong sensor type\n");
			ret = 1;
			goto out;
		}

		integer_part = new_data / 1000;
		float_part = new_data % 1000;

		//keep the possitive part
		if(float_part < 0)
			float_part = -float_part;

		state->buf_lim = sprintf(state->buf_data,"%d.%d\n", integer_part, float_part);
		state->buf_timestamp = timestamp;
		
	
//		up(&state->lock); 
//		debug("Lock released\n");
		
		debug("Data just updated\n");
		return 0;
	}
	
	debug("Data are old\n");
	debug("Leaving Update\n");
	return 1;

out:
	debug("Failed to Update with ret: %d\n", ret);
	return ret;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	/* ? */
	int ret;

	debug("entering\n");
	ret = -ENODEV;

	if ((ret = nonseekable_open(inode, filp)) < 0)
	{
		debug("Failed to open\n");
		goto out;
	}

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */

	unsigned int minor = iminor(inode);

	
	/* Allocate a new Lunix character device private state structure */
	
	//allocate the struct
	lunix_chrdev_state = kmalloc(sizeof(struct lunix_chrdev_state_struct),GFP_KERNEL);
	if(!lunix_chrdev_state)
	{
		debug("Failed to allocate memmory for temp senso struct\n");
		goto out;
	}

	//initialize the semaphore
	sema_init(&lunix_chrdev_state->lock, 1);

	//lock the semaphore
	if( ret = down_interruptible(&lunix_chrdev_state->lock) < 0)
	{
		debug("Failed to allocate lock the lunix_chrdev_state->lock\n");
		goto out;
	}
	
	//assosiate the type with the minor
	lunix_chrdev_state->type = minor % 8;

	//save the sensor which is initiallized in the global variable
	lunix_chrdev_state->sensor = &lunix_sensors[minor / 8];

	//timestamp of opening
	lunix_chrdev_state->buf_timestamp = get_seconds();	
	
	//assign the private state
	filp->private_data = lunix_chrdev_state;

	//unlock the semaphore
	up(&lunix_chrdev_state->lock);
	
	//assosiate the f_ops
	//filp->f_ops = &lunix_chrdev_fops;
	
	debug("Open completed succesfully\n");
	return 0;	

out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{

	//free the kmalloc space
	kfree(filp->private_data);

	debug("kfree private_data and close file succesfully\n");
	debug("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");

	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Why? */
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t count, loff_t *f_pos)
{
//{
//	/* Why? */
//	return -EINVAL;
//}


	ssize_t ret = -EINVAL;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	debug("Entering read..\n");

	/* Lock? */

	if(ret = down_interruptible(&state->lock) < 0)
	{
		debug("Failed to lock state->lock\n");
		goto out;
	}else{
		debug("Lock Aquired\n");
	}
	
	debug("\nf_pos: %lld\n", *f_pos);

	if(*f_pos == 0 || *f_pos >= state->buf_lim){

		debug("beginning of a file\n");

		debug("Entering update loop\n");
		
		while (lunix_chrdev_state_update(state)) {/* nothing to read */
			
			up(&state->lock); /* release the lock */
			
			if (filp->f_flags & O_NONBLOCK)
			{
				ret = -EAGAIN;
				goto out;
			}
		
			//wait until knew data arrive	
			if (wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state)))
			{
				ret = -ERESTARTSYS; /* signal: tell the fs layer to handle it */
				goto out;
			}
			
			/* otherwise loop, but first reacquire the lock */
			if (down_interruptible(&state->lock))
			{
				ret = -ERESTARTSYS; /* signal: tell the fs layer to handle it */
				goto out;
			}
		}
	
	
		debug("update all the drivers\n");
	}else{
		debug("continue to previous state\n");
	}

	/* ok, data is there, return something */

	/* Determine the number of cached bytes to copy to userspace */
	

	//read other file every time will cause a different, f_pos so:
	
	//out or end of file?
	if(*f_pos >= state->buf_lim)
	{
		count = min(count, (size_t)state->buf_lim);
		*f_pos = 0;
	}else{
		//in file but i want to read less than the whole buffer
		count = min(count, (size_t)(state->buf_lim - *f_pos));
	}


	if (copy_to_user(usrbuf, state->buf_data + *f_pos, count)) {
		debug("Failed to copy_to_user\n");
		ret = -EFAULT;
		goto out;
	}

	//next f_pos where next read will start of
	if(*f_pos + count < state->buf_lim)
	{
		//then we have to print the rest of the number
		*f_pos = *f_pos + count; 
	}else{
		//place *f_pos to 0 so we can read the next number from the beginning
		*f_pos = 0;
	}

	debug("we got : %s\n",state->buf_data);
	up(&state->lock);
	debug("Unlock the lock");


	debug("Read succesfully\n");
	return count;


	/* Auto-rewind on EOF mode? */
	/* ? */
out:
	/* Unlock? */
	up (&state->lock);
	debug("Failed to read with ret: %ld\n", ret);
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = 
{
        .owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	// dev_no  saves the major number
	dev_t dev_no;
	//number of sensors * 8 is equal to the contiguous block
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
	
	debug("initializing character device\n");

	//initialize the structure that you have already allocated with p.56
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;

	//convert a major and minor number into a dev_t type variable	
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	/* ? */
	/* ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunix_chardev");*/
	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunix_chardev");
	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}	
	/* ? */
	/* cdev_add? */
	ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
		
	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
