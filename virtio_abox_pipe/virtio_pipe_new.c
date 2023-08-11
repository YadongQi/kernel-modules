#include <linux/module.h>
#include <linux/virtio.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_config.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/version.h>

//#define SHOW_LOG

#define VIRTIO_ID_PIPE_NEW     32 /* virtio pipe new */

#define RENDER_CONTROL_BUFFER_SIZE 0x1000
#define RENDER_READ_BUFFER_SIZE 0x10000
#define RENDER_WRITE_BUFFER_SIZE 0x200000
#define COMMON_CONTROL_BUFFER_SIZE 0x1000
#define COMMON_READ_BUFFER_SIZE 0x10000
#define COMMON_WRITE_BUFFER_SIZE 0x10000

#define CMD_OPEN_PIPE 		0
#define CMD_CLOSE_PIPE 		1
#define CMD_ALIVE_PIPE 		2
#define CMD_WRITE_PIPE 		3
#define CMD_READ_PIPE 		4

#define VPIPE_SUCCESS 1
#define VPIPE_FAIL 0

#define OPERATION_CONTROL 	0
#define OPERATION_READ		1
#define OPERATION_WRITE		2

struct virtpipe_channel_queue {
	size_t buffer_size;
	struct virtqueue* queue;
	spinlock_t lock;
};

struct virtpipe_channel {
	const char* name;
	bool busy_wait;
	const struct file_operations* op;

	struct cdev cdev;
	dev_t chrdev_region;
	struct class* class;
	struct device* device;

	struct virtpipe_channel_queue queue[3];
};

struct virtpipe_queue {
	char* buffer;
	size_t buffer_size;
	atomic_t buffer_using;
	struct completion buffer_ready;
	struct completion event;
};

struct virtpipe {
	unsigned long long windows_pipe;
	char name[128];

	struct virtpipe_queue queue[3];
	struct virtpipe_channel* channel;
	unsigned int len;
};

struct virtpipe_dev {
	struct virtio_device *vdev;

	// for render pipe
	struct virtpipe_channel render_channel;

	// for common pipe
	struct virtpipe_channel common_channel;

	struct timer_list alive_timer;
};

struct virtpipe_open_data {
	unsigned int busy_wait;
	unsigned int reserved1;
	unsigned int reserved2;
	unsigned int reserved3;
	char name[32];
};

struct virtpipe_packet {
	unsigned int size; // packet size
	unsigned int operation; // operation
	unsigned int result; // VPIPE_SUCCESS or VPIPE_FAIL
	unsigned int len; // return data len
	unsigned long long windows_pipe; // windows pipe handle
	unsigned long long linux_pipe; // linux pipe handle
	unsigned char data[0]; // data
};

static struct virtpipe_dev *g_pipe_dev = NULL;
static struct virtpipe* g_alive_pipe = NULL;
#ifdef SHOW_LOG
static const char* operation_name[3] = {"control", "read", "write"};
#endif

static int virtpipe_send_recv(struct virtpipe* pipe, struct virtpipe_packet* packet, int operation) {
	int err;
	struct scatterlist sg[1];
	struct scatterlist *sgs[1] = { &sg[0] };
	unsigned int len;
	unsigned long flags = 0;
	spinlock_t* queue_lock = &pipe->channel->queue[operation].lock;
	struct virtqueue* queue = pipe->channel->queue[operation].queue;
	struct completion* event = &pipe->queue[operation].event;
	unsigned int count;

	sg_init_one(&sg[0], packet, packet->size);

	if (pipe->channel->busy_wait) {
		spin_lock_irqsave(queue_lock, flags);
		if (virtqueue_add_sgs(queue, sgs, 1, 0, packet, GFP_ATOMIC) < 0) {
			printk(KERN_DEBUG "%s: virtqueue_add_sgs fail!!!\n", __FUNCTION__);
			spin_unlock_irqrestore(queue_lock, flags);
			err = -EIO;
			goto fail;
		}
		virtqueue_kick(queue);
		count = 0;
		while (!virtqueue_get_buf(queue, &len)) {
			count++;
			if (count >= 1000000) {
				printk(KERN_DEBUG "%s: virtqueue_get_buf fail!!!\n", __FUNCTION__);
				spin_unlock_irqrestore(queue_lock, flags);
				err = -EIO;
				goto fail;
			}
			cpu_relax();
		}
		spin_unlock_irqrestore(queue_lock, flags);
		if (len < sizeof(struct virtpipe_packet)) {
			printk(KERN_DEBUG "%s: len=%d, pipe closed\n", __FUNCTION__, len);
			err = -ENOENT;
			goto fail;
		} else if (len < packet->size) {
			printk(KERN_DEBUG "%s: len=%d, packet->size=%d, pipe closed\n",
				__FUNCTION__, len, packet->size);
			err = -ENOENT;
			goto fail;
		}
	} else {
		spin_lock_irqsave(queue_lock, flags);
#ifdef SHOW_LOG
		printk(KERN_DEBUG "%s: op=%s, pipe=%s, len=%d begin\n",
			__FUNCTION__, operation_name[operation], pipe->name, packet->size);
#endif
		if (virtqueue_add_sgs(queue, sgs, 1, 0, packet, GFP_ATOMIC) < 0) {
			printk(KERN_DEBUG "%s: virtqueue_add_sgs fail!!!\n", __FUNCTION__);
			spin_unlock_irqrestore(queue_lock, flags);
			err = -EIO;
			goto fail;
		}
		virtqueue_kick(queue);
		spin_unlock_irqrestore(queue_lock, flags);
		// wait for result
		wait_for_completion_interruptible(event);
#ifdef SHOW_LOG
		printk(KERN_DEBUG "%s: op=%s, pipe=%s, len=%d end\n",
			__FUNCTION__, operation_name[operation], pipe->name, pipe->len);
#endif
	}
	err = 0;

fail:
	return err;
}

static struct virtpipe_packet* virtpipe_lock_buffer(struct virtpipe* pipe, int operation) {
	while (atomic_cmpxchg(&pipe->queue[operation].buffer_using, 0, 1) != 0) {
		wait_for_completion_interruptible(&pipe->queue[operation].buffer_ready);
	}
	return (struct virtpipe_packet*)pipe->queue[operation].buffer;
}

static void virtpipe_unlock_buffer(struct virtpipe* pipe, int operation) {
	while (atomic_cmpxchg(&pipe->queue[operation].buffer_using, 1, 0) != 1) {
		cpu_relax();
	}
	complete(&pipe->queue[operation].buffer_ready);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
static void virtpipe_timer_func(unsigned long channel) {
#else
static void virtpipe_timer_func(struct timer_list *channel) {
#endif
	int err;
	struct virtpipe* pipe = g_alive_pipe;
	struct virtpipe_packet* packet = NULL;

	packet = virtpipe_lock_buffer(pipe, OPERATION_CONTROL);

	packet->size = sizeof(struct virtpipe_packet) + 4;
	packet->operation = CMD_ALIVE_PIPE;
	packet->windows_pipe = 0; // windows pipe handle
	packet->linux_pipe = (uintptr_t)pipe; // linux pipe handle
	packet->len = 4;
	packet->result = VPIPE_FAIL;
	*(unsigned int*)&packet->data[0] = 0xDD;

	err = virtpipe_send_recv(pipe, packet, OPERATION_CONTROL);
	if (err < 0) {
		printk(KERN_DEBUG "%s: virtpipe_send_recv fail err=%d\n", __FUNCTION__, err);
		goto fail_alive;
	}

	if (packet->result != VPIPE_SUCCESS) {
		printk(KERN_DEBUG "%s: alive fail\n", __FUNCTION__);
		err = -EIO;
		goto fail_alive;
	}

	if (!packet->len) {
		printk(KERN_DEBUG "%s: alive len = 0\n", __FUNCTION__);
		err = -EIO;
		goto fail_alive;
	}

	if (*(unsigned int*)&packet->data[0] == 0) {
		printk(KERN_DEBUG "%s: alive return 0\n", __FUNCTION__);
		err = -EIO;
		goto fail_alive;
	}

fail_alive:
	virtpipe_unlock_buffer(pipe, OPERATION_CONTROL);
	mod_timer(&g_pipe_dev->alive_timer, jiffies + msecs_to_jiffies(5000));
}

static ssize_t virtpipe_fops_read(struct file *filp, char __user *ubuf, size_t count, loff_t *offp) {
	int err;
	struct virtpipe* pipe = filp->private_data;
	struct virtpipe_packet* packet;
	unsigned int offset;
	unsigned int len;

	if (!pipe) {
		printk(KERN_DEBUG "%s: filp->private_data==NULL\n", __FUNCTION__);
		return -EINVAL;
	}

	if (!pipe->windows_pipe) {
		printk(KERN_DEBUG "%s: pipe->windows_pipe==NULL\n", __FUNCTION__);
		return -EINVAL;
	}

	packet = virtpipe_lock_buffer(pipe, OPERATION_READ);

#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: pipe=%s, count=0x%x\n", __FUNCTION__, pipe->name, count);
#endif

	offset = 0;
	while (offset < count) {
		len = count - offset;
		if (len > pipe->queue[OPERATION_READ].buffer_size - sizeof(struct virtpipe_packet)) {
			len = pipe->queue[OPERATION_READ].buffer_size - sizeof(struct virtpipe_packet);
		}

		packet->size = sizeof(struct virtpipe_packet) + len;
		packet->operation = CMD_READ_PIPE;
		packet->windows_pipe = pipe->windows_pipe; // windows pipe handle
		packet->linux_pipe = (uintptr_t)pipe; // linux pipe handle
		packet->len = 0;
		packet->result = VPIPE_FAIL;

		err = virtpipe_send_recv(pipe, packet, OPERATION_READ);
		if (err < 0) {
			printk(KERN_DEBUG "%s: virtpipe_send_recv pipe %s fail err=%d\n", __FUNCTION__, pipe->name, err);
			goto fail_read;
		}

		if (packet->result != VPIPE_SUCCESS) {
			printk(KERN_DEBUG "%s: read pipe %s fail\n", __FUNCTION__, pipe->name);
			err = -EIO;
			goto fail_read;
		}

		if (!packet->len) {
			printk(KERN_DEBUG "%s: read pipe %s fail size = 0\n", __FUNCTION__, pipe->name);
			err = -EIO;
			goto fail_read;
		}

		if (copy_to_user(ubuf + offset, packet->data, packet->len)) {
			printk(KERN_DEBUG "%s: copy_to_user fail ubuf=%p, packet->data=%p, packet->len=0x%x\n",
				__FUNCTION__, ubuf, packet->data, packet->len);
			err = -EFAULT;
			goto fail_read;
		}

		offset += packet->len;
		break;
	}
	err = offset;

#ifdef SHOW_LOG
		printk(KERN_DEBUG "%s: pipe=%s, ret=0x%x\n", __FUNCTION__, pipe->name, err);
#endif

fail_read:
	virtpipe_unlock_buffer(pipe, OPERATION_READ);
#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: leave err=%d\n", __FUNCTION__, err);
#endif
	return err;
}

static long virtpipe_open(struct file *filp, const char __user *ubuf, size_t count, loff_t *offp) {
	int err;
	struct virtpipe* pipe = filp->private_data;
	struct virtpipe_packet* packet = NULL;
	struct virtpipe_open_data* open_data = NULL;

	if (!pipe) {
		printk(KERN_DEBUG "%s: filp->private_data==NULL\n", __FUNCTION__);
		return -EINVAL;
	}

	if (pipe->windows_pipe) {
		// already opened
		printk(KERN_DEBUG "%s: pipe->windows_pipe!=0\n", __FUNCTION__);
		return -EINVAL;
	}

	packet = virtpipe_lock_buffer(pipe, OPERATION_CONTROL);

	if (copy_from_user(pipe->name, ubuf, count)) {
		printk(KERN_DEBUG "%s: pipe=%s, copy_from_user fail ubuf=%p, count=%d\n", __FUNCTION__, pipe->name, ubuf, count);
		err = -EFAULT;
		goto fail_ioctl;
	}
	pipe->name[count] = 0;

#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: pipe=%s, count=0x%x\n", __FUNCTION__, pipe->name, count);
#endif

	packet->len = sizeof(struct virtpipe_open_data);
	packet->size = sizeof(struct virtpipe_packet) + packet->len;
	packet->operation = CMD_OPEN_PIPE;
	packet->windows_pipe = 0; // windows pipe handle
	packet->linux_pipe = (uintptr_t)pipe; // linux pipe handle
	packet->result = VPIPE_FAIL;

	open_data = (struct virtpipe_open_data*)packet->data;
	open_data->busy_wait = pipe->channel->busy_wait;
	strcpy(open_data->name, pipe->name);

	err = virtpipe_send_recv(pipe, packet, OPERATION_CONTROL);
	if (err < 0) {
		printk(KERN_DEBUG "%s: pipe=%s, virtpipe_send_recv fail err=%d\n", __FUNCTION__, pipe->name, err);
		goto fail_ioctl;
	}

	if (packet->result != VPIPE_SUCCESS) {
		printk(KERN_DEBUG "%s: pipe=%s, CMD_OPEN_PIPE fail\n", __FUNCTION__, pipe->name);
		err = -EIO;
		goto fail_ioctl;
	}

	// save the windows pipe handle
	pipe->windows_pipe = packet->windows_pipe;

#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: pipe=%s, pipe=%lld\n", __FUNCTION__, pipe->name, pipe->windows_pipe);
#endif

	err = count;

#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: pipe=%s, ret=0x%x\n", __FUNCTION__, pipe->name, err);
#endif

fail_ioctl:
	virtpipe_unlock_buffer(pipe, OPERATION_CONTROL);
#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: leave err=%d\n", __FUNCTION__, err);
#endif
	return err;
}

static ssize_t virtpipe_fops_write(struct file *filp, const char __user *ubuf, size_t count, loff_t *offp) {
	int err;
	struct virtpipe* pipe = filp->private_data;
	struct virtpipe_packet* packet;
	unsigned int offset;
	unsigned int len;

	if (!pipe) {
		printk(KERN_DEBUG "%s: filp->private_data==NULL\n", __FUNCTION__);
		return -EINVAL;
	}

	if (!pipe->windows_pipe) {
		// set pipe name
		return virtpipe_open(filp, ubuf, count, offp);
	}

	packet = virtpipe_lock_buffer(pipe, OPERATION_WRITE);

#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: pipe=%s, count=0x%x\n", __FUNCTION__, pipe->name, count);
#endif

	offset = 0;
	while (offset < count) {
		len = count - offset;
		if (len > pipe->queue[OPERATION_WRITE].buffer_size - sizeof(struct virtpipe_packet)) {
			len = pipe->queue[OPERATION_WRITE].buffer_size - sizeof(struct virtpipe_packet);
		}

		if (copy_from_user(packet->data, ubuf + offset, len)) {
			printk(KERN_DEBUG "%s: copy_from_user fail packet->data=%p, ubuf=%p, offset=0x%x, len=0x%x\n",
				__FUNCTION__, packet->data, ubuf, offset, len);
			err = -EFAULT;
			goto fail_write;
		}

		packet->size = sizeof(struct virtpipe_packet) + len;
		packet->operation = CMD_WRITE_PIPE;
		packet->windows_pipe = pipe->windows_pipe; // windows pipe handle
		packet->linux_pipe = (uintptr_t)pipe; // linux pipe handle
		packet->len = len;
		packet->result = VPIPE_FAIL;

		err = virtpipe_send_recv(pipe, packet, OPERATION_WRITE);
		if (err < 0) {
			printk(KERN_DEBUG "%s: virtpipe_send_recv pipe %s fail err=%d\n", __FUNCTION__, pipe->name, err);
			goto fail_write;
		}

		if (packet->result != VPIPE_SUCCESS) {
			printk(KERN_DEBUG "%s: write pipe %s fail\n", __FUNCTION__, pipe->name);
			err = -EIO;
			goto fail_write;
		}

		if (!packet->len) {
			printk(KERN_DEBUG "%s: write pipe %s fail size = 0\n", __FUNCTION__, pipe->name);
			err = -EIO;
			goto fail_write;
		}

		offset += packet->len;
	}
	err = offset;

#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: pipe=%s, ret=0x%x\n", __FUNCTION__, pipe->name, err);
#endif

fail_write:
	virtpipe_unlock_buffer(pipe, OPERATION_WRITE);
#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: leave err=%d, offset=0x%x\n", __FUNCTION__, err, offset);
#endif
	return err;
}

static char* virtpipe_alloc_buffer(size_t* buffer_size) {
	char* buffer = NULL;
	while (!buffer) {
#ifdef SHOW_LOG
		printk(KERN_DEBUG "%s: try alloc 0x%x bytes for process %s\n",
			__FUNCTION__, *buffer_size, current->comm);
#endif
		buffer = kmalloc(*buffer_size, GFP_KERNEL);
		if (buffer) {
			return buffer;
		}
		printk(KERN_DEBUG "%s: kmalloc buffer 0x%x fail\n", __FUNCTION__, *buffer_size);
		if (*buffer_size == 0x1000) {
			return NULL;
		}
		*buffer_size = *buffer_size/2;
		if (*buffer_size < 0x1000) {
			*buffer_size = 0x1000;
		}
	}
	return NULL;
}

static struct virtpipe* virtpipe_alloc(struct virtpipe_channel* channel) {
	int i;
	int err = 0;
	struct virtpipe* pipe = kzalloc(sizeof(struct virtpipe), GFP_KERNEL);
	if (!pipe) {
		printk(KERN_DEBUG "%s: kzalloc virtpipe fail\n", __FUNCTION__);
		err = -ENOMEM;
		goto fail;
	}

	pipe->channel = channel;
	pipe->windows_pipe = 0;
	pipe->len = 0;

	for (i=0;i<3;i++) {
		pipe->queue[i].buffer_size = channel->queue[i].buffer_size;
		pipe->queue[i].buffer = virtpipe_alloc_buffer(&pipe->queue[i].buffer_size);
		atomic_set(&pipe->queue[i].buffer_using, 0);
		init_completion(&pipe->queue[i].buffer_ready);
		init_completion(&pipe->queue[i].event);
	}

	return pipe;
fail:
	if (pipe) {
		for (i=0;i<3;i++) {
			if (pipe->queue[i].buffer) {
				kfree(pipe->queue[i].buffer);
				pipe->queue[i].buffer = NULL;
			}
		}
		kfree(pipe);
		pipe = NULL;
	}
	return NULL;
}

static void virtpipe_free(struct virtpipe* pipe) {
	int i;
	if (pipe) {
		for (i=0;i<3;i++) {
			if (pipe->queue[i].buffer) {
				kfree(pipe->queue[i].buffer);
				pipe->queue[i].buffer = NULL;
			}
		}
		kfree(pipe);
		pipe = NULL;
	}
}

static int virtpipe_fops_open(struct inode *inode, struct file *filp, struct virtpipe_channel* channel) {
	int err;
	struct virtpipe* pipe = NULL;

#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: enter\n", __FUNCTION__);
#endif

	// alloc virtpipe
	pipe = virtpipe_alloc(channel);
	if (!pipe) {
		err = -ENOMEM;
		goto fail;
	}

	// save pipe
	filp->private_data = pipe;
	err = 0;

fail:
#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: leave err=%d\n", __FUNCTION__, err);
#endif
	return 0;
}

static int virtpipe_fops_open_render(struct inode *inode, struct file *filp) {
	return virtpipe_fops_open(inode, filp, &g_pipe_dev->render_channel);
}

static int virtpipe_fops_open_common(struct inode *inode, struct file *filp) {
	return virtpipe_fops_open(inode, filp, &g_pipe_dev->common_channel);
}

static int virtpipe_fops_release(struct inode *inode, struct file *filp) {
	int err;
	struct virtpipe* pipe = filp->private_data;
	struct virtpipe_packet* packet = NULL;

	if (!pipe) {
		printk(KERN_DEBUG "%s: filp->private_data==NULL\n", __FUNCTION__);
		return 0;
	}

	if (pipe->windows_pipe) {
#ifdef SHOW_LOG
		printk(KERN_DEBUG "%s: pipe=%lld, name=%s\n", __FUNCTION__, pipe->windows_pipe, pipe->name);
#endif

		packet = virtpipe_lock_buffer(pipe, OPERATION_CONTROL);

		packet->size = sizeof(struct virtpipe_packet);
		packet->operation = CMD_CLOSE_PIPE;
		packet->windows_pipe = pipe->windows_pipe; // windows pipe handle
		packet->linux_pipe = (uintptr_t)pipe; // linux pipe handle
		packet->len = 0;
		packet->result = VPIPE_FAIL;

		err = virtpipe_send_recv(pipe, packet, OPERATION_CONTROL);
		if (err < 0) {
			printk(KERN_DEBUG "%s: virtpipe_send_recv fail err=%d\n", __FUNCTION__, err);
			goto fail;
		}

		if (packet->result != VPIPE_SUCCESS) {
			printk(KERN_DEBUG "%s: CMD_CLOSE_PIPE fail\n", __FUNCTION__);
			err = -EIO;
			goto fail;
		}

fail:
		virtpipe_unlock_buffer(pipe, OPERATION_CONTROL);
	}

	virtpipe_free(pipe);
	pipe = NULL;
	err = 0;

#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: leave err=%d\n", __FUNCTION__, err);
#endif
	return err;
}

static void virtpipe_queue_callback(struct virtqueue *vq)
{
	int err;
	int len;
	struct virtpipe_packet* packet;
	struct virtpipe* pipe;
	int operation;

#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: virtqueue_get_buf for queue %s\n", __FUNCTION__, vq->name);
#endif

	packet = virtqueue_get_buf(vq, &len);
	if (!packet) {
		printk(KERN_DEBUG "%s: virtqueue_get_buf fail for queue %s\n", __FUNCTION__, vq->name);
		err = -EIO;
		goto fail;
	}

	err = 0;
	if (len < sizeof(struct virtpipe_packet)) {
		printk(KERN_DEBUG "%s: len=%d, pipe closed\n", __FUNCTION__, len);
		err = -ENOENT;
		goto fail;
	} else if (len < packet->size) {
		printk(KERN_DEBUG "%s: len=%d, packet->size=%d, pipe closed\n",
			__FUNCTION__, len, packet->size);
		err = -ENOENT;
		goto fail;
	}

	pipe = (struct virtpipe*)(uintptr_t)packet->linux_pipe;
	if (!pipe) {
		err = -ENOENT;
		goto fail;
	}

	pipe->len = len;
	for (operation=0;operation<3;operation++) {
		if ((char*)packet == pipe->queue[operation].buffer) {
			break;
		}
	}
#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: op=%s, pipe=%s, len=%d\n", __FUNCTION__, operation_name[operation], pipe->name, pipe->len);
#endif
	complete(&pipe->queue[operation].event);

fail:
#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: leave\n", __FUNCTION__);
#endif
	return;
}

static const struct file_operations virtpipe_fops_render = {
	.owner = THIS_MODULE,
	.read  = virtpipe_fops_read,
	.write = virtpipe_fops_write,
	.open = virtpipe_fops_open_render,
	.release = virtpipe_fops_release,
};

static const struct file_operations virtpipe_fops_common = {
	.owner = THIS_MODULE,
	.read  = virtpipe_fops_read,
	.write = virtpipe_fops_write,
	.open = virtpipe_fops_open_common,
	.release = virtpipe_fops_release,
};

static int virtpipe_init_channel(struct virtio_device *vdev, struct virtpipe_channel* channel)
{
	int i;
	int err;

#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: enter\n", __FUNCTION__);
#endif

	for (i=0;i<3;i++) {
		spin_lock_init(&channel->queue[i].lock);
	}

	if (!channel->op) {
		// no char device
		return 0;
	}

	// create char device
	err = alloc_chrdev_region(&channel->chrdev_region, 0, 1, channel->name);
	if (err < 0) {
		printk(KERN_DEBUG "%s: alloc_chrdev_region fail err=%d\n", __FUNCTION__, err);
		goto fail_alloc_chrdev_region;
	}

	cdev_init(&channel->cdev, channel->op);

	err = cdev_add(&channel->cdev, channel->chrdev_region, 1);
	if (err < 0) {
		printk(KERN_DEBUG "%s: cdev_add fail err=%d\n", __FUNCTION__, err);
		goto fail_cdev_add;
	}

	channel->class = class_create(THIS_MODULE, channel->name);
	if (IS_ERR(channel->class)) {
		err = PTR_ERR(channel->class);
		printk(KERN_DEBUG "%s: class_create fail err=%d\n", __FUNCTION__, err);
		goto fail_class_create;
	}

	channel->device = device_create(channel->class, &vdev->dev, channel->chrdev_region, g_pipe_dev, channel->name);
	if (IS_ERR(channel->device)) {
		err = PTR_ERR(channel->device);
		printk(KERN_DEBUG "%s: device_create fail err=%d\n", __FUNCTION__, err);
		goto fail_device_create;
	}

	return 0;

fail_device_create:
	class_destroy(channel->class);
fail_class_create:
	cdev_del(&channel->cdev);
fail_cdev_add:
	unregister_chrdev_region(channel->chrdev_region, 1);
fail_alloc_chrdev_region:
#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: leave err=%d\n", __FUNCTION__, err);
#endif
	return err;
}

static void virtpipe_destroy_channel(struct virtpipe_channel* channel)
{
#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: enter\n", __FUNCTION__);
#endif
	device_destroy(channel->class, channel->chrdev_region);
	class_destroy(channel->class);
	cdev_del(&channel->cdev);
	unregister_chrdev_region(channel->chrdev_region, 1);
	device_destroy(channel->class, channel->chrdev_region);
#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: leave\n", __FUNCTION__);
#endif
}

static int virtpipe_probe(struct virtio_device *vdev)
{
	int i;
	int err;
	vq_callback_t *io_callbacks[6] = {0};
	const char *io_names[6] = {0};
	struct virtqueue *vqs[6] = {0};

	printk(KERN_DEBUG "%s: enter\n", __FUNCTION__);

	g_pipe_dev = (struct virtpipe_dev *)kzalloc(sizeof(*g_pipe_dev), GFP_KERNEL);
	if (!g_pipe_dev) {
		printk(KERN_DEBUG "%s: alloc g_pipe_dev fail\n", __FUNCTION__);
		err = -ENOMEM;
		goto fail_alloc_virtpipe_dev;
	}

	g_pipe_dev->vdev = vdev;
	vdev->priv = g_pipe_dev;

	// render pipe
	g_pipe_dev->render_channel.name = "virtpipe-render";
	g_pipe_dev->render_channel.busy_wait = true;
	g_pipe_dev->render_channel.queue[OPERATION_CONTROL].buffer_size = RENDER_CONTROL_BUFFER_SIZE;
	g_pipe_dev->render_channel.queue[OPERATION_READ].buffer_size = RENDER_READ_BUFFER_SIZE;
	g_pipe_dev->render_channel.queue[OPERATION_WRITE].buffer_size = RENDER_WRITE_BUFFER_SIZE;
	g_pipe_dev->render_channel.op = &virtpipe_fops_render;
	err = virtpipe_init_channel(vdev, &g_pipe_dev->render_channel);
	if (err < 0) {
		printk(KERN_DEBUG "%s: init render channel fail\n", __FUNCTION__);
		goto fail_init_render_channel;
	}

	// common pipe
	g_pipe_dev->common_channel.name = "virtpipe-common";
	g_pipe_dev->common_channel.busy_wait = false;
	g_pipe_dev->common_channel.queue[OPERATION_CONTROL].buffer_size = COMMON_CONTROL_BUFFER_SIZE;
	g_pipe_dev->common_channel.queue[OPERATION_READ].buffer_size = COMMON_READ_BUFFER_SIZE;
	g_pipe_dev->common_channel.queue[OPERATION_WRITE].buffer_size = COMMON_WRITE_BUFFER_SIZE;
	g_pipe_dev->common_channel.op = &virtpipe_fops_common;
	err = virtpipe_init_channel(vdev, &g_pipe_dev->common_channel);
	if (err < 0) {
		printk(KERN_DEBUG "%s: init common channel fail\n", __FUNCTION__);
		goto fail_init_common_channel;
	}

	// find vqs
	io_names[0] = "RENDER-CONTROL";
	io_names[1] = "RENDER-READ";
	io_names[2] = "RENDER-WRITE";
	io_names[3] = "COMMON-CONTROL";
	io_names[4] = "COMMON-READ";
	io_names[5] = "COMMON-WRITE";

	if (!g_pipe_dev->render_channel.busy_wait) {
		io_callbacks[0] = virtpipe_queue_callback;
		io_callbacks[1] = virtpipe_queue_callback;
		io_callbacks[2] = virtpipe_queue_callback;
	}
	if (!g_pipe_dev->common_channel.busy_wait) {
		io_callbacks[3] = virtpipe_queue_callback;
		io_callbacks[4] = virtpipe_queue_callback;
		io_callbacks[5] = virtpipe_queue_callback;
	}

	err = virtio_find_vqs(vdev, 6, vqs, io_callbacks, io_names, NULL);
	if (err) {
		printk(KERN_DEBUG "%s: find_vqs fail err=%d\n", __FUNCTION__, err);
		goto fail_find_vqs;
	}

	for (i=0;i<3;i++) {
		g_pipe_dev->render_channel.queue[i].queue = vqs[i];
	}

	for (i=0;i<3;i++) {
		g_pipe_dev->common_channel.queue[i].queue = vqs[3+i];
	}

	if (g_pipe_dev->render_channel.busy_wait) {
		for (i=0;i<3;i++) {
			virtqueue_disable_cb(g_pipe_dev->render_channel.queue[i].queue);
		}
	}
	if (g_pipe_dev->common_channel.busy_wait) {
		for (i=0;i<3;i++) {
			virtqueue_disable_cb(g_pipe_dev->common_channel.queue[i].queue);
		}
	}

	// render keep alive timer
	g_alive_pipe = virtpipe_alloc(&g_pipe_dev->render_channel);
	if (!g_alive_pipe) {
		printk(KERN_DEBUG "%s: virtpipe_alloc alive pipe fail\n", __FUNCTION__);
		goto fail_alloc_alive_pipe;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
	setup_timer(&g_pipe_dev->alive_timer, virtpipe_timer_func, 0);
#else
	timer_setup(&g_pipe_dev->alive_timer, virtpipe_timer_func, 0);
#endif
	mod_timer(&g_pipe_dev->alive_timer, jiffies + msecs_to_jiffies(5000));

	return 0;

fail_alloc_alive_pipe:
	for (i=0;i<3;i++) {
		g_pipe_dev->render_channel.queue[i].queue = NULL;
	}

	for (i=0;i<3;i++) {
		g_pipe_dev->common_channel.queue[i].queue = NULL;
	}
fail_find_vqs:
	virtpipe_destroy_channel(&g_pipe_dev->common_channel);
fail_init_common_channel:
	virtpipe_destroy_channel(&g_pipe_dev->render_channel);
fail_init_render_channel:
	kfree(g_pipe_dev);
	g_pipe_dev = NULL;
fail_alloc_virtpipe_dev:
#ifdef SHOW_LOG
	printk(KERN_DEBUG "%s: leave err=%d\n", __FUNCTION__, err);
#endif
	return err;
}

static void virtpipe_remove(struct virtio_device *vdev) {
	// for render pipe
	virtpipe_destroy_channel(&g_pipe_dev->render_channel);

	// for common pipe
	virtpipe_destroy_channel(&g_pipe_dev->common_channel);

	kfree(g_pipe_dev);
}

static struct virtio_device_id id_table[] = {
	{ VIRTIO_ID_PIPE_NEW, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

static struct virtio_driver virtio_pipe_driver = {
	.driver.name =	KBUILD_MODNAME,
	.driver.owner =	THIS_MODULE,
	.id_table =	id_table,
	.probe =	virtpipe_probe,
	.remove =	virtpipe_remove,
};

static int __init init(void)
{
	printk(KERN_DEBUG "virtio pipe new driver init id=%d\n", id_table[0].device);
	return register_virtio_driver(&virtio_pipe_driver);
}

static void __exit fini(void)
{
	printk(KERN_DEBUG "virtio pipe new driver uninit\n");
	unregister_virtio_driver(&virtio_pipe_driver);
}
module_init(init);
module_exit(fini);

MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_DESCRIPTION("Virtio pipe new driver");
MODULE_LICENSE("GPL");
