/*
 *      uvc_queue.c  --  USB Video Class driver - Buffers management
 *
 *      Copyright (C) 2005-2010
 *          Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 *
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *
 */

#include <linux/atomic.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <media/videobuf2-vmalloc.h>

#include "uvcvideo.h"

/* ------------------------------------------------------------------------
 * Video buffers queue management.
 *
 * Video queues is initialized by uvc_queue_init(). The function performs
 * basic initialization of the uvc_video_queue struct and never fails.
 *
 * Video buffers are managed by videobuf2. The driver uses a mutex to protect
 * the videobuf2 queue operations by serializing calls to videobuf2 and a
 * spinlock to protect the IRQ queue that holds the buffers to be processed by
 * the driver.
 */

/* -----------------------------------------------------------------------------
 * videobuf2 queue operations
 */

static int uvc_queue_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
			   unsigned int *nbuffers, unsigned int *nplanes,
			   unsigned int sizes[], void *alloc_ctxs[])
{
	struct uvc_video_queue *queue = vb2_get_drv_priv(vq);
	struct uvc_streaming *stream =
			container_of(queue, struct uvc_streaming, queue);

	if (*nbuffers > UVC_MAX_VIDEO_BUFFERS)
		*nbuffers = UVC_MAX_VIDEO_BUFFERS;

	*nplanes = 1;

	sizes[0] = stream->ctrl.dwMaxVideoFrameSize;

	return 0;
}

static int uvc_buffer_prepare(struct vb2_buffer *vb)
{
	struct uvc_video_queue *queue = vb2_get_drv_priv(vb->vb2_queue);
	struct uvc_buffer *buf = container_of(vb, struct uvc_buffer, buf);

	if (vb->v4l2_buf.type == V4L2_BUF_TYPE_VIDEO_OUTPUT &&
	    vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0)) {
		uvc_trace(UVC_TRACE_CAPTURE, "[E] Bytes used out of bounds.\n");
		return -EINVAL;
	}

	if (unlikely(queue->flags & UVC_QUEUE_DISCONNECTED))
		return -ENODEV;

	buf->state = UVC_BUF_STATE_QUEUED;
	buf->error = 0;
	buf->mem = vb2_plane_vaddr(vb, 0);
	buf->length = vb2_plane_size(vb, 0);
	if (vb->v4l2_buf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		buf->bytesused = 0;
	else
		buf->bytesused = vb2_get_plane_payload(vb, 0);

	return 0;
}

static void uvc_buffer_queue(struct vb2_buffer *vb)
{
	struct uvc_video_queue *queue = vb2_get_drv_priv(vb->vb2_queue);
	struct uvc_buffer *buf = container_of(vb, struct uvc_buffer, buf);
	unsigned long flags;

	spin_lock_irqsave(&queue->irqlock, flags);
	if (likely(!(queue->flags & UVC_QUEUE_DISCONNECTED))) {
		list_add_tail(&buf->queue, &queue->irqqueue);
	} else {
		/* If the device is disconnected return the buffer to userspace
		 * directly. The next QBUF call will fail with -ENODEV.
		 */
		buf->state = UVC_BUF_STATE_ERROR;
		vb2_buffer_done(&buf->buf, VB2_BUF_STATE_ERROR);
	}

	spin_unlock_irqrestore(&queue->irqlock, flags);
}

static int uvc_buffer_finish(struct vb2_buffer *vb)
{
	struct uvc_video_queue *queue = vb2_get_drv_priv(vb->vb2_queue);
	struct uvc_streaming *stream =
			container_of(queue, struct uvc_streaming, queue);
	struct uvc_buffer *buf = container_of(vb, struct uvc_buffer, buf);

	uvc_video_clock_update(stream, &vb->v4l2_buf, buf);
	return 0;
}

static struct vb2_ops uvc_queue_qops = {
	.queue_setup = uvc_queue_setup,
	.buf_prepare = uvc_buffer_prepare,
	.buf_queue = uvc_buffer_queue,
	.buf_finish = uvc_buffer_finish,
};

void uvc_queue_init(struct uvc_video_queue *queue, enum v4l2_buf_type type,
		    int drop_corrupted)
{
	queue->queue.type = type;
	queue->queue.io_modes = VB2_MMAP | VB2_USERPTR;
	queue->queue.drv_priv = queue;
	queue->queue.buf_struct_size = sizeof(struct uvc_buffer);
	queue->queue.ops = &uvc_queue_qops;
	queue->queue.mem_ops = &vb2_vmalloc_memops;
	vb2_queue_init(&queue->queue);

	mutex_init(&queue->mutex);
	spin_lock_init(&queue->irqlock);
	INIT_LIST_HEAD(&queue->irqqueue);
	queue->flags = drop_corrupted ? UVC_QUEUE_DROP_CORRUPTED : 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 queue operations
 */

int uvc_alloc_buffers(struct uvc_video_queue *queue,
		      struct v4l2_requestbuffers *rb)
{
	int ret;

	mutex_lock(&queue->mutex);
	ret = vb2_reqbufs(&queue->queue, rb);
	mutex_unlock(&queue->mutex);

	return ret ? ret : rb->count;
}

void uvc_free_buffers(struct uvc_video_queue *queue)
{
	mutex_lock(&queue->mutex);
	vb2_queue_release(&queue->queue);
	mutex_unlock(&queue->mutex);
}

int uvc_query_buffer(struct uvc_video_queue *queue, struct v4l2_buffer *buf)
{
	int ret;

	mutex_lock(&queue->mutex);
	ret = vb2_querybuf(&queue->queue, buf);
	mutex_unlock(&queue->mutex);

	return ret;
}

int uvc_queue_buffer(struct uvc_video_queue *queue, struct v4l2_buffer *buf)
{
	int ret;

	mutex_lock(&queue->mutex);
	ret = vb2_qbuf(&queue->queue, buf);
	mutex_unlock(&queue->mutex);

	return ret;
}

//ASUS_BSP+++ Patrick "[A68][USB Cam][NA][Others] Using dequeue with timeout"
#define call_memop(q, op, args...)					\
	(((q)->mem_ops->op) ?						\
		((q)->mem_ops->op(args)) : 0)

#define call_qop(q, op, args...)					\
	(((q)->ops->op) ? ((q)->ops->op(args)) : 0)

#define V4L2_BUFFER_STATE_FLAGS	(V4L2_BUF_FLAG_MAPPED | V4L2_BUF_FLAG_QUEUED | \
				 V4L2_BUF_FLAG_DONE | V4L2_BUF_FLAG_ERROR | \
				 V4L2_BUF_FLAG_PREPARED)

#define UVC_Q_WAITON_TIMEOUT 1000
#define UVC_DQ_TIMEOUT_COUNT 5

int Deque_Timeout_Count = 0;

static int uvc_from_vb2_verify_planes_array(struct vb2_buffer *vb, const struct v4l2_buffer *b)
{
	/* Is memory for copying plane information present? */
	if (NULL == b->m.planes) {
		printk("uvc_from_vb2_verify_planes_array: Multi-planar buffer passed but "
			   "planes array not provided\n");
		return -EINVAL;
	}

	if (b->length < vb->num_planes || b->length > VIDEO_MAX_PLANES) {
		printk("uvc_from_vb2_verify_planes_array: Incorrect planes array length, "
			   "expected %d, got %d\n", vb->num_planes, b->length);
		return -EINVAL;
	}

	return 0;
}

static bool uvc_from_vb2_buffer_in_use(struct vb2_queue *q, struct vb2_buffer *vb)
{
	unsigned int plane;
	for (plane = 0; plane < vb->num_planes; ++plane) {
		void *mem_priv = vb->planes[plane].mem_priv;
		/*
		 * If num_users() has not been provided, call_memop
		 * will return 0, apparently nobody cares about this
		 * case anyway. If num_users() returns more than 1,
		 * we are not the only user of the plane's memory.
		 */
		if (mem_priv && call_memop(q, num_users, mem_priv) > 1)
			return true;
	}
	return false;
}

static int uvc_from_vb2_fill_v4l2_buffer(struct vb2_buffer *vb, struct v4l2_buffer *b)
{
	struct vb2_queue *q = vb->vb2_queue;
	int ret;

	/* Copy back data such as timestamp, flags, input, etc. */
	memcpy(b, &vb->v4l2_buf, offsetof(struct v4l2_buffer, m));
	b->input = vb->v4l2_buf.input;
	b->reserved = vb->v4l2_buf.reserved;

	if (V4L2_TYPE_IS_MULTIPLANAR(q->type)) {
		ret = uvc_from_vb2_verify_planes_array(vb, b);
		if (ret)
			return ret;

		/*
		 * Fill in plane-related data if userspace provided an array
		 * for it. The memory and size is verified above.
		 */
		memcpy(b->m.planes, vb->v4l2_planes,
			b->length * sizeof(struct v4l2_plane));
	} else {
		/*
		 * We use length and offset in v4l2_planes array even for
		 * single-planar buffers, but userspace does not.
		 */
		b->length = vb->v4l2_planes[0].length;
		b->bytesused = vb->v4l2_planes[0].bytesused;
		if (q->memory == V4L2_MEMORY_MMAP)
			b->m.offset = vb->v4l2_planes[0].m.mem_offset;
		else if (q->memory == V4L2_MEMORY_USERPTR)
			b->m.userptr = vb->v4l2_planes[0].m.userptr;
	}

	/*
	 * Clear any buffer state related flags.
	 */
	b->flags &= ~V4L2_BUFFER_STATE_FLAGS;

	switch (vb->state) {
	case VB2_BUF_STATE_QUEUED:
	case VB2_BUF_STATE_ACTIVE:
		b->flags |= V4L2_BUF_FLAG_QUEUED;
		break;
	case VB2_BUF_STATE_ERROR:
		b->flags |= V4L2_BUF_FLAG_ERROR;
		/* fall through */
	case VB2_BUF_STATE_DONE:
		b->flags |= V4L2_BUF_FLAG_DONE;
		break;
	case VB2_BUF_STATE_PREPARED:
		b->flags |= V4L2_BUF_FLAG_PREPARED;
		break;
	case VB2_BUF_STATE_DEQUEUED:
		/* nothing */
		break;
	}

	if (uvc_from_vb2_buffer_in_use(q, vb))
		b->flags |= V4L2_BUF_FLAG_MAPPED;

	return 0;
}

static int uvc_from_vb2_wait_for_done_vb(struct vb2_queue *q, int nonblocking)
{
	/*
	 * All operations on vb_done_list are performed under done_lock
	 * spinlock protection. However, buffers may be removed from
	 * it and returned to userspace only while holding both driver's
	 * lock and the done_lock spinlock. Thus we can be sure that as
	 * long as we hold the driver's lock, the list will remain not
	 * empty if list_empty() check succeeds.
	 */

	for (;;) {
		int ret;

		if (!q->streaming) {
//			printk("uvc_from_vb2_wait_for_done_vb: Streaming off, will not wait for buffers\n");
			return -EINVAL;
		}

		if (!list_empty(&q->done_list)) {
			/*
			 * Found a buffer that we were waiting for.
			 */
			break;
		}

		if (nonblocking) {
//			printk("uvc_from_vb2_wait_for_done_vb: Nonblocking and no buffers to dequeue, "
//								"will not wait\n");
			return -EAGAIN;
		}

		/*
		 * We are streaming and blocking, wait for another buffer to
		 * become ready or for streamoff. Driver's lock is released to
		 * allow streamoff or qbuf to be called while waiting.
		 */
		call_qop(q, wait_prepare, q);

		/*
		 * All locks have been released, it is safe to sleep now.
		 */
		ret = wait_event_interruptible_timeout(q->done_wq,
				!list_empty(&q->done_list) || !q->streaming,
				UVC_Q_WAITON_TIMEOUT);

		/*
		 * We need to reevaluate both conditions again after reacquiring
		 * the locks or return an error if one occurred.
		 */
		call_qop(q, wait_finish, q);
		if (ret==0) {
//			printk("uvc_from_vb2_wait_for_done_vb: X, timeout\n");

			if(Deque_Timeout_Count > UVC_DQ_TIMEOUT_COUNT) {
//				printk("uvc_from_vb2_wait_for_done_vb: Deque_Timeout_Count=%d > UVC_DQ_TIMEOUT_COUNT\n", Deque_Timeout_Count);

				return -EBUSY;
			}

			Deque_Timeout_Count++;

			return -ETIME;
		} else if (ret<0) {
//			printk("uvc_from_vb2_wait_for_done_vb: X, ret<0 =%d---\n", ret);
			return ret;
		}
	}
	return 0;
}

static int uvc_from_vb2_get_done_vb(struct vb2_queue *q, struct vb2_buffer **vb,
				int nonblocking)
{
	unsigned long flags;
	int ret;

	/*
	 * Wait for at least one buffer to become available on the done_list.
	 */
	ret = uvc_from_vb2_wait_for_done_vb(q, nonblocking);

	if (ret)
		return ret;

	/*
	 * Driver's lock has been held since we last verified that done_list
	 * is not empty, so no need for another list_empty(done_list) check.
	 */
	spin_lock_irqsave(&q->done_lock, flags);
	*vb = list_first_entry(&q->done_list, struct vb2_buffer, done_entry);
	list_del(&(*vb)->done_entry);
	spin_unlock_irqrestore(&q->done_lock, flags);

	return 0;
}

int uvc_from_vb2_dqbuf(struct vb2_queue *q, struct v4l2_buffer *b, bool nonblocking)
{
	struct vb2_buffer *vb = NULL;
	int ret;

	if (q->fileio) {
		printk("uvc_from_vb2_dqbuf: file io in progress\n");
		return -EBUSY;
	}

	if (b->type != q->type) {
		printk("uvc_from_vb2_dqbuf: invalid buffer type\n");
		return -EINVAL;
	}

	ret = uvc_from_vb2_get_done_vb(q, &vb, nonblocking);
	if (ret < 0) {
		if(ret==-ETIME) {
//			printk("uvc_from_vb2_dqbuf: after vb2_wait_for_done_vb, ret=%d---\n", ret);
			return 0;
		} else {
//			printk("uvc_from_vb2_dqbuf: error getting next done buffer\n");
			return ret;
		}
	}

	ret = call_qop(q, buf_finish, vb);
	if (ret) {
		printk("uvc_from_vb2_dqbuf: buffer finish failed\n");
		return ret;
	}

	switch (vb->state) {
	case VB2_BUF_STATE_DONE:
//		printk("uvc_from_vb2_dqbuf: Returning done buffer\n");
		Deque_Timeout_Count = 0;
		break;
	case VB2_BUF_STATE_ERROR:
		printk("uvc_from_vb2_dqbuf: Returning done buffer with errors\n");
		break;
	default:
		printk("uvc_from_vb2_dqbuf: Invalid buffer state\n");
		return -EINVAL;
	}

	/* Fill buffer information for the userspace */
	uvc_from_vb2_fill_v4l2_buffer(vb, b);
	/* Remove from videobuf queue */
	list_del(&vb->queued_entry);

//	printk("uvc_from_vb2_dqbuf: dqbuf of buffer %d, with state %d\n",
//			vb->v4l2_buf.index, vb->state);

	vb->state = VB2_BUF_STATE_DEQUEUED;
	return 0;
}
//ASUS_BSP--- Patrick "[A68][Camera][NA][Others] Using dequeue with timeout"
int uvc_dequeue_buffer(struct uvc_video_queue *queue, struct v4l2_buffer *buf,
		       int nonblocking)
{
	int ret;

	mutex_lock(&queue->mutex);
	ret = uvc_from_vb2_dqbuf(&queue->queue, buf, nonblocking); //ASUS_BSP Patrick "[A68][Camera][NA][Others] Using dequeue with timeout"
	mutex_unlock(&queue->mutex);

	return ret;
}

int uvc_queue_mmap(struct uvc_video_queue *queue, struct vm_area_struct *vma)
{
	int ret;

	mutex_lock(&queue->mutex);
	ret = vb2_mmap(&queue->queue, vma);
	mutex_unlock(&queue->mutex);

	return ret;
}

unsigned int uvc_queue_poll(struct uvc_video_queue *queue, struct file *file,
			    poll_table *wait)
{
	unsigned int ret;

	mutex_lock(&queue->mutex);
	ret = vb2_poll(&queue->queue, file, wait);
	mutex_unlock(&queue->mutex);

	return ret;
}

/* -----------------------------------------------------------------------------
 *
 */

/*
 * Check if buffers have been allocated.
 */
int uvc_queue_allocated(struct uvc_video_queue *queue)
{
	int allocated;

	mutex_lock(&queue->mutex);
	allocated = vb2_is_busy(&queue->queue);
	mutex_unlock(&queue->mutex);

	return allocated;
}

#ifndef CONFIG_MMU
/*
 * Get unmapped area.
 *
 * NO-MMU arch need this function to make mmap() work correctly.
 */
unsigned long uvc_queue_get_unmapped_area(struct uvc_video_queue *queue,
		unsigned long pgoff)
{
	struct uvc_buffer *buffer;
	unsigned int i;
	unsigned long ret;

	mutex_lock(&queue->mutex);
	for (i = 0; i < queue->count; ++i) {
		buffer = &queue->buffer[i];
		if ((buffer->buf.m.offset >> PAGE_SHIFT) == pgoff)
			break;
	}
	if (i == queue->count) {
		ret = -EINVAL;
		goto done;
	}
	ret = (unsigned long)buf->mem;
done:
	mutex_unlock(&queue->mutex);
	return ret;
}
#endif

/*
 * Enable or disable the video buffers queue.
 *
 * The queue must be enabled before starting video acquisition and must be
 * disabled after stopping it. This ensures that the video buffers queue
 * state can be properly initialized before buffers are accessed from the
 * interrupt handler.
 *
 * Enabling the video queue returns -EBUSY if the queue is already enabled.
 *
 * Disabling the video queue cancels the queue and removes all buffers from
 * the main queue.
 *
 * This function can't be called from interrupt context. Use
 * uvc_queue_cancel() instead.
 */
int uvc_queue_enable(struct uvc_video_queue *queue, int enable)
{
	unsigned long flags;
	int ret;

	mutex_lock(&queue->mutex);
	if (enable) {
		ret = vb2_streamon(&queue->queue, queue->queue.type);
		if (ret < 0)
			goto done;

		queue->buf_used = 0;
	} else {
		ret = vb2_streamoff(&queue->queue, queue->queue.type);
		if (ret < 0)
			goto done;

		spin_lock_irqsave(&queue->irqlock, flags);
		INIT_LIST_HEAD(&queue->irqqueue);
		spin_unlock_irqrestore(&queue->irqlock, flags);
	}

done:
	mutex_unlock(&queue->mutex);
	return ret;
}

/*
 * Cancel the video buffers queue.
 *
 * Cancelling the queue marks all buffers on the irq queue as erroneous,
 * wakes them up and removes them from the queue.
 *
 * If the disconnect parameter is set, further calls to uvc_queue_buffer will
 * fail with -ENODEV.
 *
 * This function acquires the irq spinlock and can be called from interrupt
 * context.
 */
void uvc_queue_cancel(struct uvc_video_queue *queue, int disconnect)
{
	struct uvc_buffer *buf;
	unsigned long flags;

	spin_lock_irqsave(&queue->irqlock, flags);
	while (!list_empty(&queue->irqqueue)) {
		buf = list_first_entry(&queue->irqqueue, struct uvc_buffer,
				       queue);
		list_del(&buf->queue);
		buf->state = UVC_BUF_STATE_ERROR;
		vb2_buffer_done(&buf->buf, VB2_BUF_STATE_ERROR);
	}
	/* This must be protected by the irqlock spinlock to avoid race
	 * conditions between uvc_buffer_queue and the disconnection event that
	 * could result in an interruptible wait in uvc_dequeue_buffer. Do not
	 * blindly replace this logic by checking for the UVC_QUEUE_DISCONNECTED
	 * state outside the queue code.
	 */
	if (disconnect)
		queue->flags |= UVC_QUEUE_DISCONNECTED;
	spin_unlock_irqrestore(&queue->irqlock, flags);
}

struct uvc_buffer *uvc_queue_next_buffer(struct uvc_video_queue *queue,
		struct uvc_buffer *buf)
{
	struct uvc_buffer *nextbuf;
	unsigned long flags;

	if ((queue->flags & UVC_QUEUE_DROP_CORRUPTED) && buf->error) {
		buf->error = 0;
		buf->state = UVC_BUF_STATE_QUEUED;
		buf->bytesused = 0;								//ASUS_BSP Patrick "[A91][USB Cam][NA][Other]uvcvideo buffer error handling"
		vb2_set_plane_payload(&buf->buf, 0, 0);
		return buf;
	}

	spin_lock_irqsave(&queue->irqlock, flags);
	list_del(&buf->queue);
	if (!list_empty(&queue->irqqueue))
		nextbuf = list_first_entry(&queue->irqqueue, struct uvc_buffer,
					   queue);
	else
		nextbuf = NULL;
	spin_unlock_irqrestore(&queue->irqlock, flags);

	buf->state = buf->error ? VB2_BUF_STATE_ERROR : UVC_BUF_STATE_DONE;
	vb2_set_plane_payload(&buf->buf, 0, buf->bytesused);
	vb2_buffer_done(&buf->buf, VB2_BUF_STATE_DONE);

	return nextbuf;
}
