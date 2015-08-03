#
# Makefile for the video capture/playback device drivers.
#

tuner-objs	:=	tuner-core.o

msp3400-objs	:=	msp3400-driver.o msp3400-kthreads.o

stkwebcam-objs	:=	stk-webcam.o stk-sensor.o

omap2cam-objs	:=	omap24xxcam.o omap24xxcam-dma.o

videodev-objs	:=	v4l2-dev.o v4l2-ioctl.o v4l2-device.o v4l2-fh.o \
			v4l2-event.o v4l2-ctrls.o v4l2-subdev.o

ifeq ($(CONFIG_COMPAT),y)
  videodev-objs += v4l2-compat-ioctl32.o
endif

# V4L2 core modules

obj-$(CONFIG_VIDEO_DEV) += videodev.o v4l2-int-device.o
obj-$(CONFIG_VIDEO_V4L2_COMMON) += v4l2-common.o
obj-$(CONFIG_VIDEOBUF_GEN) += videobuf-core.o
obj-$(CONFIG_VIDEOBUF_DMA_SG) += videobuf-dma-sg.o
obj-$(CONFIG_VIDEOBUF_DMA_CONTIG) += videobuf-dma-contig.o
obj-$(CONFIG_VIDEOBUF_VMALLOC) += videobuf-vmalloc.o
obj-$(CONFIG_VIDEOBUF_DVB) += videobuf-dvb.o
obj-$(CONFIG_VIDEO_BTCX)  += btcx-risc.o

obj-$(CONFIG_VIDEOBUF2_CORE)		+= videobuf2-core.o
obj-$(CONFIG_VIDEOBUF2_MEMOPS)		+= videobuf2-memops.o
obj-$(CONFIG_VIDEOBUF2_VMALLOC)		+= videobuf2-vmalloc.o
obj-$(CONFIG_VIDEOBUF2_DMA_CONTIG)	+= videobuf2-dma-contig.o
obj-$(CONFIG_VIDEOBUF2_DMA_SG)		+= videobuf2-dma-sg.o

obj-$(CONFIG_V4L2_MEM2MEM_DEV) += v4l2-mem2mem.o
