/*
 * Minimal V4L2 definitions for ESP32-P4
 * Extracted from linux/videodev2.h
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// V4L2 buffer types
#define V4L2_BUF_TYPE_VIDEO_CAPTURE 1

// V4L2 memory types
#define V4L2_MEMORY_MMAP 1

// V4L2 pixel formats
#define V4L2_PIX_FMT_RGB565  0x52474252  /* 16  RGB-5-6-5     */
#define V4L2_PIX_FMT_YUYV    0x56595559  /* 16  YUV 4:2:2     */

// IOCTL commands
#define VIDIOC_QUERYCAP     0xC0685600
#define VIDIOC_G_FMT        0xC0D05604
#define VIDIOC_S_FMT        0xC0D05605
#define VIDIOC_REQBUFS      0xC0145608
#define VIDIOC_QUERYBUF     0xC0585609
#define VIDIOC_QBUF         0xC058560F
#define VIDIOC_DQBUF        0xC0585611
#define VIDIOC_STREAMON     0x40045612
#define VIDIOC_STREAMOFF    0x40045613

// V4L2 capability structure
struct v4l2_capability {
    uint8_t  driver[16];
    uint8_t  card[32];
    uint8_t  bus_info[32];
    uint32_t version;
    uint32_t capabilities;
    uint32_t device_caps;
    uint32_t reserved[3];
};

// V4L2 pixel format structure
struct v4l2_pix_format {
    uint32_t width;
    uint32_t height;
    uint32_t pixelformat;
    uint32_t field;
    uint32_t bytesperline;
    uint32_t sizeimage;
    uint32_t colorspace;
    uint32_t priv;
    uint32_t flags;
    uint32_t ycbcr_enc;
    uint32_t quantization;
    uint32_t xfer_func;
};

// V4L2 format structure
struct v4l2_format {
    uint32_t type;
    union {
        struct v4l2_pix_format pix;
        uint8_t raw_data[200];
    } fmt;
};

// V4L2 buffer request
struct v4l2_requestbuffers {
    uint32_t count;
    uint32_t type;
    uint32_t memory;
    uint32_t capabilities;
    uint32_t reserved[1];
};

// V4L2 buffer structure
struct v4l2_buffer {
    uint32_t index;
    uint32_t type;
    uint32_t bytesused;
    uint32_t flags;
    uint32_t field;
    struct {
        long tv_sec;
        long tv_usec;
    } timestamp;
    struct {
        uint32_t type;
        uint32_t flags;
        uint8_t  data[4];
    } timecode;
    uint32_t sequence;
    uint32_t memory;
    union {
        uint32_t offset;
        unsigned long userptr;
        void *planes;
        int32_t fd;
    } m;
    uint32_t length;
    uint32_t reserved2;
    uint32_t reserved;
};

#ifdef __cplusplus
}
#endif
