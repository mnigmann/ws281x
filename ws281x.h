#ifndef WS281X_H
#define WS281X_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define MAJOR_NUM 238

struct ws281x_chconfig {
    uint8_t mask;
    uint8_t slot;
};

struct ws281x_config {
    uint32_t flags;
    uint32_t mask;
    uint32_t stringlen;
};

#define WS281X_BYTES_PER_LED    0x00000007
#define WS281X_AUTO_UPDATE      0x00000008

// Reserved flags
#define WS281X_USE_8BIT         0x80000000

#define IOCTL_CONFIG   _IOWR(MAJOR_NUM, 0, struct config *)
#define IOCTL_CHCONFIG _IOWR(MAJOR_NUM, 1, struct chconfig *)
#define IOCTL_UPDATE   _IOWR(MAJOR_NUM, 2, int)

#define DEVICE_FILE_NAME "ws281x"

#endif
