/*
 * rocket_read_file() for native_sim: the firmware image has no host libc
 * file I/O, so read through the native simulator's host trampolines.
 */

#include <nsi_host_trampolines.h>

#include "rocket_io.h"

#define HOST_O_RDONLY 0

long rocket_read_file(const char *path, char *buf, size_t cap) {
    int fd = nsi_host_open(path, HOST_O_RDONLY);
    size_t used = 0;

    if (fd < 0) {
        return -1;
    }
    while (used < cap) {
        long n = nsi_host_read(fd, buf + used, cap - used);

        if (n < 0) {
            nsi_host_close(fd);
            return -1;
        }
        if (n == 0) {
            break;
        }
        used += (size_t)n;
    }
    nsi_host_close(fd);
    return (long)used;
}
