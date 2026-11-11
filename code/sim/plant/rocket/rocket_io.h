/*
 * Read a whole file into a buffer, for loading rocket design files. Two
 * implementations, one per build: rocket_io_stdio.c (plant_replay, host
 * programs) and rocket_io_native.c (native_sim, through the host calls).
 */

#ifndef ROCKET_IO_H_
#define ROCKET_IO_H_

#include <stddef.h>

/*
 * Read up to cap bytes of path into buf. Returns the number of bytes read,
 * which is cap when the file is cap bytes or longer, or -1 if it cannot be
 * read. buf is not NUL-terminated.
 */
long rocket_read_file(const char *path, char *buf, size_t cap);

#endif /* ROCKET_IO_H_ */
