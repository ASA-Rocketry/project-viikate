/* rocket_read_file() with stdio, for host programs such as plant_replay. */

#include <stdio.h>

#include "rocket_io.h"

long rocket_read_file(const char *path, char *buf, size_t cap) {
    FILE *f = fopen(path, "rb");
    size_t n;
    int failed;

    if (f == NULL) {
        return -1;
    }
    n = fread(buf, 1, cap, f);
    failed = ferror(f);
    fclose(f);
    return failed ? -1 : (long)n;
}
