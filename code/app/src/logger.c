#include "logger.h"

#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

#define BUFFER_SIZE 4096
#define LINE_CAPACITY VIIKATE_LOG_MAX_LINE_LEN
// One full line plus its newline always fits in a flush.
#define FLUSH_CHUNK (LINE_CAPACITY + 1)
#define FLUSH_STACK_SIZE 2048
// Below every other application thread: logging never delays control.
#define FLUSH_PRIORITY K_LOWEST_APPLICATION_THREAD_PRIO

static const char NEWLINE = '\n';

static struct k_spinlock lock;
static uint32_t dropped;  // guarded by lock

static void flush_thread(void *arg1, void *arg2, void *arg3);

RING_BUF_DECLARE(viikate_log_buffer, BUFFER_SIZE);
K_SEM_DEFINE(viikate_log_pending, 0, 1);
K_THREAD_DEFINE(
    viikate_log_thread,
    FLUSH_STACK_SIZE,
    flush_thread,
    NULL,
    NULL,
    NULL,
    FLUSH_PRIORITY,
    0,
    0
);

// Formatting. Runs on the caller's stack and never blocks.

typedef struct {
    char text[LINE_CAPACITY + 1];
    size_t len;
} line_t;

static void append(line_t *line, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintk(
        line->text + line->len,
        sizeof(line->text) - line->len,
        fmt,
        ap
    );
    va_end(ap);

    if (n > 0) {
        line->len = MIN(line->len + (size_t)n, LINE_CAPACITY);
    }
}

static const char *level_name(viikate_log_level_t level) {
    switch (level) {
        case VIIKATE_LOG_INFO:
            return "INFO";
        case VIIKATE_LOG_WARN:
            return "WARN";
        case VIIKATE_LOG_CRIT:
            return "CRIT";
    }
    return "????";
}

static void append_arg(line_t *line, const viikate_log_arg_t *arg) {
    switch (arg->type) {
        case VIIKATE_LOG_STR:
            append(line, " %s", arg->as.s != NULL ? arg->as.s : "(null)");
            break;
        case VIIKATE_LOG_CHAR:
            append(line, " %c", arg->as.c);
            break;
        case VIIKATE_LOG_BOOL:
            append(line, " %s", arg->as.b ? "true" : "false");
            break;
        case VIIKATE_LOG_INT:
            append(line, " %lld", arg->as.i);
            break;
        case VIIKATE_LOG_UINT:
            append(line, " %llu", arg->as.u);
            break;
        case VIIKATE_LOG_DOUBLE:
            append(line, " %g", arg->as.d);
            break;
        case VIIKATE_LOG_PTR:
            append(line, " %p", arg->as.p);
            break;
    }
}

// Output. Runs in the flush thread only.

static struct fs_file_t file;
static bool file_open;
static bool file_disabled;
static size_t file_size;

static void rotated_name(char out[32], int index) {
    snprintk(out, 32, VIIKATE_LOG_DIR "/log.%d.txt", index);
}

static int open_file(void) {
    fs_file_t_init(&file);
    int rc = fs_open(
        &file,
        VIIKATE_LOG_FILE,
        FS_O_CREATE | FS_O_WRITE | FS_O_APPEND
    );
    if (rc != 0) {
        return rc;
    }
    file_open = true;

    struct fs_dirent entry;
    rc = fs_stat(VIIKATE_LOG_FILE, &entry);
    file_size = rc == 0 ? entry.size : 0;
    return 0;
}

static int rotate(void) {
    char from[32];
    char to[32];

    fs_close(&file);
    file_open = false;

    rotated_name(to, VIIKATE_LOG_FILE_COUNT - 1);
    int rc = fs_unlink(to);
    if (rc != 0 && rc != -ENOENT) {
        return rc;
    }
    for (int i = VIIKATE_LOG_FILE_COUNT - 2; i >= 1; --i) {
        rotated_name(from, i);
        rotated_name(to, i + 1);
        rc = fs_rename(from, to);
        if (rc != 0 && rc != -ENOENT) {
            return rc;
        }
    }
    rotated_name(to, 1);
    rc = fs_rename(VIIKATE_LOG_FILE, to);
    if (rc != 0) {
        return rc;
    }
    return open_file();
}

static void disable_file(int rc) {
    printk("log: %s unavailable (%d), console only\n", VIIKATE_LOG_FILE, rc);
    if (file_open) {
        fs_close(&file);
        file_open = false;
    }
    file_disabled = true;
}

// text holds whole lines only, so a line never straddles two files.
static void write_file(const char *text, size_t len) {
    if (file_disabled) {
        return;
    }

    int rc = file_open ? 0 : open_file();
    if (rc == 0 && file_size > 0
        && file_size + len > VIIKATE_LOG_FILE_MAX_SIZE) {
        rc = rotate();
    }
    if (rc == 0) {
        ssize_t n = fs_write(&file, text, len);
        if (n < 0) {
            rc = (int)n;
        } else if ((size_t)n != len) {
            rc = -ENOSPC;
        } else {
            file_size += len;
        }
    }
    if (rc != 0) {
        disable_file(rc);
    }
}

static void output(const char *text, size_t len) {
    printk("%s", text);
    write_file(text, len);
}

// Moves whole lines out of the buffer into out, NUL terminated.
// Returns the number of bytes taken, 0 when the buffer is empty.
static uint32_t take_lines(char out[FLUSH_CHUNK + 1]) {
    k_spinlock_key_t key = k_spin_lock(&lock);
    uint32_t n =
        ring_buf_peek(&viikate_log_buffer, (uint8_t *)out, FLUSH_CHUNK);
    while (n > 0 && out[n - 1] != NEWLINE) {
        --n;
    }
    ring_buf_get(&viikate_log_buffer, NULL, n);
    k_spin_unlock(&lock, key);

    out[n] = '\0';
    return n;
}

static uint32_t take_dropped(void) {
    k_spinlock_key_t key = k_spin_lock(&lock);
    uint32_t n = dropped;
    dropped = 0;
    k_spin_unlock(&lock, key);
    return n;
}

static void flush_thread(void *arg1, void *arg2, void *arg3) {
    static char chunk[FLUSH_CHUNK + 1];
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    for (;;) {
        k_sem_take(&viikate_log_pending, K_FOREVER);

        uint32_t n;
        while ((n = take_lines(chunk)) > 0) {
            output(chunk, n);
        }

        uint32_t lost = take_dropped();
        if (lost > 0) {
            int len =
                snprintk(chunk, sizeof(chunk), "log: dropped %u lines\n", lost);
            output(chunk, (size_t)len);
        }

        // Commit to flash, so everything flushed so far survives power loss.
        if (file_open) {
            int rc = fs_sync(&file);
            if (rc != 0) {
                disable_file(rc);
            }
        }
    }
}

bool viikate_log_write(
    viikate_log_level_t level,
    const viikate_log_arg_t *args,
    size_t count
) {
    // Formatted on the caller's stack, so a line is complete before it is
    // queued.
    line_t line = {.len = 0};
    append(&line, "%lld %s", (long long)k_uptime_get(), level_name(level));
    for (size_t i = 0; i < count; ++i) {
        append_arg(&line, &args[i]);
    }

    bool queued = false;

    k_spinlock_key_t key = k_spin_lock(&lock);
    // All or nothing, so the console and the file never get half a line.
    if (ring_buf_space_get(&viikate_log_buffer) >= line.len + 1) {
        ring_buf_put(&viikate_log_buffer, (const uint8_t *)line.text, line.len);
        ring_buf_put(&viikate_log_buffer, (const uint8_t *)&NEWLINE, 1);
        queued = true;
    } else {
        ++dropped;
    }
    k_spin_unlock(&lock, key);

    // Never blocks, and is safe from ISRs.
    k_sem_give(&viikate_log_pending);
    return queued;
}
