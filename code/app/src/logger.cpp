#include "logger.h"

#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

namespace {
constexpr uint32_t BUFFER_SIZE = 4096;
constexpr size_t LINE_CAPACITY = VIIKATE_LOG_MAX_LINE_LEN;
// One full line plus its newline always fits in a flush.
constexpr uint32_t FLUSH_CHUNK = LINE_CAPACITY + 1;
constexpr size_t FLUSH_STACK_SIZE = 2048;
// Below every other application thread: logging never delays control.
constexpr int FLUSH_PRIORITY = K_LOWEST_APPLICATION_THREAD_PRIO;

constexpr char NEWLINE = '\n';

k_spinlock lock;
uint32_t dropped;  // guarded by lock

void flush_thread(void *, void *, void *);
}  // namespace

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

namespace {
// Formatting. Runs on the caller's stack and never blocks.

void separate(viikate_log_line_t *line) {
    if (line->args++ > 0 && line->len < LINE_CAPACITY) {
        line->text[line->len++] = ' ';
        line->text[line->len] = '\0';
    }
}

void append(viikate_log_line_t *line, const char *fmt, ...) {
    separate(line);

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintk(
        line->text + line->len,
        LINE_CAPACITY + 1 - line->len,
        fmt,
        ap
    );
    va_end(ap);

    if (n > 0) {
        line->len = MIN(line->len + static_cast<size_t>(n), LINE_CAPACITY);
    }
}

// Output. Runs in the flush thread only.

fs_file_t file;
bool file_open;
bool file_disabled;
size_t file_size;

void rotated_name(char (&out)[32], int index) {
    snprintk(out, sizeof(out), VIIKATE_LOG_DIR "/log.%d.txt", index);
}

int open_file() {
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

    fs_dirent entry;
    rc = fs_stat(VIIKATE_LOG_FILE, &entry);
    file_size = rc == 0 ? entry.size : 0;
    return 0;
}

int rotate() {
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

void disable_file(int rc) {
    printk("log: %s unavailable (%d), console only\n", VIIKATE_LOG_FILE, rc);
    if (file_open) {
        fs_close(&file);
        file_open = false;
    }
    file_disabled = true;
}

// text holds whole lines only, so a line never straddles two files.
void write_file(const char *text, size_t len) {
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
            rc = static_cast<int>(n);
        } else if (static_cast<size_t>(n) != len) {
            rc = -ENOSPC;
        } else {
            file_size += len;
        }
    }
    if (rc != 0) {
        disable_file(rc);
    }
}

void output(const char *text, size_t len) {
    printk("%s", text);
    write_file(text, len);
}

// Moves whole lines out of the buffer into out, NUL terminated.
// Returns the number of bytes taken, 0 when the buffer is empty.
uint32_t take_lines(char (&out)[FLUSH_CHUNK + 1]) {
    k_spinlock_key_t key = k_spin_lock(&lock);
    uint32_t n = ring_buf_peek(
        &viikate_log_buffer,
        reinterpret_cast<uint8_t *>(out),
        FLUSH_CHUNK
    );
    while (n > 0 && out[n - 1] != NEWLINE) {
        --n;
    }
    ring_buf_get(&viikate_log_buffer, nullptr, n);
    k_spin_unlock(&lock, key);

    out[n] = '\0';
    return n;
}

uint32_t take_dropped() {
    k_spinlock_key_t key = k_spin_lock(&lock);
    uint32_t n = dropped;
    dropped = 0;
    k_spin_unlock(&lock, key);
    return n;
}

void flush_thread(void *, void *, void *) {
    static char chunk[FLUSH_CHUNK + 1];

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
            output(chunk, static_cast<size_t>(len));
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
}  // namespace

void viikate_log_begin(viikate_log_line_t *line) {
    line->text[0] = '\0';
    line->len = 0;
    line->args = 0;
}

void viikate_log_put_str(viikate_log_line_t *line, const char *value) {
    append(line, "%s", value != nullptr ? value : "(null)");
}

void viikate_log_put_char(viikate_log_line_t *line, char value) {
    append(line, "%c", value);
}

void viikate_log_put_bool(viikate_log_line_t *line, bool value) {
    append(line, "%s", value ? "true" : "false");
}

void viikate_log_put_int(viikate_log_line_t *line, long long value) {
    append(line, "%lld", value);
}

void viikate_log_put_uint(viikate_log_line_t *line, unsigned long long value) {
    append(line, "%llu", value);
}

void viikate_log_put_float(viikate_log_line_t *line, float value) {
    append(line, "%g", static_cast<double>(value));
}

void viikate_log_put_double(viikate_log_line_t *line, double value) {
    append(line, "%g", value);
}

void viikate_log_put_ptr(viikate_log_line_t *line, const void *value) {
    append(line, "%p", value);
}

bool viikate_log_end(viikate_log_line_t *line) {
    bool queued = false;

    k_spinlock_key_t key = k_spin_lock(&lock);
    // All or nothing, so the console and the file never get half a line.
    if (ring_buf_space_get(&viikate_log_buffer) >= line->len + 1) {
        ring_buf_put(
            &viikate_log_buffer,
            reinterpret_cast<const uint8_t *>(line->text),
            line->len
        );
        ring_buf_put(
            &viikate_log_buffer,
            reinterpret_cast<const uint8_t *>(&NEWLINE),
            1
        );
        queued = true;
    } else {
        ++dropped;
    }
    k_spin_unlock(&lock, key);

    // Never blocks, and is safe from ISRs.
    k_sem_give(&viikate_log_pending);
    return queued;
}
