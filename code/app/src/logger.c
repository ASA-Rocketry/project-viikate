#include "logger.h"

#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

#define BUFFER_SIZE 4096
#define LINE_CAPACITY VIIKATE_LOG_MAX_LINE_LEN
// One full line, with its level tag and newline, always fits in a flush.
#define FLUSH_CHUNK (1 + LINE_CAPACITY + 1)
// FatFs and the SD card driver run on these threads.
#define FLUSH_STACK_SIZE 4096
#define MOUNT_STACK_SIZE 4096
// Below the control threads: logging never delays control. The mount thread
// is lower still, so probing for a card never delays a flush.
#define FLUSH_PRIORITY (K_LOWEST_APPLICATION_THREAD_PRIO - 1)
#define MOUNT_PRIORITY K_LOWEST_APPLICATION_THREAD_PRIO
// How often a missing or failed SD card is tried again.
#define MOUNT_RETRY K_SECONDS(5)

static const char NEWLINE = '\n';

static struct k_spinlock lock;
static uint32_t dropped;  // guarded by lock

static void flush_thread(void *arg1, void *arg2, void *arg3);
#ifdef CONFIG_FILE_SYSTEM
static void mount_thread(void *arg1, void *arg2, void *arg3);
#endif

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
#ifdef CONFIG_FILE_SYSTEM
// Given by the mount thread after its first try.
K_SEM_DEFINE(viikate_log_sd_tried, 0, 1);
K_THREAD_DEFINE(
    viikate_log_mount_thread,
    MOUNT_STACK_SIZE,
    mount_thread,
    NULL,
    NULL,
    NULL,
    MOUNT_PRIORITY,
    0,
    0
);
#endif

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

// " [x, y, z]" for a vector, " [[a, b], [c, d]]" for a matrix (row-major).
static void
append_floats(line_t *line, const float *v, unsigned rows, unsigned cols) {
    append(line, rows > 1 ? " [" : " ");
    for (unsigned r = 0; r < rows; ++r) {
        append(line, r > 0 ? ", [" : "[");
        for (unsigned c = 0; c < cols; ++c) {
            append(line, c > 0 ? ", %g" : "%g", (double)v[r * cols + c]);
        }
        append(line, "]");
    }
    if (rows > 1) {
        append(line, "]");
    }
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
        case VIIKATE_LOG_FLOATS:
            append_floats(line, arg->as.f.v, arg->as.f.rows, arg->as.f.cols);
            break;
    }
}

// Output. Runs in the flush thread only.

// Storage. Without file systems (native_sim) lines only go to the console.
#ifdef CONFIG_FILE_SYSTEM

FS_FSTAB_DECLARE_ENTRY(DT_NODELABEL(sd));

// A log file and its rotated copies. When the live file would grow past
// max_size, name.txt becomes name.1.txt and so on, keeping count files: the
// oldest lines are dropped, so the files act as a ring.
typedef struct {
    const char *dir;
    const char *name;
    size_t max_size;
    int count;

    struct fs_file_t file;
    bool open;
    bool disabled;
    size_t size;
} target_t;

// Every line, on the microSD card.
static target_t sd_log = {
    .dir = VIIKATE_LOG_SD_DIR,
    .name = "log",
    .max_size = VIIKATE_LOG_FILE_MAX_SIZE,
    .count = VIIKATE_LOG_FILE_COUNT,
};

// Who owns the SD card. The mount thread mounts it (DOWN -> UP). The flush
// thread writes to it only while it is UP and, when a write fails, closes the
// file and hands it back (UP -> BROKEN). The mount thread then unmounts it and
// tries again (BROKEN -> DOWN -> UP).
enum { SD_DOWN, SD_UP, SD_BROKEN };

static atomic_t sd_state = ATOMIC_INIT(SD_DOWN);

// CRIT lines only, on the onboard flash: a small ring that survives a lost
// or broken SD card without wearing out the flash with every line.
static target_t crit_log = {
    .dir = VIIKATE_LOG_DIR,
    .name = "crit",
    .max_size = VIIKATE_LOG_CRIT_FILE_MAX_SIZE,
    .count = VIIKATE_LOG_CRIT_FILE_COUNT,
};

    #define PATH_LEN 32

// index 0 is the live file, name.txt; 1 and up are rotated copies.
static void log_path(const target_t *t, int index, char out[PATH_LEN]) {
    if (index == 0) {
        snprintk(out, PATH_LEN, "%s/%s.txt", t->dir, t->name);
    } else {
        snprintk(out, PATH_LEN, "%s/%s.%d.txt", t->dir, t->name, index);
    }
}

static int open_file(target_t *t) {
    char path[PATH_LEN];
    log_path(t, 0, path);

    fs_file_t_init(&t->file);
    int rc = fs_open(&t->file, path, FS_O_CREATE | FS_O_WRITE | FS_O_APPEND);
    if (rc != 0) {
        return rc;
    }
    t->open = true;

    struct fs_dirent entry;
    rc = fs_stat(path, &entry);
    t->size = rc == 0 ? entry.size : 0;
    return 0;
}

static int rotate(target_t *t) {
    char from[PATH_LEN];
    char to[PATH_LEN];

    fs_close(&t->file);
    t->open = false;

    log_path(t, t->count - 1, to);
    int rc = fs_unlink(to);
    if (rc != 0 && rc != -ENOENT) {
        return rc;
    }
    for (int i = t->count - 2; i >= 0; --i) {
        log_path(t, i, from);
        log_path(t, i + 1, to);
        rc = fs_rename(from, to);
        if (rc != 0 && (rc != -ENOENT || i == 0)) {
            return rc;
        }
    }
    return open_file(t);
}

static void close_target(target_t *t) {
    if (t->open) {
        fs_close(&t->file);
        t->open = false;
    }
}

static bool target_ready(const target_t *t) {
    if (t == &sd_log) {
        return atomic_get(&sd_state) == SD_UP;
    }
    return !t->disabled;
}

// A failed SD card goes back to the mount thread, which tries it again. The
// flash is not retried. Either way the console and the other target go on.
static void fail_target(target_t *t, int rc) {
    close_target(t);
    if (t == &sd_log) {
        atomic_set(&sd_state, SD_BROKEN);
        log_crit("log: SD card failed, error", rc, "retrying");
    } else {
        printk("log: %s/%s.txt unavailable (%d)\n", t->dir, t->name, rc);
        t->disabled = true;
    }
}

// text is one whole line, so a line never straddles two files.
static void write_target(target_t *t, const char *text, size_t len) {
    if (!target_ready(t)) {
        return;
    }

    int rc = 0;
    if (!t->open) {
        rc = open_file(t);
    }
    if (rc == 0 && t->size > 0 && t->size + len > t->max_size) {
        rc = rotate(t);
    }
    if (rc == 0) {
        ssize_t n = fs_write(&t->file, text, len);
        if (n < 0) {
            rc = (int)n;
        } else if ((size_t)n != len) {
            rc = -ENOSPC;
        } else {
            t->size += len;
        }
    }
    if (rc != 0) {
        fail_target(t, rc);
    }
}

// Commits everything written so far, so it survives power loss.
static void sync_target(target_t *t) {
    if (t->open) {
        int rc = fs_sync(&t->file);
        if (rc != 0) {
            fail_target(t, rc);
        }
    }
}

// Mounts the SD card, so the flush thread never waits on a card that does
// not answer: with no card detect pin, a missing card only shows up after the
// driver's timeouts.
static void mount_thread(void *arg1, void *arg2, void *arg3) {
    struct fs_mount_t *mount = &FS_FSTAB_ENTRY(DT_NODELABEL(sd));
    int last_rc = 0;
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    for (bool first = true;; first = false) {
        if (atomic_get(&sd_state) == SD_BROKEN) {
            // Errors are ignored: the card may be gone already.
            (void)fs_unmount(mount);
            atomic_set(&sd_state, SD_DOWN);
        }
        if (atomic_get(&sd_state) == SD_DOWN) {
            int rc = fs_mount(mount);
            if (rc == 0) {
                last_rc = 0;
                atomic_set(&sd_state, SD_UP);
                log_info("log: SD card mounted");
            } else if (rc != last_rc) {
                // Only once per new error, not on every retry.
                last_rc = rc;
                log_warn("log: no SD card, error", rc, "retrying");
            }
        }
        if (first) {
            k_sem_give(&viikate_log_sd_tried);
        }
        k_sleep(MOUNT_RETRY);
    }
}

static void
store_line(viikate_log_level_t level, const char *text, size_t len) {
    write_target(&sd_log, text, len);
    if (level == VIIKATE_LOG_CRIT) {
        write_target(&crit_log, text, len);
    }
}

static void sync_storage(void) {
    sync_target(&sd_log);
    sync_target(&crit_log);
}

// Waits a little for the card, so the boot lines reach it too.
static void wait_for_storage(void) {
    (void)k_sem_take(&viikate_log_sd_tried, MOUNT_RETRY);
}

#else

static void
store_line(viikate_log_level_t level, const char *text, size_t len) {
    ARG_UNUSED(level);
    ARG_UNUSED(text);
    ARG_UNUSED(len);
}

static void sync_storage(void) {}

static void wait_for_storage(void) {}

#endif

// text is one line including its newline.
static void
output_line(viikate_log_level_t level, const char *text, size_t len) {
    printk("%.*s", (int)len, text);
    store_line(level, text, len);
}

// In the buffer each line is preceded by one byte holding its level, which
// decides where it is written and is never printed.
static void output(const char *chunk, size_t len) {
    while (len > 0) {
        const char *end = memchr(chunk, NEWLINE, len);
        size_t line_len = (size_t)(end - chunk) + 1;
        output_line((viikate_log_level_t)chunk[0], chunk + 1, line_len - 1);
        chunk += line_len;
        len -= line_len;
    }
}

// Moves whole lines out of the buffer into out.
// Returns the number of bytes taken, 0 when the buffer is empty.
static uint32_t take_lines(char out[FLUSH_CHUNK]) {
    k_spinlock_key_t key = k_spin_lock(&lock);
    uint32_t n =
        ring_buf_peek(&viikate_log_buffer, (uint8_t *)out, FLUSH_CHUNK);
    while (n > 0 && out[n - 1] != NEWLINE) {
        --n;
    }
    ring_buf_get(&viikate_log_buffer, NULL, n);
    k_spin_unlock(&lock, key);
    return n;
}

static uint32_t take_dropped(void) {
    k_spinlock_key_t key = k_spin_lock(&lock);
    uint32_t n = dropped;
    dropped = 0;
    k_spin_unlock(&lock, key);
    return n;
}

// Set by viikate_log_fatal: store everything queued, then stop the board.
static atomic_t is_logging_irrecoverable_failures = ATOMIC_INIT(0);

static char flush_chunk[FLUSH_CHUNK];

// Prints and stores every queued line, then commits the files.
static void flush_all(void) {
    uint32_t n;
    while ((n = take_lines(flush_chunk)) > 0) {
        output(flush_chunk, n);
    }

    uint32_t lost = take_dropped();
    if (lost > 0) {
        int len = snprintk(
            flush_chunk,
            sizeof(flush_chunk),
            "log: dropped %u lines\n",
            lost
        );
        output_line(VIIKATE_LOG_WARN, flush_chunk, (size_t)len);
    }

    sync_storage();
}

// Builds with CONFIG_ASSERT (the simulator and the HIL test firmware) halt,
// so a run stops where the failure happened. Flight builds reset and go on.
static FUNC_NORETURN void stop_board(void) {
    // Gives the USB console a moment to send the last lines.
    k_sleep(K_MSEC(100));
#ifdef CONFIG_ASSERT
    k_panic();
#else
    sys_reboot(SYS_REBOOT_COLD);
#endif
    CODE_UNREACHABLE;
}

static void flush_thread(void *arg1, void *arg2, void *arg3) {
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    wait_for_storage();

    for (;;) {
        k_sem_take(&viikate_log_pending, K_FOREVER);
        flush_all();
        if (atomic_get(&is_logging_irrecoverable_failures)) {
            // Again, for lines queued while the files were being synced.
            flush_all();
            stop_board();
        }
    }
}

void viikate_log_fatal(void) {
    atomic_set(&is_logging_irrecoverable_failures, 1);

    if (k_current_get() == viikate_log_thread) {
        // Failed inside the logger thread, which nobody else will flush.
        flush_all();
        stop_board();
    }

    // Ahead of every other thread, so the logs are stored before anything
    // else runs. From an ISR, that happens as soon as the ISR returns.
    k_thread_priority_set(viikate_log_thread, K_HIGHEST_THREAD_PRIO);
    k_sem_give(&viikate_log_pending);

    if (!k_is_in_isr()) {
        // The failed thread must not carry on.
        for (;;) {
            k_sleep(K_FOREVER);
        }
    }
}

// Queues one line, all or nothing, so the console and the files never get
// half a line. Returns false if the buffer was full and the line dropped.
static bool queue_line(viikate_log_level_t level, const line_t *line) {
    const uint8_t tag = (uint8_t)level;
    bool queued = false;

    k_spinlock_key_t key = k_spin_lock(&lock);
    if (ring_buf_space_get(&viikate_log_buffer) >= 1 + line->len + 1) {
        ring_buf_put(&viikate_log_buffer, &tag, 1);
        ring_buf_put(
            &viikate_log_buffer,
            (const uint8_t *)line->text,
            line->len
        );
        ring_buf_put(&viikate_log_buffer, (const uint8_t *)&NEWLINE, 1);
        queued = true;
    } else {
        ++dropped;
    }
    k_spin_unlock(&lock, key);
    return queued;
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
    bool queued = queue_line(level, &line);

    // Never blocks, and is safe from ISRs.
    k_sem_give(&viikate_log_pending);
    return queued;
}
