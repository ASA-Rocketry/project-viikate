#include "logger.h"

#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

namespace {
constexpr uint32_t BUFFER_SIZE = 4096;
// One full message plus its newline always fits in a flush.
constexpr uint32_t FLUSH_CHUNK = VIIKATE_LOG_MAX_MESSAGE_LEN + 1;
constexpr size_t FLUSH_STACK_SIZE = 1024;
// Below every other application thread: logging never delays control.
constexpr int FLUSH_PRIORITY = K_LOWEST_APPLICATION_THREAD_PRIO;

constexpr uint8_t NEWLINE = '\n';

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

        while (take_lines(chunk) > 0) {
            printk("%s", chunk);
        }

        uint32_t lost = take_dropped();
        if (lost > 0) {
            printk("log: dropped %u messages\n", lost);
        }
    }
}
}  // namespace

bool viikate::log(const char *msg) noexcept {
    if (msg == nullptr) {
        return false;
    }

    size_t len = strnlen(msg, VIIKATE_LOG_MAX_MESSAGE_LEN);
    if (len > 0 && msg[len - 1] == NEWLINE) {
        --len;
    }

    bool queued = false;
    k_spinlock_key_t key = k_spin_lock(&lock);
    // All or nothing, so the console never shows half a line.
    if (ring_buf_space_get(&viikate_log_buffer) >= len + 1) {
        ring_buf_put(
            &viikate_log_buffer,
            reinterpret_cast<const uint8_t *>(msg),
            len
        );
        ring_buf_put(&viikate_log_buffer, &NEWLINE, 1);
        queued = true;
    } else {
        ++dropped;
    }
    k_spin_unlock(&lock, key);

    // Never blocks, and is safe from ISRs.
    k_sem_give(&viikate_log_pending);
    return queued;
}

extern "C" bool viikate_log(const char *msg) {
    return viikate::log(msg);
}
