#include "persistent_memory.h"

#include <string.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/kvss/nvs.h>
#include <zephyr/storage/flash_map.h>

#define PERSIST_MAX_LEN 32  // Largest single data to be saved in bits
#define PERSIST_QUEUE_DEPTH 8  // How many messages can wait in the queue
#define TASK_PERSIST_STACK_SIZE 2048
#define TASK_PERSIST_PRIORITY \
    K_LOWEST_APPLICATION_THREAD_PRIO  // Writing is of lowest priority due to possibility of long execution period (RMS).

typedef struct {
    uint16_t id;
    uint8_t len;
    uint8_t data[PERSIST_MAX_LEN];
} write_item_t;

static struct nvs_fs fs;
static bool ready = false;

K_MSGQ_DEFINE(write_q, sizeof(write_item_t), PERSIST_QUEUE_DEPTH, 4);

static void task_persist(void *arg1, void *arg2, void *arg3) {
    (void)arg1;
    (void)arg2;
    (void)arg3;

    write_item_t item;
    while (1) {
        k_msgq_get(&write_q, &item, K_FOREVER);
        nvs_write(&fs, item.id, item.data, item.len);
    }
}

K_THREAD_DEFINE(
    task_persist_thread_id,
    TASK_PERSIST_STACK_SIZE,
    task_persist,
    NULL,
    NULL,
    NULL,
    TASK_PERSIST_PRIORITY,
    0,
    0
);

bool persist_init(void) {
    if (ready) {
        return true;
    }

    struct flash_pages_info info;

    fs.flash_device = PARTITION_DEVICE(storage_partition);
    if (!device_is_ready(fs.flash_device)) {
        return false;
    }
    fs.offset = PARTITION_OFFSET(storage_partition);
    if (flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info) != 0) {
        return false;
    }
    fs.sector_size = info.size;
    fs.sector_count = PARTITION_SIZE(storage_partition) / info.size;
    if (fs.sector_count < 2) {
        return false;
    }

    if (nvs_mount(&fs) != 0) {
        return false;
    }

    ready = true;
    return true;
}

// Non blocking
bool persist_write(uint16_t id, const void *data, size_t len) {
    if (!ready || len > PERSIST_MAX_LEN) {
        return false;
    }
    write_item_t item = {.id = id, .len = len};
    memcpy(
        item.data,
        data,
        len
    );  // Copy data so that it exists at execution time
    return k_msgq_put(&write_q, &item, K_NO_WAIT) == 0;
}

// Synchronic. Only to be used during boot.
bool persist_read(uint16_t id, void *data, size_t len) {
    return ready
        && nvs_read(&fs, id, data, len)
        == (ssize_t)len;  // nvs_read must read the correct amount of data
}