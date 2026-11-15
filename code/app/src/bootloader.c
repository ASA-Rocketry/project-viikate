/*
 * Reboot into the Teensy bootloader when the host opens the console at
 * 134 baud, the same convention Teensyduino and teensy_loader_cli use. This
 * lets the board be reflashed without pressing the PROGRAM button.
 */
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#define REBOOT_BAUD 134
#define POLL_MS 50

static void watch_console(void *arg1, void *arg2, void *arg3) {
    const struct device *console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    (void)arg1;
    (void)arg2;
    (void)arg3;

    for (;;) {
        uint32_t baud = 0;

        if (device_is_ready(console)
            && uart_line_ctrl_get(console, UART_LINE_CTRL_BAUD_RATE, &baud) == 0
            && baud == REBOOT_BAUD) {
            // The Teensy's bootloader chip catches this breakpoint and
            // takes over, exactly like Teensyduino's reboot.
            __asm__ volatile("bkpt #251");
        }
        k_msleep(POLL_MS);
    }
}

K_THREAD_DEFINE(
    bootloader_watch_thread_id,
    512,
    watch_console,
    NULL,
    NULL,
    NULL,
    K_LOWEST_APPLICATION_THREAD_PRIO,
    0,
    0
);
