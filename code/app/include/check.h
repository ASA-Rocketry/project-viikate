#ifndef VIIKATE_CHECK_H
#define VIIKATE_CHECK_H

/*
 * Runtime checks. Unlike assert(), they stay in flight builds.
 *
 *     if (!EXPECT(rc == 0, "rc", rc)) {
 *         return rc;
 *     }
 *     ASSERT(dt > 0.0f, "dt", dt);
 *
 * A failed check logs a CRIT line with the condition and where it is, then
 * any extra arguments, printed as by log_crit:
 *
 *     1234 CRIT EXPECT failed: rc == 0 at main.c:146 rc -5
 *
 * EXPECT is for things that should not happen but can be survived: it only
 * logs, and evaluates to the condition so the caller can fall back.
 *
 * ASSERT is for states the code cannot go on from. It stores every line
 * logged so far, including its own, on the SD card and the flash, then resets
 * the board. Builds with CONFIG_ASSERT (the simulator and the HIL test
 * firmware) halt instead, so a test fails where it happened rather than in a
 * reboot loop. The failed thread never goes on. In an ISR the rest of the
 * handler still runs, and the board stops as soon as it returns.
 */

#include <stdbool.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

#include "logger.h"

#define EXPECT(cond, ...) \
    (likely(cond) \
         ? true \
         : (VIIKATE_CHECK_FAILED_("EXPECT", #cond, ##__VA_ARGS__), false))

#define ASSERT(cond, ...) \
    do { \
        if (unlikely(!(cond))) { \
            VIIKATE_CHECK_FAILED_("ASSERT", #cond, ##__VA_ARGS__); \
            viikate_log_fatal(); \
        } \
    } while (0)

/* Implementation detail; use the macros above. */
#define VIIKATE_CHECK_FAILED_(kind, text, ...) \
    log_crit( \
        kind " failed:", \
        text, \
        "at", \
        __FILE_NAME__ ":" STRINGIFY(__LINE__), \
        ##__VA_ARGS__ \
    )

#endif
