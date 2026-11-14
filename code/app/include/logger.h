#ifndef VIIKATE_LOGGER_H
#define VIIKATE_LOGGER_H

/*
 * Non-blocking line logger.
 *
 *     log_info("barometer...", "ok");
 *     log_warn("pressure", pressure, "Pa after", tries, "tries");
 *     log_crit("imu lost");
 *
 * Arguments of any standard type (strings, chars, bools, integers, floats,
 * pointers) are printed separated by spaces, after the uptime in milliseconds
 * and the level, and a newline is appended:
 *
 *     1234 WARN pressure 101325 Pa after 3 tries
 *
 * Lines longer than VIIKATE_LOG_MAX_LINE_LEN are cut off.
 *
 * A line is formatted in full before it is handed over, so it is never
 * flushed half written. A low priority thread then prints it to the console
 * and appends it to VIIKATE_LOG_FILE. The caller never waits on the console or
 * the flash, so logging is safe from control loops and ISRs. If the buffer is
 * full the line is dropped, the call evaluates to false and the flush thread
 * reports how many lines were lost.
 *
 * When the log file would grow past VIIKATE_LOG_FILE_MAX_SIZE it is rotated:
 * log.txt becomes log.1.txt, log.1.txt becomes log.2.txt and so on. At most
 * VIIKATE_LOG_FILE_COUNT files are kept, the oldest one is deleted.
 */

#include <stdbool.h>
#include <stddef.h>
#include <zephyr/sys/util_macro.h>

#define VIIKATE_LOG_MAX_LINE_LEN 200
#define VIIKATE_LOG_DIR "/lfs"
#define VIIKATE_LOG_FILE VIIKATE_LOG_DIR "/log.txt"
#define VIIKATE_LOG_FILE_MAX_SIZE (64 * 1024)
#define VIIKATE_LOG_FILE_COUNT 4

#define log_info(...) VIIKATE_LOG_(VIIKATE_LOG_INFO, __VA_ARGS__)
#define log_warn(...) VIIKATE_LOG_(VIIKATE_LOG_WARN, __VA_ARGS__)
#define log_crit(...) VIIKATE_LOG_(VIIKATE_LOG_CRIT, __VA_ARGS__)

/* Implementation details below; use the macros above. */

typedef enum {
    VIIKATE_LOG_INFO,
    VIIKATE_LOG_WARN,
    VIIKATE_LOG_CRIT,
} viikate_log_level_t;

typedef enum {
    VIIKATE_LOG_STR,
    VIIKATE_LOG_CHAR,
    VIIKATE_LOG_BOOL,
    VIIKATE_LOG_INT,
    VIIKATE_LOG_UINT,
    VIIKATE_LOG_DOUBLE,
    VIIKATE_LOG_PTR,
} viikate_log_type_t;

typedef struct {
    viikate_log_type_t type;

    union {
        const char *s;
        char c;
        bool b;
        long long i;
        unsigned long long u;
        double d;
        const void *p;
    } as;
} viikate_log_arg_t;

bool viikate_log_write(
    viikate_log_level_t level,
    const viikate_log_arg_t *args,
    size_t count
);

// clang-format off
static inline viikate_log_arg_t viikate_log_str_(const char *v) { return (viikate_log_arg_t){VIIKATE_LOG_STR, {.s = v}}; }
static inline viikate_log_arg_t viikate_log_char_(char v) { return (viikate_log_arg_t){VIIKATE_LOG_CHAR, {.c = v}}; }
static inline viikate_log_arg_t viikate_log_bool_(bool v) { return (viikate_log_arg_t){VIIKATE_LOG_BOOL, {.b = v}}; }
static inline viikate_log_arg_t viikate_log_int_(long long v) { return (viikate_log_arg_t){VIIKATE_LOG_INT, {.i = v}}; }
static inline viikate_log_arg_t viikate_log_uint_(unsigned long long v) { return (viikate_log_arg_t){VIIKATE_LOG_UINT, {.u = v}}; }
static inline viikate_log_arg_t viikate_log_double_(double v) { return (viikate_log_arg_t){VIIKATE_LOG_DOUBLE, {.d = v}}; }
static inline viikate_log_arg_t viikate_log_ptr_(const void *v) { return (viikate_log_arg_t){VIIKATE_LOG_PTR, {.p = v}}; }

#define VIIKATE_LOG_ARG_(value) \
    _Generic((value), \
        char *: viikate_log_str_, \
        const char *: viikate_log_str_, \
        char: viikate_log_char_, \
        bool: viikate_log_bool_, \
        signed char: viikate_log_int_, \
        short: viikate_log_int_, \
        int: viikate_log_int_, \
        long: viikate_log_int_, \
        long long: viikate_log_int_, \
        unsigned char: viikate_log_uint_, \
        unsigned short: viikate_log_uint_, \
        unsigned int: viikate_log_uint_, \
        unsigned long: viikate_log_uint_, \
        unsigned long long: viikate_log_uint_, \
        float: viikate_log_double_, \
        double: viikate_log_double_, \
        long double: viikate_log_double_, \
        default: viikate_log_ptr_)(value)

#define VIIKATE_LOG_(level, ...) \
    viikate_log_write( \
        (level), \
        (const viikate_log_arg_t[]){FOR_EACH(VIIKATE_LOG_ARG_, (,), __VA_ARGS__)}, \
        NUM_VA_ARGS(__VA_ARGS__))
// clang-format on

#endif
