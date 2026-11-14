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
 * pointers) and the linmath types are printed separated by spaces, after the
 * uptime in milliseconds and the level, and a newline is appended:
 *
 *     1234 WARN pressure 101325 Pa after 3 tries
 *     1250 INFO acceleration [0.1, -0.02, 9.81]
 *
 * vec2, vec3, vec4 and quat print as [x, y, ...], mat4x4 as four such rows
 * inside brackets. A vector received as a function parameter has decayed to
 * a plain float pointer and prints as an address.
 *
 * Lines longer than VIIKATE_LOG_MAX_LINE_LEN are cut off.
 *
 * A line is formatted in full before it is handed over, so it is never
 * flushed half written. A low priority thread then prints it to the console
 * and appends it to log.txt on the microSD card (VIIKATE_LOG_SD_DIR). CRIT
 * lines are also appended to crit.txt on the onboard flash (VIIKATE_LOG_DIR),
 * so the most important ones survive a lost or broken card. If one of them
 * fails, logging carries on to the other and the console. The card is mounted
 * by its own thread, which tries again every few seconds while it is missing
 * or after it failed, so a card inserted late or reseated is picked up again;
 * lines logged while it is down only reach the console and, if CRIT, the
 * flash. FAT and exFAT cards both work. The caller never
 * waits on the console or storage, so logging is safe from control loops and
 * ISRs. If the buffer is full the line is dropped, the call evaluates to false
 * and the flush thread reports how many lines were lost.
 *
 * When log.txt would grow past VIIKATE_LOG_FILE_MAX_SIZE it is rotated:
 * log.txt becomes log.1.txt, log.1.txt becomes log.2.txt and so on. At most
 * VIIKATE_LOG_FILE_COUNT files are kept, the oldest one is deleted. crit.txt
 * rotates the same way within its own, smaller limits, so the flash only ever
 * holds the latest critical lines.
 */

#include <stdbool.h>
#include <stddef.h>
#include <zephyr/sys/util_macro.h>

#define VIIKATE_LOG_MAX_LINE_LEN 200
#define VIIKATE_LOG_DIR "/lfs"
#define VIIKATE_LOG_SD_DIR "/SD:"
#define VIIKATE_LOG_FILE_MAX_SIZE (64 * 1024)
#define VIIKATE_LOG_FILE_COUNT 4
#define VIIKATE_LOG_CRIT_FILE_MAX_SIZE (16 * 1024)
#define VIIKATE_LOG_CRIT_FILE_COUNT 2

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
    VIIKATE_LOG_FLOATS,
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

        struct {
            const float *v;
            unsigned char rows;
            unsigned char cols;
        } f;
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
static inline viikate_log_arg_t viikate_log_vec2_(const void *v) { return (viikate_log_arg_t){VIIKATE_LOG_FLOATS, {.f = {(const float *)v, 1, 2}}}; }
static inline viikate_log_arg_t viikate_log_vec3_(const void *v) { return (viikate_log_arg_t){VIIKATE_LOG_FLOATS, {.f = {(const float *)v, 1, 3}}}; }
static inline viikate_log_arg_t viikate_log_vec4_(const void *v) { return (viikate_log_arg_t){VIIKATE_LOG_FLOATS, {.f = {(const float *)v, 1, 4}}}; }
static inline viikate_log_arg_t viikate_log_mat4x4_(const void *v) { return (viikate_log_arg_t){VIIKATE_LOG_FLOATS, {.f = {(const float *)v, 4, 4}}}; }

/* linmath types (vec2, vec3, vec4, quat, mat4x4) are float arrays, which
 * _Generic only sees as float *. Matching on a pointer to the argument's type
 * keeps the array size; everything else is matched by value. */
#define VIIKATE_LOG_ARG_(value) \
    _Generic((__typeof__(value) *)0, \
        float (*)[2]: viikate_log_vec2_, \
        const float (*)[2]: viikate_log_vec2_, \
        float (*)[3]: viikate_log_vec3_, \
        const float (*)[3]: viikate_log_vec3_, \
        float (*)[4]: viikate_log_vec4_, \
        const float (*)[4]: viikate_log_vec4_, \
        float (*)[4][4]: viikate_log_mat4x4_, \
        const float (*)[4][4]: viikate_log_mat4x4_, \
        default: VIIKATE_LOG_SCALAR_(value))(value)

#define VIIKATE_LOG_SCALAR_(value) \
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
        default: viikate_log_ptr_)

#define VIIKATE_LOG_(level, ...) \
    viikate_log_write( \
        (level), \
        (const viikate_log_arg_t[]){FOR_EACH(VIIKATE_LOG_ARG_, (,), __VA_ARGS__)}, \
        NUM_VA_ARGS(__VA_ARGS__))
// clang-format on

#endif
