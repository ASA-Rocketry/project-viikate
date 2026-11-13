#ifndef VIIKATE_LOGGER_H
#define VIIKATE_LOGGER_H

/*
 * Non-blocking line logger.
 *
 *     viikate_log("barometer...", "ok");
 *     viikate_log("pressure", pressure, "Pa after", tries, "tries");
 *
 * Arguments are printed separated by spaces and a newline is appended. Strings,
 * chars, bools, every integer and floating point type and pointers work, from
 * both C and C++. Lines longer than VIIKATE_LOG_MAX_LINE_LEN are cut off.
 *
 * The line is formatted on the caller's stack and only handed over once it is
 * complete, so a line is never flushed half written. A low priority thread
 * then prints it to the console and appends it to VIIKATE_LOG_FILE. The caller
 * never waits on the console or the flash, so logging is safe from control
 * loops and ISRs. If the buffer is full the line is dropped, viikate_log()
 * evaluates to false and the flush thread reports how many lines were lost.
 *
 * When the log file would grow past VIIKATE_LOG_FILE_MAX_SIZE it is rotated:
 * log.txt becomes log.1.txt, log.1.txt becomes log.2.txt and so on. At most
 * VIIKATE_LOG_FILE_COUNT files are kept, the oldest one is deleted.
 */

#include <stdbool.h>
#include <stddef.h>

#define VIIKATE_LOG_MAX_LINE_LEN 200
#define VIIKATE_LOG_DIR "/lfs"
#define VIIKATE_LOG_FILE VIIKATE_LOG_DIR "/log.txt"
#define VIIKATE_LOG_FILE_MAX_SIZE (64 * 1024)
#define VIIKATE_LOG_FILE_COUNT 4

#ifdef __cplusplus
extern "C" {
#endif

/* A line being assembled. Use viikate_log() instead of these directly. */
typedef struct {
    char text[VIIKATE_LOG_MAX_LINE_LEN + 1];
    size_t len;
    size_t args;
} viikate_log_line_t;

void viikate_log_begin(viikate_log_line_t *line);
void viikate_log_put_str(viikate_log_line_t *line, const char *value);
void viikate_log_put_char(viikate_log_line_t *line, char value);
void viikate_log_put_bool(viikate_log_line_t *line, bool value);
void viikate_log_put_int(viikate_log_line_t *line, long long value);
void viikate_log_put_uint(viikate_log_line_t *line, unsigned long long value);
void viikate_log_put_float(viikate_log_line_t *line, float value);
void viikate_log_put_double(viikate_log_line_t *line, double value);
void viikate_log_put_ptr(viikate_log_line_t *line, const void *value);
bool viikate_log_end(viikate_log_line_t *line);

#ifdef __cplusplus
}

namespace viikate {
namespace detail {
    // clang-format off
    inline void put(viikate_log_line_t *l, const char *v) { viikate_log_put_str(l, v); }
    inline void put(viikate_log_line_t *l, char v) { viikate_log_put_char(l, v); }
    inline void put(viikate_log_line_t *l, bool v) { viikate_log_put_bool(l, v); }
    inline void put(viikate_log_line_t *l, signed char v) { viikate_log_put_int(l, v); }
    inline void put(viikate_log_line_t *l, short v) { viikate_log_put_int(l, v); }
    inline void put(viikate_log_line_t *l, int v) { viikate_log_put_int(l, v); }
    inline void put(viikate_log_line_t *l, long v) { viikate_log_put_int(l, v); }
    inline void put(viikate_log_line_t *l, long long v) { viikate_log_put_int(l, v); }
    inline void put(viikate_log_line_t *l, unsigned char v) { viikate_log_put_uint(l, v); }
    inline void put(viikate_log_line_t *l, unsigned short v) { viikate_log_put_uint(l, v); }
    inline void put(viikate_log_line_t *l, unsigned int v) { viikate_log_put_uint(l, v); }
    inline void put(viikate_log_line_t *l, unsigned long v) { viikate_log_put_uint(l, v); }
    inline void put(viikate_log_line_t *l, unsigned long long v) { viikate_log_put_uint(l, v); }
    inline void put(viikate_log_line_t *l, float v) { viikate_log_put_float(l, v); }
    inline void put(viikate_log_line_t *l, double v) { viikate_log_put_double(l, v); }
    inline void put(viikate_log_line_t *l, long double v) { viikate_log_put_double(l, static_cast<double>(v)); }
    inline void put(viikate_log_line_t *l, const void *v) { viikate_log_put_ptr(l, v); }

    // clang-format on
}  // namespace detail

template<typename... Args>
bool log(const Args &...args) noexcept {
    viikate_log_line_t line;
    viikate_log_begin(&line);
    (detail::put(&line, args), ...);
    return viikate_log_end(&line);
}
}  // namespace viikate

    #define viikate_log(...) ::viikate::log(__VA_ARGS__)

#else

    #include <zephyr/sys/util_macro.h>

    // clang-format off
    #define VIIKATE_LOG_PUT_(value, line) \
        _Generic((value), \
            char *: viikate_log_put_str, \
            const char *: viikate_log_put_str, \
            char: viikate_log_put_char, \
            bool: viikate_log_put_bool, \
            signed char: viikate_log_put_int, \
            short: viikate_log_put_int, \
            int: viikate_log_put_int, \
            long: viikate_log_put_int, \
            long long: viikate_log_put_int, \
            unsigned char: viikate_log_put_uint, \
            unsigned short: viikate_log_put_uint, \
            unsigned int: viikate_log_put_uint, \
            unsigned long: viikate_log_put_uint, \
            unsigned long long: viikate_log_put_uint, \
            float: viikate_log_put_float, \
            double: viikate_log_put_double, \
            long double: viikate_log_put_double, \
            default: viikate_log_put_ptr)(line, value)

    #define viikate_log(...) \
        ({ \
            viikate_log_line_t viikate_log_line_; \
            viikate_log_begin(&viikate_log_line_); \
            FOR_EACH_FIXED_ARG(VIIKATE_LOG_PUT_, (;), &viikate_log_line_, __VA_ARGS__); \
            viikate_log_end(&viikate_log_line_); \
        })
    // clang-format on

#endif

#endif
