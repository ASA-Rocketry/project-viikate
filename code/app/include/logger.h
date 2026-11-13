#ifndef VIIKATE_LOGGER_H
#define VIIKATE_LOGGER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Longest message kept; anything past it is cut off. */
#define VIIKATE_LOG_MAX_MESSAGE_LEN 255

/*
 * Queue one line for the console without blocking.
 *
 * The message is copied into a RAM ring buffer and a low priority thread
 * writes it out later, so the caller never waits on the console. Callable
 * from threads and ISRs. A trailing newline is optional, one is added.
 *
 * Returns false if the message was dropped because the buffer is full. The
 * flush thread reports how many messages were lost.
 */
bool viikate_log(const char *msg);

#ifdef __cplusplus
}

namespace viikate {
bool log(const char *msg) noexcept;
}  // namespace viikate
#endif

#endif
