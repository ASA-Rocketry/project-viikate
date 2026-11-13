// Test-only shell commands for the logger. Not part of the flight image.
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include "logger.h"

namespace {
// Fixed padding so lines have a realistic length.
constexpr const char *FILL_PADDING = "0123456789abcdefghijklmnopqrstuvwxyz";

// vlog fill <tag> <count>: log "fill <tag> <i> <padding>" for i in
// 0..count-1, waiting for room whenever the buffer is full so no line is lost.
int cmd_fill(const shell *sh, size_t, char **argv) {
  const char *tag = argv[1];
  long count = strtol(argv[2], nullptr, 10);

  for (long i = 0; i < count; ++i) {
    while (!viikate::log("fill", tag, i, FILL_PADDING)) {
      k_msleep(5);
    }
  }
  shell_print(sh, "filled %ld", count);
  return 0;
}
} // namespace

SHELL_STATIC_SUBCMD_SET_CREATE(sub_vlog,
                               SHELL_CMD_ARG(fill, NULL, "fill <tag> <count>",
                                             cmd_fill, 3, 0),
                               SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(vlog, &sub_vlog, "Logger test commands", NULL);
