// Test-only shell commands for the logger. Not part of the flight image.
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include "logger.h"

// Fixed padding so lines have a realistic length.
#define FILL_PADDING "0123456789abcdefghijklmnopqrstuvwxyz"

// vlog fill <tag> <count>: log "fill <tag> <i> <padding>" for i in
// 0..count-1, waiting for room whenever the buffer is full so no line is lost.
static int cmd_fill(const struct shell *sh, size_t argc, char **argv) {
  ARG_UNUSED(argc);
  const char *tag = argv[1];
  long count = strtol(argv[2], NULL, 10);

  for (long i = 0; i < count; ++i) {
    while (!log_info("fill", tag, i, FILL_PADDING)) {
      k_msleep(5);
    }
  }
  shell_print(sh, "filled %ld", count);
  return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_vlog,
                               SHELL_CMD_ARG(fill, NULL, "fill <tag> <count>",
                                             cmd_fill, 3, 0),
                               SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(vlog, &sub_vlog, "Logger test commands", NULL);
