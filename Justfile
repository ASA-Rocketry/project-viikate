test *args:
    #!/usr/bin/env bash
    set -euo pipefail
    cmake -S tests -B build/tests -G Ninja -DTEST_ARGS='{{args}}'
    cmake --build build/tests
    if [ -n "{{args}}" ]; then
        ctest --test-dir build/tests -V
    else
        ctest --test-dir build/tests --output-on-failure
    fi

clean:
    rm -rf build/tests tests/build
