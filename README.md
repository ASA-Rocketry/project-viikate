# Project Viikate

1. Clone this repository using `git clone https://github.com/ASA-Rocketry/Project-Viikate`.
2. Open the repository in your preferred editor or just `cd` into it.
3. Run `pip install pre-commit` to install the pre-commit hook manager.
4. Run `pre-commit install --hook-type commit-msg` to activate all the local checks.
5. Run `meson setup code/build code --wipe` to generate the project compilation commands.
6. Run `meson compile -C code/build` to compile the binary. The final binary will be in `code/build/firmware/firmware`.

## Structure

```
project-viikate/
  cad/          -- the fusion files for the structure of the rocket
  code/         -- firmware, simulations and other adjacent software
    firmware/   -- the actual firmware that goes on the rocket
    dashboard/
  docs/         -- textual documentation related to the operations of the project
  minutes/      -- meeting notes
  pcb/          -- electronics, pinouts and the core PCB
```

## Software

## Style Choices

- For details on how to style your code refer to [RocketStyle](./code/README.md).
- For details on how to style your mechanical designs refer to [Mechanical Design Guide for Aalto Space Association Projects with Fusion](./cad/README.md).
- Folder names should follow `kebab-case` unless mandated by the style guide of the specific piece of software.
- Files should follow `snake_case` unless mandated by the style guide of a specific piece of software.

## Testing Policy

The priority of tests is strictly in this order:

0. Structural failures.
1. Serialization/deserialization.
2. Unit conversions.
3. Sensor fusion edgecases.
4. State machine transitions.
5. Numerical stability.
6. Watchdog/fault recovery.

Notice how getters and setters are not included. They are not important.

## Reading test results

Unit tests live in [`tests/`](./tests) and run on your development machine,
not on the avionics board. CMake configures and builds them, CTest runs them:

```bash
just test
```

A green run is short — one line per test binary and the summary:

```
    Start 1: test_matmul
1/1 Test #1: test_matmul ......................   Passed    0.00 sec

100% tests passed, 0 tests failed out of 1
```

A red run prints the failing binary's whole utest output instead of the
`Passed` line, so the case level detail is there when you need it:

```
1/1 Test #1: test_matmul ......................***Failed    0.00 sec
[==========] Running 5 test cases.
[ RUN      ] matmul.vec4_product
/home/…/tests/test_matmul.c:85: Failure
  Expected : 71.000000
    Actual : 70.000000
[  FAILED  ] matmul.vec4_product (4629ns)
[==========] 5 test cases ran.
[  PASSED  ] 4 tests.
[  FAILED  ] 1 tests, listed below:
[  FAILED  ] matmul.vec4_product

0% tests passed, 1 tests failed out of 1

The following tests FAILED:
      1 - test_matmul (Failed)
```

How to read it:

- `[ RUN ]` starts a case, `[ OK ]` means every assertion in it held,
  `[ FAILED ]` means at least one did not. The time in brackets is how long
  the case took.
- A failed `ASSERT_*` prints the file and line of the assertion, then the two
  values: `Expected` is what the test wanted, `Actual` is what it got. The
  tolerance on these checks is `1e-4`.
- The `[  FAILED  ] … listed below:` block names every failing case, e.g.
  `[  FAILED  ] matmul.vec4_product`.
- `The following tests FAILED:` is CTest's own summary. CTest works per test
  binary, utest works per case — the binary entry tells you which file to
  open, the utest block tells you which case broke.
- `just test` exits `0` only when everything passed (CTest returns `8` on a
  failure), which is what CI keys off.

Anything after the recipe name is handed to utest through CMake's `TEST_ARGS`,
and CTest is then asked to show every case (`-V`), so selecting what to run
still prints the results:

```bash
just test --list-tests
just test --filter='matmul.identity_*'
```

Binaries are written to `build/tests/`, which is gitignored; `just clean`
removes it.

## How to make Git less painful

- Use rebases instead of merges. Merges pollute history, rebases straighten it back out.
- Don't commit stuff that is not supposed to be `diff`-ed. If you can't meaningfully interpret a file changing across time it should not be in git.
- Don't commit caches. Temporary files, output binaries, stuff like that.
- A commit message should follow the strict `type(scope): Message` format. This is needed to make the scope of the changes easier to understand. For example: `feat(gnc): Add the barrel roll logic`. The accepted types are as follows:
  - `feat` - a new feature, functionality that was previously missing.
  - `fix` - a fix to a bug that was causing trouble.
  - `style` - making some part of the code adhere to a style.
  - `docs` - adding documentation, amending typos.
  - `refactor` - making an old piece of code less ugly.
  - `chore` - reordering files, deleting stuff that shouldn't be tracked, etc.
- The commit text should read as what it does when applied. Not `added more tests`, but `add more tests`.
- Do not include OS-specific files in git. To exclude file locally you can add it to the `.git/info/exclude` file.
