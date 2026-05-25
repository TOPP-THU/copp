# COPP C Bindings

## Convex-Objective Path Parameterization

This directory contains the public C ABI for the open-source COPP library. It focuses on using COPP from C through stable headers, ordinary C data types, and explicit ownership rules.

The C ABI is in its feedback and stabilization phase. It is intended to be usable today, but function names, problem descriptors, result structs, and packaging details may still evolve before the first stable C release. Feedback from downstream bindings, robotics applications, and packaging workflows is welcome; if you need a specific API shape or compatibility guarantee, please see [Contact Us](../../README.md#contact-us).

> **Open-source / PRO note:** this README documents the open-source C ABI. The repository root [README](../../README.md) is the single source of truth for the open-source vs PRO algorithm matrix, solver-selection guidance, benchmark comparisons, citation information, and collaboration contact details.

The C API follows a deliberately small set of rules:

- include `copp/copp.h` for the full API, or a narrower header such as `copp/robot.h`, `copp/topp2.h`, or `copp/copp3.h`;
- most functions return `enum CoppStatus`;
- borrowed slices and matrix views are used only during the call;
- COPP-owned outputs are released with the matching `copp_*_free` function;
- matrices support column-major, row-major, and strided views, with contiguous column-major as the zero-copy fast path.

The algorithmic background, open-source algorithm availability, benchmark tables, and citation information live in the repository root README. This page focuses on building, linking, installing, and using the C ABI.

## API Availability

| Problem class  | Public C API                                                                           |
| -------------- | -------------------------------------------------------------------------------------- |
| Core utilities | status, last error, slices, matrix views, owned vectors/matrices                       |
| Path           | waypoint spline paths, evaluator paths, metadata, 2nd/3rd derivative evaluation        |
| Robot          | station grids, path sampling, raw constraints, axial limits, inverse dynamics callback |
| TOPP2          | TOPP2-RA, ReachSet2, 2nd-order interpolation                                           |
| COPP2          | COPP2-SOCP                                                                             |
| TOPP3          | TOPP3-LP, TOPP3-SOCP, 3rd-order interpolation                                          |
| COPP3          | COPP3-SOCP                                                                             |

Runnable examples are in [examples](examples/). They are built as CMake targets named `example_*`.

## Quick Start

Prebuilt C ABI SDK packages for the latest tagged release are attached to the repository's GitHub Releases. If you want to build COPP from source instead, follow the steps below.

### Prerequisites

You need:

- Cargo
- CMake
- a C compiler toolchain
- `cbindgen` if you need to refresh generated headers

Install `cbindgen` once:

```sh
cargo install cbindgen
```

### Build the Native Library

Run from the repository root:

```sh
cargo build --release
```

Cargo produces the C ABI libraries under:

```text
target/release/
```

Typical artifact names are:

| Platform | Dynamic                     | Static      |
| -------- | --------------------------- | ----------- |
| Windows  | `copp.dll` + `copp.dll.lib` | `copp.lib`  |
| Linux    | `libcopp.so`                | `libcopp.a` |
| macOS    | `libcopp.dylib`             | `libcopp.a` |

The public C symbols and headers use the `copp` prefix.

### Refresh Headers

Headers are generated from `src/ffi/c` and split into:

```text
bindings/c/include/copp/
```

Windows:

```powershell
powershell -ExecutionPolicy Bypass -File bindings/c/scripts/generate_headers.ps1
```

Linux / macOS:

```sh
pwsh -File bindings/c/scripts/generate_headers.ps1
```

The checked-in headers can be used directly when you do not need to regenerate them.

### Build Examples and Smoke Tests

Dynamic linking:

```sh
cmake -S bindings/c -B bindings/c/build
cmake --build bindings/c/build --config Release
```

Static linking:

```sh
cmake -S bindings/c -B bindings/c/build -DCOPP_LINK_STATIC=ON
cmake --build bindings/c/build --config Release
```

Run C tests:

```sh
ctest --test-dir bindings/c/build --output-on-failure -C Release
```

Register examples as CTest tests:

```sh
cmake -S bindings/c -B bindings/c/build -DCOPP_TEST_EXAMPLES=ON
cmake --build bindings/c/build --config Release
ctest --test-dir bindings/c/build --output-on-failure -C Release
```

Build one example target:

```sh
cmake --build bindings/c/build --config Release --target example_topp3_lp
```

Disable tests or examples when configuring:

```sh
cmake -S bindings/c -B bindings/c/build -DCOPP_BUILD_TESTS=OFF
cmake -S bindings/c -B bindings/c/build -DCOPP_BUILD_EXAMPLES=OFF
```

## Install and Link

The CMake install step copies public headers, the native library, and a package config:

```text
<prefix>/
  include/copp/*.h
  lib/...
  bin/...                 # Windows dynamic DLL only
  lib/cmake/copp/...
```

Dynamic install:

```sh
cmake -S bindings/c -B bindings/c/build
cmake --build bindings/c/build --config Release
cmake --install bindings/c/build --config Release --prefix <install-prefix>
```

Static install:

```sh
cmake -S bindings/c -B bindings/c/build -DCOPP_LINK_STATIC=ON
cmake --build bindings/c/build --config Release
cmake --install bindings/c/build --config Release --prefix <install-prefix>
```

Example prefixes:

```powershell
cmake --install bindings/c/build --config Release --prefix C:/libs/copp
```

```sh
cmake --install bindings/c/build --config Release --prefix "$HOME/.local"
```

Downstream projects can then use:

```cmake
find_package(copp CONFIG REQUIRED)

add_executable(app main.c)
target_link_libraries(app PRIVATE copp::copp)
```

Configure the downstream project with:

```sh
cmake -S app -B app/build -DCMAKE_PREFIX_PATH=<install-prefix>
cmake --build app/build --config Release
```

On Windows with dynamic linking, the package target records both the import library and the runtime DLL. The DLL still needs to be next to the executable or visible through `PATH` at run time.

For static packages, the exported `copp::copp` target includes common platform native libraries. If a platform or toolchain needs extra native libraries, configure COPP with:

```sh
cmake -S bindings/c -B bindings/c/build -DCOPP_LINK_STATIC=ON -DCOPP_STATIC_LINK_LIBRARIES="lib1;lib2"
```

### Package Smoke Test

The optional install test installs COPP into a temporary prefix, configures a tiny downstream project with `find_package(copp CONFIG REQUIRED)`, builds it, and runs it.

Dynamic package test:

```sh
cmake -S bindings/c -B bindings/c/build -DCOPP_BUILD_INSTALL_TESTS=ON
cmake --build bindings/c/build --config Release
ctest --test-dir bindings/c/build --output-on-failure -C Release -R "test_install_copp_package|test_find_package_copp"
```

Static package test:

```sh
cmake -S bindings/c -B bindings/c/build -DCOPP_LINK_STATIC=ON -DCOPP_BUILD_INSTALL_TESTS=ON
cmake --build bindings/c/build --config Release
ctest --test-dir bindings/c/build --output-on-failure -C Release -R "test_install_copp_package|test_find_package_copp"
```

## General Workflow

Most C examples follow the same shape:

1. Build a path from waypoints or evaluator callbacks.
2. Build a station grid.
3. Create a `CoppRobot`.
4. Append stations and sample path derivatives into the robot.
5. Add velocity, acceleration, jerk, torque, or raw constraints.
6. Build a borrowed `Topp*Problem` or `Copp*Problem` descriptor.
7. Call a solver.
8. Convert solver output to timing data with interpolation helpers.
9. Release COPP-owned outputs with the matching free function.

The C API intentionally avoids implementation-specific lifetimes, generics, closures, and builders. Problem structs are borrowed descriptors used only during the solver call.

## Minimal Program

```c
#include <stdio.h>

#include "copp/copp.h"

int main(void) {
    printf("COPP version: %s\n", copp_version());
    printf("OK means: %s\n", copp_status_message(COPP_STATUS_OK));
    return 0;
}
```

## Error Handling

Most C ABI functions return `enum CoppStatus`. `COPP_STATUS_OK` means success; other values are stable machine-readable failure classes.

Use `copp_status_message(status)` for a short static description. For detailed call-specific diagnostics, use the thread-local last-error API:

```c
size_t len = 0;
enum CoppStatus status = copp_robot_len(NULL, &len);
if (status != COPP_STATUS_OK) {
    fprintf(stderr, "%s\n", copp_last_error_message());
}
```

Successful C ABI calls clear the thread-local last-error slot. Failed calls set it to a UTF-8 message capped at 64 KiB. The pointer returned by `copp_last_error_message()` is owned by COPP and remains valid until the next COPP C ABI call on the same thread updates or clears it.

Wrappers that need ownership should copy the message:

```c
size_t msg_len = 0;
copp_last_error_message_copy(NULL, 0, &msg_len);

char *buffer = malloc(msg_len + 1);
copp_last_error_message_copy(buffer, msg_len + 1, NULL);
```

Callbacks can attach details before returning an error:

```c
copp_set_last_error_message(
    COPP_STATUS_ROBOT_DYNAMICS_ERROR,
    "inverse dynamics failed at station 42");
return COPP_STATUS_ROBOT_DYNAMICS_ERROR;
```

## Documentation

The C API reference is generated from the public headers under:

```text
bindings/c/include/copp/
```

Install Doxygen:

Windows:

```powershell
winget install -e --id DimitriVanHeesch.Doxygen
winget install -e --id Graphviz.Graphviz
```

Ubuntu / Debian:

```sh
sudo apt update
sudo apt install doxygen graphviz
```

macOS:

```sh
brew install doxygen graphviz
```

Generate docs:

Windows:

```powershell
powershell -ExecutionPolicy Bypass -File bindings/c/scripts/generate_docs.ps1
```

Linux / macOS:

```sh
sh bindings/c/scripts/generate_docs.sh
```

Open:

```text
bindings/c/docs/html/index.html
```

The generated `bindings/c/docs/` directory is ignored by Git.

## Header Layout

| Header                 | Contents                                                |
| ---------------------- | ------------------------------------------------------- |
| `copp/copp.h`          | Umbrella header                                         |
| `copp/core.h`          | status, last error, matrices, vectors, Clarabel options |
| `copp/path.h`          | path handles and path evaluation                        |
| `copp/robot.h`         | robot handles, sampling, constraints, callbacks         |
| `copp/formulation.h`   | problem descriptors, objectives, profiles               |
| `copp/interpolation.h` | 2nd/3rd-order interpolation utilities                   |
| `copp/topp2.h`         | TOPP2-RA and ReachSet2                                  |
| `copp/copp2.h`         | COPP2-SOCP                                              |
| `copp/topp3.h`         | TOPP3-LP and TOPP3-SOCP                                 |
| `copp/copp3.h`         | COPP3-SOCP                                              |

## Examples

Example sources:

```text
bindings/c/examples/
```

Available example targets:

```text
example_topp2_ra
example_reach_set2
example_copp2_socp
example_topp3_lp
example_topp3_socp
example_copp3_socp
```

Algorithms not listed in the open-source availability table in the repository root README are not documented as part of the open-source C ABI.

## Troubleshooting

### Header Generation Fails

Install `cbindgen`, then run the header generation command from the repository root. If you do not need to change the ABI, use the checked-in headers.

### CMake Cannot Find the COPP Library

Run:

```sh
cargo build --release
```

The source-tree CMake flow expects the native library artifact under `target/release/`.

### Downstream `find_package` Fails

Make sure the downstream project sees the install prefix:

```sh
cmake -S app -B app/build -DCMAKE_PREFIX_PATH=<install-prefix>
```

### Runtime DLL Is Missing on Windows

For dynamic linking, ensure `copp.dll` is next to your executable or visible through `PATH`.

### Static Linking Fails With Missing Native Symbols

Pass additional native libraries through:

```sh
cmake -S bindings/c -B bindings/c/build -DCOPP_LINK_STATIC=ON -DCOPP_STATIC_LINK_LIBRARIES="lib1;lib2"
```
