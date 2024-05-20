# Fuzzing setup

There's an `AFL++` and a `libfuzzer` version for building.

## Requirements

- AFL++
- cmake
- make
- clang
- (prometheus-cpp)

## How to

```bash
cd fuzz/
# Running `make` will produce a ./harness binary that can be run as is
make
# Alternatively:
make afl
# This will produce a ./harness_afl that can be executed the AFL++ way
```
