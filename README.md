# DDC-C

DDC-C is a standalone **C11** implementation of **DDC (Decoupling Data and Control)** bitmap compression. It implements the standard four-level `DDCBtv` format, segmented `DDC` containers, and `SparseDDC`, and is intended as a drop-in C replacement for the reference C++ DDC library.

The library depends only on the C standard library. It does not link against or call the C++ implementation. On-disk formats and bit ordering are kept compatible with the reference C++ DDC where the corresponding format is supported.

## Relationship to the reference C++ implementation

The reference DDC implementation is a separate C++ project:

- **Upstream repository:** <https://github.com/junchangwang/DDC>
- **Project:** DDC — Decoupling Data and Control for Bitmap Compression

DDC-C is an independent C11 reimplementation of that project's standard four-level format. The production library is pure C and has no build-time or run-time dependency on the C++ code.

This repository contains **two optional C++ files**, used only to validate the C implementation against the reference implementation:

| File | Purpose | Built by `make all`? |
|---|---|---|
| `test/ddc_c_cpp_parity.cpp` | Compares C and C++ behavior, state, compression size, and wire formats | No |
| `tools/ddc_c_cpp_boolean_bench.cpp` | Side-by-side C/C++ benchmark for boolean operations | No |

Both files `#include "ddc.h"` from the upstream C++ project and therefore require a local checkout of <https://github.com/junchangwang/DDC>; see [C/C++ parity harness](#cc-parity-harness-optional). They are validation tooling, not part of the library. No upstream C++ source code is vendored in this repository. The upstream repository did not declare a license at the time of writing, so its code is not redistributed here.

## Highlights

- Exact four-level DDC encoding (`DDCBtv`) with segmented containers
- AND / OR / XOR / NOT, both out-of-place and in-place
- Multi-way OR and sparse-bitmap construction
- Serialization formats: V1/V2, packed V3, and V4
- Optional AVX-512F/BW/VBMI2 fast paths with scalar fallback
- Streaming traversal of set bits and literal words
- No third-party dependencies

## Requirements

- Linux (x86-64 recommended)
- A C11 compiler; the bundled `Makefile` uses `/usr/bin/gcc`
- GNU Make

AVX-512 support is optional and is selected at compile time. The Makefile uses `-march=native`, so build artifacts are tuned for the build host; use a generic `-march` if you need portable binaries.

## Build

```sh
make all
```

This produces:

| Output | Description |
|---|---|
| `build/ddc_c_selftest` | Self-test |
| `build/gen_ddc_c` | Bitmap generator |
| `build/bench_ops_w8_c` | Boolean-operation benchmark |

Run the self-test:

```sh
./build/ddc_c_selftest
# ddc_c self-test passed
```

To remove build outputs:

```sh
make clean
```

## Quick start

```c
#include <stdio.h>
#include <stdlib.h>

#include "ddc_c.h"

int main(void) {
    enum { N = 100000 };
    uint8_t *bits = calloc(N, 1);
    if (bits == NULL) return 1;

    /* One byte per logical bit: zero is clear, non-zero is set. */
    for (int i = 0; i < N; ++i)
        bits[i] = (uint8_t)(i % 7 == 0);

    ddc_c d;
    ddc_c_init(&d);
    if (ddc_c_compress(&d, bits, N, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK)
        return 1;

    printf("set bits: %zu\n", ddc_c_popcount(&d));

    /* N bytes is a safe upper bound for N logical bits. */
    uint8_t *wire = malloc(N);
    size_t written = 0;
    if (ddc_c_serialize(&d, wire, N, &written) != DDC_C_OK)
        return 1;
    printf("serialized: %zu bytes\n", written);

    ddc_c restored;
    ddc_c_init(&restored);
    size_t consumed = 0;
    if (ddc_c_deserialize(&restored, wire, written, &consumed) != DDC_C_OK)
        return 1;

    uint8_t *roundtrip = calloc(N, 1);
    if (ddc_c_decompress(&restored, roundtrip, N) != DDC_C_OK)
        return 1;

    int same = 1;
    for (int i = 0; i < N; ++i)
        same &= (roundtrip[i] != 0) == (bits[i] != 0);

    printf("roundtrip: %s\n", same ? "ok" : "MISMATCH");

    free(roundtrip);
    free(wire);
    ddc_c_destroy(&restored);
    ddc_c_destroy(&d);
    free(bits);
    return same ? 0 : 1;
}
```

Compile it together with the four core sources:

```sh
/usr/bin/gcc -O2 -std=gnu11 -Wall -Wextra -Iinclude example.c \
    src/ddc_c_base.c src/ddc_c_ops.c src/ddc_c_serial.c src/ddc_c_sparse.c \
    -o example
./example
# set bits: 14286
# serialized: 12687 bytes
# roundtrip: ok
```

## API overview

The complete public interface is in [`include/ddc_c.h`](include/ddc_c.h).

| Area | Representative functions |
|---|---|
| Lifecycle | `ddc_c_init`, `ddc_c_destroy`, `ddc_c_btv_init`, `ddc_c_sparse_init` |
| Construction | `ddc_c_compress`, `ddc_c_btv_compress`, `ddc_c_from_sparse_positions` |
| Decoding | `ddc_c_decompress`, `ddc_c_btv_decompress` |
| Boolean ops | `ddc_c_and`, `ddc_c_or`, `ddc_c_xor`, `ddc_c_not`, `*_inplace` variants |
| Multi-way / sparse | `ddc_c_or_many`, `ddc_c_sparse_from_positions`, `ddc_c_sparse_or_many` |
| Queries | `ddc_c_popcount`, `ddc_c_popcount_and`, `ddc_c_is_all_zero`, `ddc_c_compression_ratio` |
| Traversal | `ddc_c_for_each_literal`, `ddc_c_for_each_set_bit`, `ddc_c_set_bit_positions` |
| Serialization | `ddc_c_serialize` / `ddc_c_deserialize`, `_packed`, `_v4`, `_trusted` |

Behavioral notes:

- Binary operations require the output object not to alias either input.
- `ddc_c_set_compress_results` is a process-global switch controlling whether boolean results are stored compressed. It is not thread-safe; set it during initialization.
- The serializer writes into a caller-provided buffer. Size the buffer according to your data; in the worst case, `bit_count` bytes is a safe bound.
- The `_trusted` deserializers skip validation and should only be used on data produced by the same library.
- Separate library objects can be used concurrently; the library keeps no mutable global state other than the `compress_results` switch.
- The native wire formats use fixed-width integers and do not promise cross-endian compatibility.

## Tools

### `gen_ddc_c`

Generates one compressed `.bm` file per value from a binary `int32` dataset. Input values must be in `[1, cardinality]`.

```sh
./build/gen_ddc_c -n <rows> -c <cardinality> -i <dataset.bin> -o <outdir> \
    [-S <segment_bits>] [-w <word_size>]
```

- `-n` — number of rows in the dataset
- `-c` — number of distinct values (one output bitmap per value)
- `-i` — input file containing `int32` values
- `-o` — output directory for `<v>.bm` files and `done.txt`
- `-S` — segment size in bits, a power of two in `[64, 2^24]` (default `65536`)
- `-w` — informational only; recorded in `done.txt` (default `8`)

### `bench_ops_w8_c`

Measures load, pairwise boolean operations, NOT, and multi-way OR with 5-iteration medians over a directory of `.bm` files.

```sh
./build/bench_ops_w8_c <bitmap_root_dir> <out_csv> [cardinality] [--raw-csv <raw_csv>]
```

Optional arguments restrict the run to one cardinality and emit per-iteration raw samples.

### Deserialization fuzzer

[`test/ddc_c_deserialize_fuzz.c`](test/ddc_c_deserialize_fuzz.c) mutates serialized bitmaps and checks that malformed input is rejected without crashing. It is not part of `make all`.

```sh
/usr/bin/gcc -O2 -std=gnu11 -Iinclude test/ddc_c_deserialize_fuzz.c \
    src/ddc_c_base.c src/ddc_c_ops.c src/ddc_c_serial.c src/ddc_c_sparse.c \
    -o build/ddc_c_deserialize_fuzz
./build/ddc_c_deserialize_fuzz [iterations]
```

### C/C++ parity harness (optional)

[`test/ddc_c_cpp_parity.cpp`](test/ddc_c_cpp_parity.cpp) compares the C and C++ implementations for logic, state, compression size, and wire formats. It links against the **reference C++ DDC project** at <https://github.com/junchangwang/DDC>, which must be checked out separately; it is not needed to build or use the C library.

```sh
# 1. Check out the reference C++ implementation somewhere outside this repo.
git clone https://github.com/junchangwang/DDC.git /path/to/DDC

# 2. Compile the C sources as C, then link the harness with the C++ sources.
for f in base ops serial sparse; do
    /usr/bin/gcc -O3 -march=native -std=gnu11 -Iinclude \
        -c src/ddc_c_$f.c -o /tmp/ddc_c_$f.o
done

/usr/bin/g++ -O3 -march=native -std=c++17 \
    -I/path/to/DDC/src/core/ddc/include -Iinclude \
    test/ddc_c_cpp_parity.cpp /tmp/ddc_c_*.o \
    /path/to/DDC/src/core/ddc/src/{ddc,and,or,xor,not}.cpp \
    -o build/ddc_c_cpp_parity
```

`tools/ddc_c_cpp_boolean_bench.cpp` is the corresponding C/C++ benchmark and has the same external prerequisite. Use `--quick` for a smoke run, or `--help` for the full matrix and profiling options.

## Repository layout

```text
include/ddc_c.h                    Public API
src/ddc_c_base.c                   Lifecycle, compression, decoding, queries
src/ddc_c_ops.c                    Boolean operations and SIMD kernels
src/ddc_c_serial.c                 V1/V2, V3 (packed), and V4 serialization
src/ddc_c_sparse.c                 Sparse DDC operations
src/ddc_c_internal.h               Internal helpers and SIMD gates
test/ddc_c_selftest.c              Self-test
test/ddc_c_deserialize_fuzz.c      Malformed-input fuzzer
test/ddc_c_cpp_parity.cpp          Optional parity test vs. upstream C++ DDC; not built by default
tools/gen_ddc_c.c                  Bitmap generator
tools/bench_ops_w8_c.c             Boolean-operation benchmark
tools/ddc_c_cpp_boolean_bench.cpp  Optional C/C++ benchmark vs. upstream C++ DDC; not built by default
Makefile                           Standalone build
```

## License

No license has been selected yet. Until a license file is added, all rights are reserved by the copyright holders.
