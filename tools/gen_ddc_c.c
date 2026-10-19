/*
 * gen_ddc_c.c — Pure C DDC bitmap generator
 *
 * Reads a dataset file (int32 binary, values in [1..cardinality]) and writes
 * one DDC-C compressed .bm file per cardinality value.
 *
 * Usage:
 *   gen_ddc_c -n <rows> -c <cardinality> -i <dataset_file> -o <output_dir>
 *             [-S <segment_bits>] [-w <word_size>]
 *
 * Options:
 *   -n <rows>          Total number of rows in the dataset (required)
 *   -c <cardinality>   Number of distinct values / output bitmaps (required)
 *   -i <dataset_file>  Path to the binary int32 dataset file (required)
 *   -o <output_dir>    Directory to write <v>.bm files into (required)
 *   -S <segment_bits>  DDC segment size in bits; must be a power of 2 in
 *                      [64, 2^24].  Default: 65536 (DDC_C_DEFAULT_SEGMENT_BITS)
 *   -w <word_size>     Informational only (recorded in done.txt). Default: 8
 *
 * Output:
 *   <output_dir>/<v>.bm  for v = 1 .. cardinality
 *   <output_dir>/done.txt
 *
 * Build (standalone, no C++ required):
 *   /usr/bin/gcc -O3 -mavx512f -mavx512vbmi2 -funroll-loops \
 *     -I src/core/ddc/include_c \
 *     util/gen_ddc_c.c \
 *     src/core/ddc/src_c/ddc_c_base.c \
 *     src/core/ddc/src_c/ddc_c_ops.c \
 *     src/core/ddc/src_c/ddc_c_serial.c \
 *     src/core/ddc/src_c/ddc_c_sparse.c \
 *     -o build/gen_ddc_c
 */

#include "ddc_c.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

/* ---- portable mkdir -p -------------------------------------------------- */

static int mkdir_p(const char *path)
{
    char tmp[4096];
    size_t len = strlen(path);
    if (len == 0 || len >= sizeof(tmp)) return -1;
    memcpy(tmp, path, len + 1);
    for (size_t i = 1; i <= len; i++) {
        if (tmp[i] == '/' || tmp[i] == '\0') {
            char saved = tmp[i];
            tmp[i] = '\0';
#ifdef _WIN32
            _mkdir(tmp);
#else
            mkdir(tmp, 0755);
#endif
            tmp[i] = saved;
        }
    }
    return 0;
}

/* ---- write a complete buffer to a file ---------------------------------- */

static int write_file(const char *path, const uint8_t *data, size_t len)
{
    FILE *f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "[gen_ddc_c] Error: cannot open '%s' for writing: %s\n",
                path, strerror(errno));
        return -1;
    }
    size_t written = fwrite(data, 1, len, f);
    fclose(f);
    if (written != len) {
        fprintf(stderr, "[gen_ddc_c] Error: short write to '%s' (%zu/%zu bytes)\n",
                path, written, len);
        return -1;
    }
    return 0;
}

/* ---- helpers ------------------------------------------------------------ */

static int is_power_of_two(size_t v) { return v > 0 && (v & (v - 1)) == 0; }

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage: %s -n <rows> -c <cardinality> -i <dataset> -o <outdir>\n"
        "          [-S <segment_bits>] [-w <word_size>]\n"
        "\n"
        "  -n <rows>          Number of rows in the dataset (required)\n"
        "  -c <cardinality>   Number of distinct values / output bitmaps (required)\n"
        "  -i <dataset>       Binary int32 dataset file (required)\n"
        "  -o <outdir>        Output directory for .bm files (required)\n"
        "  -S <segment_bits>  Power of 2 in [64, 2^24]; default 65536\n"
        "  -w <word_size>     Informational, recorded in done.txt; default 8\n",
        prog);
}

/* ---- main --------------------------------------------------------------- */

int main(int argc, char *argv[])
{
    long long n_arg        = -1;
    int       c_arg        = -1;
    int       w_arg        = 8;
    long long seg_bits_arg = -1;
    const char *input_file = NULL;
    const char *output_dir = NULL;

    /* --- argument parsing --- */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
            n_arg = atoll(argv[++i]);
        } else if (strcmp(argv[i], "-c") == 0 && i + 1 < argc) {
            c_arg = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-i") == 0 && i + 1 < argc) {
            input_file = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (strcmp(argv[i], "-S") == 0 && i + 1 < argc) {
            seg_bits_arg = atoll(argv[++i]);
        } else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc) {
            w_arg = atoi(argv[++i]);
        } else {
            fprintf(stderr, "[gen_ddc_c] Unknown option: %s\n", argv[i]);
            usage(argv[0]);
            return 1;
        }
    }

    if (n_arg <= 0 || c_arg <= 0 || !input_file || !output_dir) {
        usage(argv[0]);
        return 1;
    }

    size_t num_rows    = (size_t)n_arg;
    int    cardinality = c_arg;

    size_t segment_bits = DDC_C_DEFAULT_SEGMENT_BITS;
    if (seg_bits_arg > 0) {
        if (!is_power_of_two((size_t)seg_bits_arg) ||
            (size_t)seg_bits_arg < 64 ||
            (size_t)seg_bits_arg > ((size_t)1 << 24)) {
            fprintf(stderr,
                "[gen_ddc_c] Error: -S must be a power of 2 in [64, 2^24], got %lld\n",
                seg_bits_arg);
            return 1;
        }
        segment_bits = (size_t)seg_bits_arg;
    }

    /* --- read dataset --- */
    fprintf(stderr, "[gen_ddc_c] Reading dataset '%s' (%zu rows, cardinality=%d)...\n",
            input_file, num_rows, cardinality);

    FILE *fin = fopen(input_file, "rb");
    if (!fin) {
        fprintf(stderr, "[gen_ddc_c] Error: cannot open '%s': %s\n",
                input_file, strerror(errno));
        return 1;
    }

    int32_t *data = (int32_t *)malloc(num_rows * sizeof(int32_t));
    if (!data) {
        fprintf(stderr, "[gen_ddc_c] Error: out of memory allocating dataset buffer\n");
        fclose(fin);
        return 1;
    }

    size_t nread = fread(data, sizeof(int32_t), num_rows, fin);
    fclose(fin);
    if (nread != num_rows) {
        fprintf(stderr,
            "[gen_ddc_c] Error: expected %zu int32 values, got %zu\n",
            num_rows, nread);
        free(data);
        return 1;
    }

    /* --- create output directory --- */
    if (mkdir_p(output_dir) != 0) {
        fprintf(stderr, "[gen_ddc_c] Error: cannot create output directory '%s'\n",
                output_dir);
        free(data);
        return 1;
    }

    fprintf(stderr, "[gen_ddc_c] Compressing %d bitmaps -> %s\n"
                    "[gen_ddc_c]   segment_bits=%zu  word_size=%d\n",
            cardinality, output_dir, segment_bits, w_arg);

    /* --- allocate shared bit array (one byte per logical bit, reused per v) --- */
    uint8_t *bits = (uint8_t *)calloc(num_rows, 1);
    if (!bits) {
        fprintf(stderr, "[gen_ddc_c] Error: out of memory allocating bit buffer\n");
        free(data);
        return 1;
    }

    /* serialise buffer: upper bound = ~2 bytes per compressed bit + header slop */
    size_t serial_cap = num_rows / 4 + (1 << 20);
    uint8_t *serial_buf = (uint8_t *)malloc(serial_cap);
    if (!serial_buf) {
        fprintf(stderr, "[gen_ddc_c] Error: out of memory allocating serialise buffer\n");
        free(bits);
        free(data);
        return 1;
    }

    int errors = 0;

    for (int v = 1; v <= cardinality; v++) {

        /* build bit array for value v */
        memset(bits, 0, num_rows);
        for (size_t i = 0; i < num_rows; i++) {
            if (data[i] == v)
                bits[i] = 1;
        }

        /* compress */
        ddc_c ddc;
        ddc_c_init(&ddc);
        ddc_c_status st = ddc_c_compress(&ddc, bits, num_rows,
                                          /*l1_fill_ones=*/0, segment_bits);
        if (st != DDC_C_OK) {
            fprintf(stderr,
                "[gen_ddc_c] Error: ddc_c_compress failed for v=%d (status=%d)\n",
                v, (int)st);
            ddc_c_destroy(&ddc);
            errors++;
            continue;
        }

        /* serialize — grow buffer if needed */
        size_t written = 0;
        st = ddc_c_serialize(&ddc, serial_buf, serial_cap, &written);
        if (st == DDC_C_BUFFER_TOO_SMALL) {
            /* query exact size */
            ddc_c_status st2 = ddc_c_serialize(&ddc, NULL, 0, &written);
            (void)st2;
            if (written > serial_cap) {
                uint8_t *nb = (uint8_t *)realloc(serial_buf, written);
                if (!nb) {
                    fprintf(stderr,
                        "[gen_ddc_c] Error: realloc failed for v=%d\n", v);
                    ddc_c_destroy(&ddc);
                    errors++;
                    continue;
                }
                serial_buf = nb;
                serial_cap = written;
            }
            st = ddc_c_serialize(&ddc, serial_buf, serial_cap, &written);
        }
        ddc_c_destroy(&ddc);

        if (st != DDC_C_OK) {
            fprintf(stderr,
                "[gen_ddc_c] Error: ddc_c_serialize failed for v=%d (status=%d)\n",
                v, (int)st);
            errors++;
            continue;
        }

        /* write .bm file */
        char out_path[4096];
        snprintf(out_path, sizeof(out_path), "%s/%d.bm", output_dir, v);
        if (write_file(out_path, serial_buf, written) != 0) {
            errors++;
            continue;
        }

        if (v % 100 == 0 || v == cardinality)
            fprintf(stderr, "[gen_ddc_c] Written %d/%d bitmaps\n", v, cardinality);
    }

    free(serial_buf);
    free(bits);
    free(data);

    if (errors > 0) {
        fprintf(stderr, "[gen_ddc_c] Completed with %d error(s).\n", errors);
        return 1;
    }

    /* write done.txt */
    char done_path[4096];
    snprintf(done_path, sizeof(done_path), "%s/done.txt", output_dir);
    FILE *done = fopen(done_path, "w");
    if (done) {
        fprintf(done, "algorithm=ddc\nn=%lld\nc=%d\nw=%d\nS=%zu\n",
                (long long)num_rows, cardinality, w_arg, segment_bits);
        fclose(done);
    }

    fprintf(stderr,
        "[gen_ddc_c] Done. %d bitmaps written to '%s'\n",
        cardinality, output_dir);
    return 0;
}
