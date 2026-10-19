/*
 * bench_ops_w8_c.c - C equivalent of bench_ops_w8.cpp for ddc_c.
 *
 * Reads the same bm_100m_cN_ddc_w8 directories and measures DDC-C load,
 * pairwise boolean operations, NOT, and multi-way OR with 5-iteration medians.
 *
 * The first six data columns keep the original bench_ops_w8-compatible shape:
 *   cardinality,and_ms,or_ms,xor_ms,andnot_ms,not_ms
 */

#include "ddc_c.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

/* ---------- timing ---------- */

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6;
}

/* ---------- median of 5 ---------- */

static int cmp_double(const void *a, const void *b) {
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

static double median5(double v[5]) {
    double tmp[5];
    memcpy(tmp, v, sizeof tmp);
    qsort(tmp, 5, sizeof(double), cmp_double);
    return tmp[2];
}

/* ---------- .bm file helpers ---------- */

typedef struct {
    char *path;
    int num;
} bm_entry;

static int cmp_bm_entry(const void *a, const void *b) {
    const bm_entry *x = (const bm_entry *)a;
    const bm_entry *y = (const bm_entry *)b;
    return (x->num > y->num) - (x->num < y->num);
}

static char *make_path(const char *dir, const char *name) {
    size_t dir_len = strlen(dir);
    size_t name_len = strlen(name);
    char *path = (char *)malloc(dir_len + 1u + name_len + 1u);
    if (path == NULL) return NULL;
    memcpy(path, dir, dir_len);
    path[dir_len] = '/';
    memcpy(path + dir_len + 1u, name, name_len + 1u);
    return path;
}

static void free_bm_entries(bm_entry *entries, size_t count) {
    size_t i;
    if (entries == NULL) return;
    for (i = 0; i < count; ++i) free(entries[i].path);
    free(entries);
}

static int append_bm_entry(bm_entry **entries, size_t *count, size_t *cap,
                           const char *dir, const char *name, int num) {
    bm_entry *next;
    if (*count == *cap) {
        size_t new_cap = *cap == 0 ? 64u : *cap * 2u;
        next = (bm_entry *)realloc(*entries, new_cap * sizeof(**entries));
        if (next == NULL) return 0;
        *entries = next;
        *cap = new_cap;
    }
    (*entries)[*count].path = make_path(dir, name);
    if ((*entries)[*count].path == NULL) return 0;
    (*entries)[*count].num = num;
    ++(*count);
    return 1;
}

static int collect_bm_files(const char *dir, bm_entry **out_entries,
                            size_t *out_count) {
    DIR *d;
    struct dirent *ent;
    bm_entry *entries = NULL;
    size_t count = 0, cap = 0;

    if (out_entries == NULL || out_count == NULL) return 0;
    *out_entries = NULL;
    *out_count = 0;

    d = opendir(dir);
    if (!d) return 0;

    while ((ent = readdir(d)) != NULL) {
        const char *name = ent->d_name;
        size_t len = strlen(name);
        if (len > 3 && strcmp(name + len - 3, ".bm") == 0) {
            char stem[256];
            char *end;
            long num;
            if (len - 3 >= sizeof stem) continue;
            memcpy(stem, name, len - 3);
            stem[len - 3] = '\0';
            num = strtol(stem, &end, 10);
            if (*end == '\0') {
                if (!append_bm_entry(&entries, &count, &cap, dir, name, (int)num)) {
                    closedir(d);
                    free_bm_entries(entries, count);
                    return 0;
                }
            }
        }
    }
    closedir(d);

    qsort(entries, count, sizeof(bm_entry), cmp_bm_entry);
    *out_entries = entries;
    *out_count = count;
    return 1;
}

static size_t file_size_bytes(const char *path) {
    struct stat st;
    if (stat(path, &st) != 0 || st.st_size < 0) return 0;
    return (size_t)st.st_size;
}

static size_t total_file_size_bytes(const bm_entry *entries, size_t count) {
    size_t i, total = 0;
    for (i = 0; i < count; ++i) total += file_size_bytes(entries[i].path);
    return total;
}

/* ---------- load .bm files via ddc_c_deserialize ---------- */

static int load_bm(const char *path, ddc_c *out) {
    int fd;
    struct stat stbuf;
    void *mapped;
    size_t consumed = 0;
    ddc_c_status st;

    fd = open(path, O_RDONLY);
    if (fd < 0) {
        fprintf(stderr, "cannot open %s: %s\n", path, strerror(errno));
        return 0;
    }
    if (fstat(fd, &stbuf) != 0 || stbuf.st_size <= 0) {
        close(fd);
        return 0;
    }

    mapped = mmap(NULL, (size_t)stbuf.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
    close(fd);
    if (mapped == MAP_FAILED) return 0;

    st = ddc_c_deserialize_trusted(out, (const uint8_t *)mapped,
                           (size_t)stbuf.st_size, &consumed);
    munmap(mapped, (size_t)stbuf.st_size);
    return st == DDC_C_OK && consumed == (size_t)stbuf.st_size;
}

static void destroy_bitmaps(ddc_c *bitmaps, size_t count) {
    size_t i;
    if (bitmaps == NULL) return;
    for (i = 0; i < count; ++i) ddc_c_destroy(&bitmaps[i]);
    free(bitmaps);
}

static int load_all_bms(const bm_entry *entries, size_t count,
                        ddc_c **out_bitmaps) {
    size_t i;
    ddc_c *bitmaps;

    if (out_bitmaps == NULL) return 0;
    *out_bitmaps = NULL;
    if (count == 0) return 0;

    bitmaps = (ddc_c *)calloc(count, sizeof(*bitmaps));
    if (bitmaps == NULL) return 0;
    for (i = 0; i < count; ++i) ddc_c_init(&bitmaps[i]);

    for (i = 0; i < count; ++i) {
        if (!load_bm(entries[i].path, &bitmaps[i])) {
            fprintf(stderr, "load failed: %s\n", entries[i].path);
            destroy_bitmaps(bitmaps, count);
            return 0;
        }
    }
    *out_bitmaps = bitmaps;
    return 1;
}

/* ---------- DDC-C benchmark helpers ---------- */

static int compute_storage_summary(const ddc_c *bitmaps, size_t count,
                                   ddc_c_size_info *out) {
    size_t i;
    if (out == NULL) return 0;
    memset(out, 0, sizeof(*out));
    for (i = 0; i < count; ++i) {
        ddc_c_size_info part;
        ddc_c_size_breakdown(&bitmaps[i], &part);
        out->l3_bits += part.l3_bits;
        out->l4_bits += part.l4_bits;
        out->l3_literal_bits += part.l3_literal_bits;
        out->l2_literal_bits += part.l2_literal_bits;
        out->l1_literal_bits += part.l1_literal_bits;
        out->total_bits += part.total_bits;
    }
    return 1;
}

typedef ddc_c_status (*ddc_c_binary_op)(ddc_c *, const ddc_c *, const ddc_c *);

/* Summary uses the native standalone DDC-C API and caller-owned result
 * values. Unlike C++'s production backend path, DDC-C has no required outer
 * handle abstraction. */
static int run_binary_summary(ddc_c_binary_op op, const ddc_c *a, const ddc_c *b,
                              ddc_c *out_result) {
    if (op == NULL || a == NULL || b == NULL || out_result == NULL) return 0;
    return op(out_result, a, b) == DDC_C_OK;
}

static int run_multi_or_summary(const ddc_c *bitmaps, size_t count,
                                ddc_c *out_result) {
    size_t k;
    ddc_c tmp;
    const ddc_c **ptrs;

    if (count < 2 || out_result == NULL) return 0;

    /* build pointer array for ddc_c_or_many (avoids O(count) intermediate allocs) */
    ptrs = (const ddc_c **)malloc(count * sizeof(const ddc_c *));
    if (ptrs == NULL) return 0;
    for (k = 0; k < count; ++k) ptrs[k] = &bitmaps[k];

    ddc_c_init(&tmp);
    if (ddc_c_or_many(&tmp, count, ptrs) != DDC_C_OK) {
        ddc_c_destroy(&tmp);
        free(ptrs);
        return 0;
    }
    free(ptrs);
    ddc_c_destroy(out_result);
    *out_result = tmp;
    return 1;
}

static double bits_to_mb(size_t bits) {
    return (double)bits / 8.0 / 1024.0 / 1024.0;
}

static void write_raw_sample(FILE *raw, const char *mode, int cardinality,
                             int iteration, const char *operation,
                             double time_ms, size_t result_cardinality,
                             size_t loaded_count) {
    if (raw == NULL) return;
    fprintf(raw, "ddc-c-lifecycle-v1,ddc_c,%s,%d,%d,%s,%.6f,%zu,%zu\n",
            mode, cardinality, iteration, operation, time_ms,
            result_cardinality, loaded_count);
}

/* ---------- main ---------- */

int main(int argc, char **argv) {
    int cards[] = {2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000,
                   3000, 4000, 5000, 8000, 10000, 20000, 50000};
    int n_cards = (int)(sizeof cards / sizeof cards[0]);
    const int N_ITER = 5;
    const char *root;
    const char *out_path;
    const char *raw_path = NULL;
    int only_card = 0;
    FILE *out;
    FILE *raw = NULL;
    int ci;

    if (argc < 3 || argc > 6) {
        fprintf(stderr,
                "Usage: bench_ops_w8_c <bitmap_root_dir> <out_csv> "
                "[cardinality] [--raw-csv <raw_csv>]\n");
        return 1;
    }
    root = argv[1];
    out_path = argv[2];
    for (ci = 3; ci < argc; ++ci) {
        if (strcmp(argv[ci], "--raw-csv") == 0) {
            if (++ci >= argc || raw_path != NULL) {
                fprintf(stderr, "invalid --raw-csv argument\n");
                return 1;
            }
            raw_path = argv[ci];
        } else {
            char *end = NULL;
            long parsed;
            if (only_card != 0) {
                fprintf(stderr, "unexpected argument: %s\n", argv[ci]);
                return 1;
            }
            parsed = strtol(argv[ci], &end, 10);
            if (end == argv[ci] || *end != '\0' || parsed <= 0 || parsed > 50000) {
                fprintf(stderr, "invalid cardinality: %s\n", argv[ci]);
                return 1;
            }
            only_card = (int)parsed;
        }
    }

    out = fopen(out_path, "w");
    if (!out) {
        fprintf(stderr, "cannot open %s\n", out_path);
        return 1;
    }
    fprintf(out,
            "cardinality,direct_and_ms,direct_or_ms,direct_xor_ms,"
            "direct_andnot_ms,direct_not_ms,summary_load_ms,loaded_count,"
            "density,summary_multi_or_ms,summary_multi_or_card,l4_mb,l3_mb,"
            "l2_mb,l1_mb,total_mb,compressed_bytes,summary_and_ms,"
            "summary_or_ms,summary_xor_ms\n");
    if (raw_path != NULL) {
        raw = fopen(raw_path, "w");
        if (raw == NULL) {
            fprintf(stderr, "cannot open %s\n", raw_path);
            fclose(out);
            return 1;
        }
        fprintf(raw,
                "protocol,implementation,mode,cardinality,iteration,operation,"
                "time_ms,result_cardinality,loaded_count\n");
    }

    for (ci = 0; ci < n_cards; ci++) {
        int c = cards[ci];
        char dir[512];
        if (only_card != 0 && c != only_card) continue;
        bm_entry *entries = NULL;
        size_t n_files = 0;
        size_t compressed_bytes;
        double density = 1.0 / (double)c;
        double tLOAD[5], tAND[5], tOR[5], tXOR[5], tANDNOT[5], tNOT[5];
        double tSummaryAND[5], tSummaryOR[5], tSummaryXOR[5], tSummaryMOR[5];
        double mLOAD, mAND, mOR, mXOR, mANDNOT, mNOT;
        double mSummaryAND, mSummaryOR, mSummaryXOR, mSummaryMOR;
        size_t summary_multi_or_card = 0;
        ddc_c_size_info storage;
        ddc_c A, B;
        int i;

        snprintf(dir, sizeof dir, "%s/bm_100m_c%d_ddc_w8", root, c);
        if (!collect_bm_files(dir, &entries, &n_files) || n_files < 2) {
            fprintf(stderr, "[skip] %s (<2 .bm files)\n", dir);
            free_bm_entries(entries, n_files);
            continue;
        }
        compressed_bytes = total_file_size_bytes(entries, n_files);

        /* Match benchmark_main.cpp's one-time compressed Summary warmup. */
        {
            ddc_c warm_a, warm_b, warm_result;
            ddc_c_init(&warm_a); ddc_c_init(&warm_b); ddc_c_init(&warm_result);
            if (!load_bm(entries[0].path, &warm_a) ||
                !load_bm(entries[1].path, &warm_b) ||
                ddc_c_or(&warm_result, &warm_a, &warm_b) != DDC_C_OK) {
                fprintf(stderr, "[skip] %s (summary warmup failed)\n", dir);
                ddc_c_destroy(&warm_a); ddc_c_destroy(&warm_b);
                ddc_c_destroy(&warm_result);
                free_bm_entries(entries, n_files);
                continue;
            }
            ddc_c_destroy(&warm_a); ddc_c_destroy(&warm_b);
            ddc_c_destroy(&warm_result);
        }

        /* Each Summary sample has the C++ compressed benchmark lifecycle:
         * load every input, measure operations, then destroy all inputs before
         * the next sample. DDC-C uses its native result ownership path. */
        for (i = 0; i < N_ITER; i++) {
            ddc_c *loaded = NULL;
            ddc_c or_result, and_result, xor_result, multi_result;
            size_t or_card, and_card, xor_card;
            double t0;
            ddc_c_init(&or_result);
            ddc_c_init(&and_result);
            ddc_c_init(&xor_result);
            ddc_c_init(&multi_result);
            t0 = now_ms();
            if (!load_all_bms(entries, n_files, &loaded)) {
                fprintf(stderr, "[skip] %s (summary load-all failed)\n", dir);
                ddc_c_destroy(&or_result); ddc_c_destroy(&and_result);
                ddc_c_destroy(&xor_result); ddc_c_destroy(&multi_result);
                break;
            }
            tLOAD[i] = now_ms() - t0;
            if (i == 0) compute_storage_summary(loaded, n_files, &storage);

            t0 = now_ms();
            if (!run_binary_summary(ddc_c_or, &loaded[0], &loaded[1], &or_result)) {
                fprintf(stderr, "[skip] %s (summary OR failed)\n", dir);
                destroy_bitmaps(loaded, n_files);
                ddc_c_destroy(&or_result); ddc_c_destroy(&and_result);
                ddc_c_destroy(&xor_result); ddc_c_destroy(&multi_result);
                break;
            }
            tSummaryOR[i] = now_ms() - t0;

            t0 = now_ms();
            if (!run_binary_summary(ddc_c_and, &loaded[0], &loaded[1], &and_result)) {
                fprintf(stderr, "[skip] %s (summary AND failed)\n", dir);
                destroy_bitmaps(loaded, n_files);
                ddc_c_destroy(&or_result); ddc_c_destroy(&and_result);
                ddc_c_destroy(&xor_result); ddc_c_destroy(&multi_result);
                break;
            }
            tSummaryAND[i] = now_ms() - t0;

            t0 = now_ms();
            if (!run_binary_summary(ddc_c_xor, &loaded[0], &loaded[1], &xor_result)) {
                fprintf(stderr, "[skip] %s (summary XOR failed)\n", dir);
                destroy_bitmaps(loaded, n_files);
                ddc_c_destroy(&or_result); ddc_c_destroy(&and_result);
                ddc_c_destroy(&xor_result); ddc_c_destroy(&multi_result);
                break;
            }
            tSummaryXOR[i] = now_ms() - t0;

            /* Match benchmark_main.cpp's pairwise scope: compute cardinality,
             * then release all three pairwise results before multi-way OR. */
            or_card = ddc_c_popcount(&or_result);
            and_card = ddc_c_popcount(&and_result);
            xor_card = ddc_c_popcount(&xor_result);
            write_raw_sample(raw, "summary", c, i + 1, "load", tLOAD[i], 0, n_files);
            write_raw_sample(raw, "summary", c, i + 1, "OR", tSummaryOR[i], or_card, n_files);
            write_raw_sample(raw, "summary", c, i + 1, "AND", tSummaryAND[i], and_card, n_files);
            write_raw_sample(raw, "summary", c, i + 1, "XOR", tSummaryXOR[i], xor_card, n_files);
            ddc_c_destroy(&or_result);
            ddc_c_destroy(&and_result);
            ddc_c_destroy(&xor_result);

            t0 = now_ms();
            if (!run_multi_or_summary(loaded, n_files, &multi_result)) {
                fprintf(stderr, "[skip] %s (summary multi-way OR failed)\n", dir);
                destroy_bitmaps(loaded, n_files);
                ddc_c_destroy(&multi_result);
                break;
            }
            tSummaryMOR[i] = now_ms() - t0;
            summary_multi_or_card = ddc_c_popcount(&multi_result);
            write_raw_sample(raw, "summary", c, i + 1, "multi-OR", tSummaryMOR[i],
                             summary_multi_or_card, n_files);
            ddc_c_destroy(&multi_result);
            destroy_bitmaps(loaded, n_files);
        }
        if (i != N_ITER) {
            free_bm_entries(entries, n_files);
            continue;
        }

        /* C++ measures operation-only after all Summary samples on separately
         * loaded inputs; keep the same phase ordering for standalone DDC-C. */
        ddc_c_init(&A);
        ddc_c_init(&B);
        if (!load_bm(entries[0].path, &A) || !load_bm(entries[1].path, &B)) {
            fprintf(stderr, "[skip] %s (direct first-two load failed)\n", dir);
            ddc_c_destroy(&A);
            ddc_c_destroy(&B);
            free_bm_entries(entries, n_files);
            continue;
        }

        /* Direct-operation warmup. */
        { ddc_c w; ddc_c_init(&w); ddc_c_and(&w, &A, &B); ddc_c_destroy(&w); }
        { ddc_c w; ddc_c_init(&w); ddc_c_or(&w,  &A, &B); ddc_c_destroy(&w); }
        { ddc_c w; ddc_c_init(&w); ddc_c_xor(&w, &A, &B); ddc_c_destroy(&w); }
        /* NOT warmup: keep not_r alive so subsequent calls hit fast path. */
        ddc_c not_r; ddc_c_init(&not_r); ddc_c_not(&not_r, &A);

        for (i = 0; i < N_ITER; i++) {
            ddc_c r; ddc_c_init(&r);
            double t0 = now_ms(); ddc_c_and(&r, &A, &B); tAND[i] = now_ms() - t0;
            write_raw_sample(raw, "direct", c, i + 1, "AND", tAND[i],
                             ddc_c_popcount(&r), 2);
            ddc_c_destroy(&r);
        }
        for (i = 0; i < N_ITER; i++) {
            ddc_c r; ddc_c_init(&r);
            double t0 = now_ms(); ddc_c_or(&r, &A, &B); tOR[i] = now_ms() - t0;
            write_raw_sample(raw, "direct", c, i + 1, "OR", tOR[i],
                             ddc_c_popcount(&r), 2);
            ddc_c_destroy(&r);
        }
        for (i = 0; i < N_ITER; i++) {
            ddc_c r; ddc_c_init(&r);
            double t0 = now_ms(); ddc_c_xor(&r, &A, &B); tXOR[i] = now_ms() - t0;
            write_raw_sample(raw, "direct", c, i + 1, "XOR", tXOR[i],
                             ddc_c_popcount(&r), 2);
            ddc_c_destroy(&r);
        }

        for (i = 0; i < N_ITER; i++) {
            ddc_c nb; ddc_c_init(&nb); ddc_c_not(&nb, &B);
            ddc_c r;  ddc_c_init(&r);
            double t0 = now_ms(); ddc_c_and(&r, &A, &nb); tANDNOT[i] = now_ms() - t0;
            ddc_c_destroy(&nb); ddc_c_destroy(&r);
        }
        for (i = 0; i < N_ITER; i++) {
            /* Reuse not_r across iterations so ddc_c_not can take the
             * buffer-reuse fast path (avoids per-segment malloc). */
            double t0 = now_ms(); ddc_c_not(&not_r, &A); tNOT[i] = now_ms() - t0;
            write_raw_sample(raw, "direct", c, i + 1, "NOT", tNOT[i],
                             ddc_c_popcount(&not_r), 2);
        }
        ddc_c_destroy(&not_r);

        mLOAD   = median5(tLOAD);
        mAND    = median5(tAND);
        mOR     = median5(tOR);
        mXOR    = median5(tXOR);
        mANDNOT = median5(tANDNOT);
        mNOT    = median5(tNOT);
        mSummaryAND = median5(tSummaryAND);
        mSummaryOR  = median5(tSummaryOR);
        mSummaryXOR = median5(tSummaryXOR);
        mSummaryMOR = median5(tSummaryMOR);

        printf("c=%d direct(and=%.4f or=%.4f xor=%.4f) "
               "summary(and=%.4f or=%.4f xor=%.4f multi_or=%.4f) ms\n",
               c, mAND, mOR, mXOR,
               mSummaryAND, mSummaryOR, mSummaryXOR, mSummaryMOR);

        fprintf(out,
                "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%zu,%.8f,%.4f,%zu,"
                "%.4f,%.4f,%.4f,%.4f,%.4f,%zu,%.4f,%.4f,%.4f\n",
                c, mAND, mOR, mXOR, mANDNOT, mNOT, mLOAD, n_files, density,
                mSummaryMOR, summary_multi_or_card, bits_to_mb(storage.l4_bits),
                bits_to_mb(storage.l3_literal_bits),
                bits_to_mb(storage.l2_literal_bits),
                bits_to_mb(storage.l1_literal_bits),
                bits_to_mb(storage.total_bits), compressed_bytes,
                mSummaryAND, mSummaryOR, mSummaryXOR);

        ddc_c_destroy(&A);
        ddc_c_destroy(&B);
        free_bm_entries(entries, n_files);
    }

    if (raw != NULL) fclose(raw);
    fclose(out);
    printf("[done] wrote %s\n", out_path);
    return 0;
}
