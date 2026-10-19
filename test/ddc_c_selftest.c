#include "../include/ddc_c.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int equal_bits(const uint8_t *actual, const uint8_t *expected, size_t count) {
    return memcmp(actual, expected, count) == 0;
}

static int check_standard(void) {
    enum { N = 131117 };
    uint8_t *a = calloc(N, 1), *b = calloc(N, 1), *actual = calloc(N, 1), *expect = calloc(N, 1);
    uint8_t *wire = NULL;
    size_t i, written = 0, consumed = 0;
    ddc_c ca, cb, result, decoded;
    int ok = 0;
    if (a == NULL || b == NULL || actual == NULL || expect == NULL) goto done;
    for (i = 0; i < N; ++i) {
        a[i] = (uint8_t)((i % 17u == 0u) || (i > 65000u && i < 65536u));
        b[i] = (uint8_t)((i % 29u == 0u) || (i > 65520u && i < 65580u));
    }
    ddc_c_init(&ca); ddc_c_init(&cb); ddc_c_init(&result); ddc_c_init(&decoded);
    if (ddc_c_compress(&ca, a, N, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK ||
        ddc_c_compress(&cb, b, N, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK) goto destroy;
    if (ddc_c_and(&result, &ca, &cb) != DDC_C_OK ||
        ddc_c_decompress(&result, actual, N) != DDC_C_OK) goto destroy;
    for (i = 0; i < N; ++i) expect[i] = (uint8_t)(a[i] & b[i]);
    if (!equal_bits(actual, expect, N)) goto destroy;
    if (ddc_c_not_inplace(&result) != DDC_C_OK ||
        ddc_c_decompress(&result, actual, N) != DDC_C_OK) goto destroy;
    for (i = 0; i < N; ++i) expect[i] = (uint8_t)!expect[i];
    if (!equal_bits(actual, expect, N)) goto destroy;
    wire = malloc(N * 3u);
    if (wire == NULL) goto destroy;
    if (ddc_c_serialize(&ca, wire, N * 3u, &written) != DDC_C_OK ||
        ddc_c_deserialize(&decoded, wire, written, &consumed) != DDC_C_OK ||
        consumed != written || ddc_c_decompress(&decoded, actual, N) != DDC_C_OK ||
        !equal_bits(actual, a, N)) goto destroy;
    ddc_c_destroy(&decoded);
    ddc_c_init(&decoded);
    if (ddc_c_serialize_packed(&ca, wire, N * 3u, &written) != DDC_C_OK ||
        ddc_c_deserialize_packed(&decoded, wire, written, &consumed) != DDC_C_OK ||
        consumed != written || ddc_c_decompress(&decoded, actual, N) != DDC_C_OK ||
        !equal_bits(actual, a, N)) goto destroy;
    ddc_c_destroy(&decoded);
    ddc_c_init(&decoded);
    if (ddc_c_serialize_v4(&ca, wire, N * 3u, &written) != DDC_C_OK ||
        ddc_c_deserialize_v4(&decoded, wire, written, &consumed) != DDC_C_OK ||
        consumed != written || ddc_c_decompress(&decoded, actual, N) != DDC_C_OK ||
        !equal_bits(actual, a, N)) goto destroy;
    ok = 1;
destroy:
    ddc_c_destroy(&ca); ddc_c_destroy(&cb); ddc_c_destroy(&result); ddc_c_destroy(&decoded);
done:
    free(wire); free(a); free(b); free(actual); free(expect);
    return ok;
}

typedef struct position_list {
    size_t values[8];
    size_t count;
} position_list;

static void collect_set_bit(void *ctx, size_t position) {
    position_list *positions = (position_list *)ctx;
    if (positions->count < sizeof(positions->values) / sizeof(positions->values[0]))
        positions->values[positions->count] = position;
    ++positions->count;
}

static int check_traversal_and_boundaries(void) {
    enum { N = 65545 };
    const size_t expected[] = {0, 6, 7, 8, 9, 65535, 65536, N - 1};
    uint8_t *bits = NULL, *decoded = NULL;
    size_t positions[sizeof(expected) / sizeof(expected[0])];
    size_t untouched[sizeof(expected) / sizeof(expected[0])];
    size_t required = 0, i;
    position_list callback_positions = {{0}, 0};
    ddc_c data, out, mismatched;
    int ok = 0;

    bits = calloc(N, 1);
    decoded = calloc(N, 1);
    if (bits == NULL || decoded == NULL) goto done;
    for (i = 0; i < sizeof(expected) / sizeof(expected[0]); ++i) bits[expected[i]] = 1;
    for (i = 0; i < sizeof(untouched) / sizeof(untouched[0]); ++i) untouched[i] = SIZE_MAX;

    ddc_c_init(&data); ddc_c_init(&out); ddc_c_init(&mismatched);
    if (ddc_c_compress(&data, bits, N, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK ||
        ddc_c_compress(&out, bits, N, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK ||
        ddc_c_compress(&mismatched, bits, N - 1, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK)
        goto destroy;

    if (ddc_c_for_each_set_bit(&data, collect_set_bit, &callback_positions) != DDC_C_OK ||
        callback_positions.count != sizeof(expected) / sizeof(expected[0]) ||
        memcmp(callback_positions.values, expected, sizeof(expected)) != 0)
        goto destroy;
    if (ddc_c_for_each_set_bit(&data, NULL, NULL) != DDC_C_INVALID_ARGUMENT ||
        ddc_c_set_bit_positions(&data, NULL, 0, NULL) != DDC_C_INVALID_ARGUMENT)
        goto destroy;
    if (ddc_c_set_bit_positions(&data, NULL, 0, &required) != DDC_C_BUFFER_TOO_SMALL ||
        required != sizeof(expected) / sizeof(expected[0]))
        goto destroy;
    if (ddc_c_set_bit_positions(&data, untouched,
                                sizeof(untouched) / sizeof(untouched[0]) - 1,
                                &required) != DDC_C_BUFFER_TOO_SMALL ||
        required != sizeof(expected) / sizeof(expected[0]))
        goto destroy;
    for (i = 0; i < sizeof(untouched) / sizeof(untouched[0]); ++i)
        if (untouched[i] != SIZE_MAX) goto destroy;
    if (ddc_c_set_bit_positions(&data, positions,
                                sizeof(positions) / sizeof(positions[0]),
                                &required) != DDC_C_OK ||
        required != sizeof(expected) / sizeof(expected[0]) ||
        memcmp(positions, expected, sizeof(expected)) != 0)
        goto destroy;

    if (ddc_c_decompress(&data, decoded, N - 1) != DDC_C_BUFFER_TOO_SMALL ||
        ddc_c_and(&out, &data, &out) != DDC_C_INVALID_ARGUMENT ||
        ddc_c_and(&out, &data, &mismatched) != DDC_C_INVALID_ARGUMENT ||
        ddc_c_decompress(&out, decoded, N) != DDC_C_OK ||
        !equal_bits(decoded, bits, N))
        goto destroy;
    ok = 1;

destroy:
    ddc_c_destroy(&data); ddc_c_destroy(&out); ddc_c_destroy(&mismatched);
done:
    free(bits); free(decoded);
    return ok;
}

static int check_binary_edge_cases(void) {
    /* Bit pattern: bits 0..N-1, alternating 0/1. */
    enum { N = 512 };
    uint8_t *bits_a = NULL, *bits_b = NULL, *out_buf = NULL;
    ddc_c ca, cb, zero_ddc, ones_ddc, result, tmp;
    size_t i;
    int ok = 0;

    bits_a = calloc(N, 1);
    bits_b = calloc(N, 1);
    out_buf = calloc(N, 1);
    if (!bits_a || !bits_b || !out_buf) goto done;

    for (i = 0; i < N; ++i) {
        bits_a[i] = (uint8_t)(i % 3 == 0);
        bits_b[i] = (uint8_t)(i % 5 == 0);
    }

    ddc_c_init(&ca); ddc_c_init(&cb);
    ddc_c_init(&zero_ddc); ddc_c_init(&ones_ddc);
    ddc_c_init(&result); ddc_c_init(&tmp);

    /* Compress test operands. */
    if (ddc_c_compress(&ca, bits_a, N, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK) goto destroy;
    if (ddc_c_compress(&cb, bits_b, N, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK) goto destroy;

    /* Build all-zero and all-ones DDC with same N. */
    {
        uint8_t *zero_bits = calloc(N, 1);
        uint8_t *ones_bits = calloc(N, 1);
        if (!zero_bits || !ones_bits) { free(zero_bits); free(ones_bits); goto destroy; }
        memset(ones_bits, 1, N);
        if (ddc_c_compress(&zero_ddc, zero_bits, N, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK ||
            ddc_c_compress(&ones_ddc, ones_bits, N, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK) {
            free(zero_bits); free(ones_bits); goto destroy;
        }
        free(zero_bits); free(ones_bits);
    }

    /* --- segment_bits mismatch must return DDC_C_INVALID_ARGUMENT --- */
    {
        ddc_c mismatched_seg;
        ddc_c_init(&mismatched_seg);
        if (ddc_c_compress(&mismatched_seg, bits_a, N, 0, DDC_C_DEFAULT_SEGMENT_BITS / 2) != DDC_C_OK) {
            ddc_c_destroy(&mismatched_seg); goto destroy;
        }
        /* binary_inplace with segment_bits mismatch. */
        if (ddc_c_and_inplace(&ca, &mismatched_seg) != DDC_C_INVALID_ARGUMENT) {
            ddc_c_destroy(&mismatched_seg); goto destroy;
        }
        /* binary (out-of-place) with segment_bits mismatch. */
        if (ddc_c_and(&result, &ca, &mismatched_seg) != DDC_C_INVALID_ARGUMENT) {
            ddc_c_destroy(&mismatched_seg); goto destroy;
        }
        /* popcount_and with segment_bits mismatch must return 0. */
        if (ddc_c_popcount_and(&ca, &mismatched_seg) != 0) {
            ddc_c_destroy(&mismatched_seg); goto destroy;
        }
        ddc_c_destroy(&mismatched_seg);
    }

    /* --- XOR inplace: all-zero rhs → lhs unchanged --- */
    {
        uint8_t *before = calloc(N, 1), *after = calloc(N, 1);
        if (!before || !after) { free(before); free(after); goto destroy; }
        if (ddc_c_decompress(&ca, before, N) != DDC_C_OK) { free(before); free(after); goto destroy; }
        /* Make a copy of ca to mutate. */
        if (ddc_c_compress(&tmp, bits_a, N, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK) {
            free(before); free(after); goto destroy;
        }
        if (ddc_c_xor_inplace(&tmp, &zero_ddc) != DDC_C_OK) { free(before); free(after); goto destroy; }
        if (ddc_c_decompress(&tmp, after, N) != DDC_C_OK) { free(before); free(after); goto destroy; }
        if (!equal_bits(after, before, N)) { free(before); free(after); goto destroy; }
        ddc_c_destroy(&tmp); ddc_c_init(&tmp);
        free(before); free(after);
    }

    /* --- XOR inplace: all-ones rhs → NOT(lhs) --- */
    {
        uint8_t *notted = calloc(N, 1);
        if (!notted) goto destroy;
        if (ddc_c_compress(&result, bits_a, N, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK ||
            ddc_c_not_inplace(&result) != DDC_C_OK ||
            ddc_c_decompress(&result, notted, N) != DDC_C_OK) { free(notted); goto destroy; }
        if (ddc_c_compress(&tmp, bits_a, N, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK) {
            free(notted); goto destroy;
        }
        if (ddc_c_xor_inplace(&tmp, &ones_ddc) != DDC_C_OK) { free(notted); goto destroy; }
        if (ddc_c_decompress(&tmp, out_buf, N) != DDC_C_OK) { free(notted); goto destroy; }
        if (!equal_bits(out_buf, notted, N)) { free(notted); goto destroy; }
        ddc_c_destroy(&tmp); ddc_c_init(&tmp);
        ddc_c_destroy(&result); ddc_c_init(&result);
        free(notted);
    }

    /* --- XOR inplace: all-zero lhs → copy rhs --- */
    {
        uint8_t *rhs_buf = calloc(N, 1);
        if (!rhs_buf) goto destroy;
        if (ddc_c_decompress(&cb, rhs_buf, N) != DDC_C_OK) { free(rhs_buf); goto destroy; }
        if (ddc_c_compress(&tmp, NULL, 0, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK) {
            /* fallback: compress zero array */
            uint8_t *zb = calloc(N, 1);
            if (!zb) { free(rhs_buf); goto destroy; }
            ddc_c_destroy(&tmp); ddc_c_init(&tmp);
            if (ddc_c_compress(&tmp, zb, N, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK) {
                free(zb); free(rhs_buf); goto destroy;
            }
            free(zb);
        }
        /* Use zero_ddc as lhs by compressing a fresh copy. */
        {
            uint8_t *zb = calloc(N, 1);
            if (!zb) { free(rhs_buf); goto destroy; }
            ddc_c_destroy(&tmp); ddc_c_init(&tmp);
            if (ddc_c_compress(&tmp, zb, N, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK) {
                free(zb); free(rhs_buf); goto destroy;
            }
            free(zb);
        }
        if (ddc_c_xor_inplace(&tmp, &cb) != DDC_C_OK) { free(rhs_buf); goto destroy; }
        if (ddc_c_decompress(&tmp, out_buf, N) != DDC_C_OK) { free(rhs_buf); goto destroy; }
        if (!equal_bits(out_buf, rhs_buf, N)) { free(rhs_buf); goto destroy; }
        ddc_c_destroy(&tmp); ddc_c_init(&tmp);
        free(rhs_buf);
    }

    /* --- XOR inplace: all-ones lhs → NOT(rhs) --- */
    {
        uint8_t *not_rhs = calloc(N, 1);
        if (!not_rhs) goto destroy;
        if (ddc_c_compress(&result, bits_b, N, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK ||
            ddc_c_not_inplace(&result) != DDC_C_OK ||
            ddc_c_decompress(&result, not_rhs, N) != DDC_C_OK) { free(not_rhs); goto destroy; }
        /* Start with all-ones lhs. */
        {
            uint8_t *ob = calloc(N, 1);
            if (!ob) { free(not_rhs); goto destroy; }
            memset(ob, 1, N);
            ddc_c_destroy(&tmp); ddc_c_init(&tmp);
            if (ddc_c_compress(&tmp, ob, N, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK) {
                free(ob); free(not_rhs); goto destroy;
            }
            free(ob);
        }
        if (ddc_c_xor_inplace(&tmp, &cb) != DDC_C_OK) { free(not_rhs); goto destroy; }
        if (ddc_c_decompress(&tmp, out_buf, N) != DDC_C_OK) { free(not_rhs); goto destroy; }
        if (!equal_bits(out_buf, not_rhs, N)) { free(not_rhs); goto destroy; }
        ddc_c_destroy(&tmp); ddc_c_init(&tmp);
        ddc_c_destroy(&result); ddc_c_init(&result);
        free(not_rhs);
    }

    /* --- OR inplace: all-zero lhs → lhs becomes rhs (was a bug: used to continue) --- */
    {
        uint8_t *rhs_buf = calloc(N, 1);
        uint8_t *zb = calloc(N, 1);
        if (!rhs_buf || !zb) { free(rhs_buf); free(zb); goto destroy; }
        if (ddc_c_decompress(&cb, rhs_buf, N) != DDC_C_OK) { free(rhs_buf); free(zb); goto destroy; }
        if (ddc_c_compress(&tmp, zb, N, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK) {
            free(rhs_buf); free(zb); goto destroy;
        }
        free(zb);
        if (ddc_c_or_inplace(&tmp, &cb) != DDC_C_OK) { free(rhs_buf); goto destroy; }
        if (ddc_c_decompress(&tmp, out_buf, N) != DDC_C_OK) { free(rhs_buf); goto destroy; }
        if (!equal_bits(out_buf, rhs_buf, N)) { free(rhs_buf); goto destroy; }
        ddc_c_destroy(&tmp); ddc_c_init(&tmp);
        free(rhs_buf);
    }

    /* --- ddc_c_btv_not on zero bit_count --- */
    {
        ddc_c_btv empty_btv, out_btv;
        ddc_c_btv_init(&empty_btv, 0, 0, DDC_C_COMPRESSED);
        ddc_c_btv_init(&out_btv, 0, 0, DDC_C_COMPRESSED);
        if (ddc_c_btv_not(&out_btv, &empty_btv) != DDC_C_OK || out_btv.bit_count != 0) {
            ddc_c_btv_destroy(&empty_btv); ddc_c_btv_destroy(&out_btv); goto destroy;
        }
        /* ddc_c_btv_not_inplace on empty BTV */
        if (ddc_c_btv_not_inplace(&empty_btv) != DDC_C_OK || empty_btv.bit_count != 0) {
            ddc_c_btv_destroy(&empty_btv); ddc_c_btv_destroy(&out_btv); goto destroy;
        }
        ddc_c_btv_destroy(&empty_btv); ddc_c_btv_destroy(&out_btv);
    }

    ok = 1;
destroy:
    ddc_c_destroy(&ca); ddc_c_destroy(&cb);
    ddc_c_destroy(&zero_ddc); ddc_c_destroy(&ones_ddc);
    ddc_c_destroy(&result); ddc_c_destroy(&tmp);
done:
    free(bits_a); free(bits_b); free(out_buf);
    return ok;
}

int main(void) {
    if (!check_standard() || !check_traversal_and_boundaries() ||
        !check_binary_edge_cases()) {
        fprintf(stderr, "ddc_c standard self-test failed\n");
        return EXIT_FAILURE;
    }
    puts("ddc_c self-test passed");
    return EXIT_SUCCESS;
}
