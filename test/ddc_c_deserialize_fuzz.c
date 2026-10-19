#include "../include/ddc_c.h"

#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef ddc_c_status (*ddc_c_serializer)(const ddc_c *, uint8_t *, size_t, size_t *);
typedef ddc_c_status (*ddc_c_deserializer)(ddc_c *, const uint8_t *, size_t, size_t *);

typedef struct fuzz_wire {
    uint8_t *data;
    size_t len;
    size_t cap;
} fuzz_wire;

typedef struct fuzz_format {
    const char *name;
    ddc_c_deserializer deserialize;
} fuzz_format;

static void fuzz_wire_destroy(fuzz_wire *wire) {
    free(wire->data);
    wire->data = NULL;
    wire->len = 0;
    wire->cap = 0;
}

static int fuzz_wire_reserve(fuzz_wire *wire, size_t extra) {
    size_t needed;
    uint8_t *data;
    if (extra > SIZE_MAX - wire->len) return 0;
    needed = wire->len + extra;
    if (needed <= wire->cap) return 1;
    data = (uint8_t *)realloc(wire->data, needed);
    if (data == NULL) return 0;
    wire->data = data;
    wire->cap = needed;
    return 1;
}

static int fuzz_wire_put(fuzz_wire *wire, const void *data, size_t len) {
    if (!fuzz_wire_reserve(wire, len)) return 0;
    if (len != 0) memcpy(wire->data + wire->len, data, len);
    wire->len += len;
    return 1;
}

static int fuzz_wire_put_u8(fuzz_wire *wire, uint8_t value) {
    return fuzz_wire_put(wire, &value, sizeof(value));
}

static int fuzz_wire_put_u64(fuzz_wire *wire, uint64_t value) {
    return fuzz_wire_put(wire, &value, sizeof(value));
}

static int fuzz_make_v1_btv(fuzz_wire *wire, const ddc_c_btv *btv) {
    uint8_t *l3_bits = NULL;
    size_t l3_bytes = btv->l3_count / 8u + (btv->l3_count % 8u != 0u);
    size_t i, l3_lit_index = 0;
    const uint8_t fmt = 8u;
    if (l3_bytes != 0) {
        l3_bits = (uint8_t *)malloc(l3_bytes);
        if (l3_bits == NULL) return 0;
        memset(l3_bits, btv->l3_fill_ones ? UINT8_MAX : 0, l3_bytes);
        for (i = 0; i < btv->l4_count; ++i) {
            if ((btv->l4_bits.data[i / 8u] >> (i % 8u)) & 1u) {
                if (l3_lit_index >= btv->l3_lit_count) {
                    free(l3_bits);
                    return 0;
                }
                l3_bits[i] = btv->l3_lits.data[l3_lit_index++];
            }
        }
        if (btv->l3_count % 8u != 0u)
            l3_bits[l3_bytes - 1u] &= (uint8_t)((1u << (btv->l3_count % 8u)) - 1u);
    }
    if (l3_lit_index != btv->l3_lit_count ||
        !fuzz_wire_put_u8(wire, fmt) ||
        !fuzz_wire_put_u8(wire, btv->l1_fill_ones) ||
        !fuzz_wire_put_u8(wire, btv->l2_fill_ones) ||
        !fuzz_wire_put_u64(wire, (uint64_t)btv->bit_count) ||
        !fuzz_wire_put_u64(wire, (uint64_t)btv->l2_count) ||
        !fuzz_wire_put_u64(wire, (uint64_t)btv->l3_count) ||
        !fuzz_wire_put_u64(wire, (uint64_t)l3_bytes) ||
        !fuzz_wire_put(wire, l3_bits, l3_bytes) ||
        !fuzz_wire_put_u64(wire, (uint64_t)btv->l2_lit_count) ||
        !fuzz_wire_put(wire, btv->l2_lits.data, btv->l2_lit_count) ||
        !fuzz_wire_put_u64(wire, (uint64_t)btv->l1_lit_count) ||
        !fuzz_wire_put(wire, btv->l1_lits.data, btv->l1_lit_count)) {
        free(l3_bits);
        return 0;
    }
    free(l3_bits);
    return 1;
}

static int fuzz_make_v1(fuzz_wire *wire, const ddc_c *ddc) {
    size_t i;
    if (!fuzz_wire_put_u64(wire, (uint64_t)ddc->bit_count) ||
        !fuzz_wire_put_u64(wire, (uint64_t)ddc->segment_bits) ||
        !fuzz_wire_put_u64(wire, (uint64_t)ddc->segment_count))
        return 0;
    for (i = 0; i < ddc->segment_count; ++i)
        if (!fuzz_make_v1_btv(wire, &ddc->segments[i])) return 0;
    return 1;
}

static uint32_t fuzz_next(uint32_t *state) {
    uint32_t value = *state;
    value ^= value << 13u;
    value ^= value >> 17u;
    value ^= value << 5u;
    *state = value;
    return value;
}

static int fuzz_matches(const ddc_c *actual, const uint8_t *expected, size_t count) {
    uint8_t *decoded;
    int matches;
    if (actual->bit_count != count) return 0;
    decoded = (uint8_t *)calloc(count == 0 ? 1u : count, 1);
    if (decoded == NULL) return 0;
    matches = ddc_c_decompress(actual, decoded, count) == DDC_C_OK &&
              memcmp(decoded, expected, count) == 0;
    free(decoded);
    return matches;
}

static int fuzz_check_success(const fuzz_format *format, const uint8_t *data, size_t len,
                              const uint8_t *expected, size_t expected_count) {
    ddc_c decoded;
    size_t consumed = 0;
    int ok;
    ddc_c_init(&decoded);
    ok = format->deserialize(&decoded, data, len, &consumed) == DDC_C_OK &&
         consumed == len && fuzz_matches(&decoded, expected, expected_count);
    ddc_c_destroy(&decoded);
    return ok;
}

static int fuzz_check_rejection(const fuzz_format *format, const uint8_t *data, size_t len) {
    ddc_c decoded;
    ddc_c_status status;
    ddc_c_init(&decoded);
    status = format->deserialize(&decoded, data, len, NULL);
    if (status == DDC_C_OK) {
        ddc_c_destroy(&decoded);
        return 0;
    }
    if (decoded.segments != NULL || decoded.segment_count != 0 || decoded.bit_count != 0 ||
        decoded.segment_bits != DDC_C_DEFAULT_SEGMENT_BITS) {
        ddc_c_destroy(&decoded);
        return 0;
    }
    ddc_c_destroy(&decoded);
    return 1;
}

static int fuzz_exercise_format(const fuzz_format *format, const fuzz_wire *wire,
                                const uint8_t *expected, size_t expected_count,
                                unsigned iterations) {
    static const uint8_t masks[] = {0x01u, 0x80u, 0xffu};
    uint8_t *scratch;
    size_t i, mask_index;
    uint32_t state = 0x9e3779b9u;

    if (!fuzz_check_success(format, wire->data, wire->len, expected, expected_count)) {
        fprintf(stderr, "%s valid seed rejected\n", format->name);
        return 0;
    }
    for (i = 0; i < wire->len; ++i) {
        if (!fuzz_check_rejection(format, wire->data, i)) {
            fprintf(stderr, "%s accepted truncated input at %zu bytes\n", format->name, i);
            return 0;
        }
    }

    scratch = (uint8_t *)malloc(wire->len == 0 ? 1u : wire->len);
    if (scratch == NULL) return 0;
    for (i = 0; i < wire->len; ++i) {
        for (mask_index = 0; mask_index < sizeof(masks); ++mask_index) {
            memcpy(scratch, wire->data, wire->len);
            scratch[i] ^= masks[mask_index];
            if (!fuzz_check_rejection(format, scratch, wire->len)) {
                ddc_c decoded;
                ddc_c_init(&decoded);
                if (format->deserialize(&decoded, scratch, wire->len, NULL) != DDC_C_OK ||
                    decoded.bit_count > expected_count + 64u) {
                    fprintf(stderr, "%s accepted invalid mutation at byte %zu\n", format->name, i);
                    ddc_c_destroy(&decoded);
                    free(scratch);
                    return 0;
                }
                ddc_c_destroy(&decoded);
            }
        }
    }
    for (i = 0; i < iterations; ++i) {
        size_t mutations = 1u + fuzz_next(&state) % 8u;
        size_t mutation;
        memcpy(scratch, wire->data, wire->len);
        for (mutation = 0; mutation < mutations; ++mutation) {
            size_t offset = fuzz_next(&state) % wire->len;
            scratch[offset] ^= (uint8_t)fuzz_next(&state);
        }
        if (!fuzz_check_rejection(format, scratch, wire->len)) {
            ddc_c decoded;
            ddc_c_init(&decoded);
            if (format->deserialize(&decoded, scratch, wire->len, NULL) != DDC_C_OK ||
                decoded.bit_count > expected_count + 64u) {
                fprintf(stderr, "%s accepted randomized invalid input\n", format->name);
                ddc_c_destroy(&decoded);
                free(scratch);
                return 0;
            }
            ddc_c_destroy(&decoded);
        }
    }
    free(scratch);
    return 1;
}

static int fuzz_exercise_headers(const fuzz_format *format) {
    uint8_t bad_header[25] = {0};
    uint64_t huge = UINT64_MAX;
    memcpy(bad_header + 1, &huge, sizeof(huge));
    memcpy(bad_header + 9, &huge, sizeof(huge));
    memcpy(bad_header + 17, &huge, sizeof(huge));
    if (strcmp(format->name, "V2/V1") != 0) bad_header[0] =
        strcmp(format->name, "V3") == 0 ? 0xCBu : 0xC4u;
    return fuzz_check_rejection(format, bad_header,
                                strcmp(format->name, "V2/V1") == 0 ? 24u : 25u);
}

int main(int argc, char **argv) {
    enum { BIT_COUNT = 131117 };
    uint8_t *bits = NULL;
    uint8_t *serialized = NULL;
    size_t serialized_len = 0;
    size_t i;
    unsigned iterations = 1000u;
    ddc_c source;
    fuzz_wire v1 = {0}, v2 = {0}, v3 = {0}, v4 = {0};
    const fuzz_format normal = {"V2/V1", ddc_c_deserialize};
    const fuzz_format packed = {"V3", ddc_c_deserialize_packed};
    const fuzz_format packed_v4 = {"V4", ddc_c_deserialize_v4};
    int ok = 0;

    if (argc == 2) {
        char *end = NULL;
        unsigned long value = strtoul(argv[1], &end, 10);
        if (end == argv[1] || *end != '\0' || value > 1000000ul) {
            fprintf(stderr, "usage: %s [iterations: 0..1000000]\n", argv[0]);
            return EXIT_FAILURE;
        }
        iterations = (unsigned)value;
    } else if (argc != 1) {
        fprintf(stderr, "usage: %s [iterations: 0..1000000]\n", argv[0]);
        return EXIT_FAILURE;
    }

    bits = (uint8_t *)calloc(BIT_COUNT, 1);
    serialized = (uint8_t *)malloc(BIT_COUNT * 3u);
    if (bits == NULL || serialized == NULL) goto done;
    for (i = 0; i < BIT_COUNT; ++i)
        bits[i] = (uint8_t)(i % 17u == 0u || i % 251u == 13u ||
                            (i >= 65520u && i < 65570u) ||
                            (i >= 131000u && i < 131100u));

    ddc_c_init(&source);
    if (ddc_c_compress(&source, bits, BIT_COUNT, 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK ||
        !fuzz_make_v1(&v1, &source)) goto destroy_source;
    if (ddc_c_serialize(&source, serialized, BIT_COUNT * 3u, &serialized_len) != DDC_C_OK ||
        !fuzz_wire_put(&v2, serialized, serialized_len)) goto destroy_source;
    if (ddc_c_serialize_packed(&source, serialized, BIT_COUNT * 3u, &serialized_len) != DDC_C_OK ||
        !fuzz_wire_put(&v3, serialized, serialized_len)) goto destroy_source;
    if (ddc_c_serialize_v4(&source, serialized, BIT_COUNT * 3u, &serialized_len) != DDC_C_OK ||
        !fuzz_wire_put(&v4, serialized, serialized_len)) goto destroy_source;

    ok = fuzz_exercise_format(&normal, &v1, bits, BIT_COUNT, iterations) &&
         fuzz_exercise_format(&normal, &v2, bits, BIT_COUNT, iterations) &&
         fuzz_exercise_format(&packed, &v3, bits, BIT_COUNT, iterations) &&
         fuzz_exercise_format(&packed_v4, &v4, bits, BIT_COUNT, iterations) &&
         fuzz_exercise_headers(&normal) && fuzz_exercise_headers(&packed) &&
         fuzz_exercise_headers(&packed_v4);

destroy_source:
    ddc_c_destroy(&source);
done:
    fuzz_wire_destroy(&v1);
    fuzz_wire_destroy(&v2);
    fuzz_wire_destroy(&v3);
    fuzz_wire_destroy(&v4);
    free(serialized);
    free(bits);
    if (!ok) return EXIT_FAILURE;
    printf("ddc_c deserialization fuzz test passed (%u randomized mutations per format)\n",
           iterations);
    return EXIT_SUCCESS;
}
