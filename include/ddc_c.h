#ifndef DDC_C_H
#define DDC_C_H

/*
 * Standalone C implementation of DDC's four-level bitmap format.
 *
 * This API intentionally mirrors the behavior and on-disk formats of the
 * C++ DDC library in ../include/ddc.h. Logical input bits are one byte per
 * bit (zero is clear; nonzero is set). L1 bitmap bytes are MSB-first; the
 * L2/L3/L4 control-marker bits are LSB-first.
 */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DDC_C_DEFAULT_SEGMENT_BITS ((size_t)1u << 16)
#define DDC_C_WORD_BITS 8u
#define DDC_C_WORDS_PER_REG 64u

typedef enum ddc_c_status {
    DDC_C_OK = 0,
    DDC_C_INVALID_ARGUMENT,
    DDC_C_ALLOCATION_FAILED,
    DDC_C_BUFFER_TOO_SMALL,
    DDC_C_MALFORMED_INPUT,
    DDC_C_UNSUPPORTED_FORMAT,
    DDC_C_STATE_ERROR
} ddc_c_status;

typedef enum ddc_c_state {
    DDC_C_UNCOMPRESSED = 0,
    DDC_C_COMPRESSED = 1,
    DDC_C_DECOMPRESSED = 2
} ddc_c_state;

typedef struct ddc_c_bytes {
    uint8_t *data;
    size_t len;
    size_t cap;
} ddc_c_bytes;

typedef struct ddc_c_size_info {
    size_t l3_bits;
    size_t l4_bits;
    size_t l3_literal_bits;
    size_t l2_literal_bits;
    size_t l1_literal_bits;
    size_t total_bits;
} ddc_c_size_info;

/* Per-segment DDCBtv equivalent. Fields match the C++ logical layout. */
typedef struct ddc_c_btv {
    ddc_c_state state;
    uint8_t l1_fill_ones;
    uint8_t l2_fill_ones;
    size_t bit_count;
    size_t l2_count;
    ddc_c_bytes l2_flat;
    size_t l3_count;
    ddc_c_bytes l3_bits;
    ddc_c_bytes l2_lits;
    size_t l2_lit_count;
    uint8_t l3_fill_ones;
    size_t l4_count;
    ddc_c_bytes l4_bits;
    ddc_c_bytes l3_lits;
    size_t l3_lit_count;
    ddc_c_bytes l1_lits;
    size_t l1_lit_count;
} ddc_c_btv;

/* Segmented DDC equivalent. */
typedef struct ddc_c {
    ddc_c_btv *segments;
    size_t segment_count;
    size_t segment_cap;
    size_t bit_count;
    size_t segment_bits;
    /* Contiguous backing store for decompressed-result L1 literal buffers
       produced by ddc_c_binary's generic per-segment kernel path. Segments
       whose l1_lits.data points into this arena carry cap == 0 to mark
       them as borrowed (ddc_c_bytes_destroy must not free borrowed data);
       this single block is freed here instead. NULL/len 0 when unused. */
    ddc_c_bytes l1_arena;
} ddc_c;

/* SparseDDC equivalent. */
typedef struct ddc_c_sparse {
    size_t bit_count;
    size_t segment_bits;
    size_t num_set_bits;
    uint32_t *seg_indices;
    ddc_c_btv *seg_data;
    size_t segment_count;
    size_t segment_cap;
} ddc_c_sparse;

typedef void (*ddc_c_literal_fn)(void *ctx, uint32_t word_pos, uint8_t value);
typedef void (*ddc_c_set_bit_fn)(void *ctx, size_t position);

/* Object lifecycle. Passing NULL is always safe for destroy calls. */
void ddc_c_btv_init(ddc_c_btv *btv, uint8_t l1_fill_ones,
                    uint8_t l2_fill_ones, ddc_c_state state);
void ddc_c_btv_destroy(ddc_c_btv *btv);
void ddc_c_init(ddc_c *ddc);
void ddc_c_destroy(ddc_c *ddc);
void ddc_c_sparse_init(ddc_c_sparse *sparse);
void ddc_c_sparse_destroy(ddc_c_sparse *sparse);

/* Result form matches the C++ mutable ddc_compress_results default (false). */
void ddc_c_set_compress_results(uint8_t enabled);
uint8_t ddc_c_get_compress_results(void);

/* Construction and decoding. */
ddc_c_status ddc_c_btv_compress(ddc_c_btv *out, const uint8_t *bits,
                                 size_t bit_count, uint8_t l1_fill_ones);
ddc_c_status ddc_c_btv_compress_sparse_segment(
    ddc_c_btv *out, const uint16_t *sorted_positions, size_t position_count,
    size_t segment_bits, uint8_t l1_fill_ones);
ddc_c_status ddc_c_btv_decompress(const ddc_c_btv *btv, uint8_t *out_bits,
                                   size_t out_capacity);

ddc_c_status ddc_c_compress(ddc_c *out, const uint8_t *bits,
                             size_t bit_count, uint8_t l1_fill_ones,
                             size_t segment_bits);
ddc_c_status ddc_c_from_sparse_positions(ddc_c *out,
                                          const uint32_t *positions,
                                          size_t position_count,
                                          size_t num_rows,
                                          size_t segment_bits);
ddc_c_status ddc_c_decompress(const ddc_c *ddc, uint8_t *out_bits,
                               size_t out_capacity);

/* Boolean operations. Results must not alias either input. */
ddc_c_status ddc_c_btv_and(ddc_c_btv *out, const ddc_c_btv *a,
                            const ddc_c_btv *b);
ddc_c_status ddc_c_btv_or(ddc_c_btv *out, const ddc_c_btv *a,
                           const ddc_c_btv *b);
ddc_c_status ddc_c_btv_xor(ddc_c_btv *out, const ddc_c_btv *a,
                            const ddc_c_btv *b);
ddc_c_status ddc_c_btv_not(ddc_c_btv *out, const ddc_c_btv *in);
ddc_c_status ddc_c_btv_and_inplace(ddc_c_btv *lhs, const ddc_c_btv *rhs);
ddc_c_status ddc_c_btv_or_inplace(ddc_c_btv *lhs, const ddc_c_btv *rhs);
ddc_c_status ddc_c_btv_xor_inplace(ddc_c_btv *lhs, const ddc_c_btv *rhs);
ddc_c_status ddc_c_btv_not_inplace(ddc_c_btv *btv);
ddc_c_status ddc_c_btv_and_no_bypass(ddc_c_btv *out, const ddc_c_btv *a,
                                      const ddc_c_btv *b);

ddc_c_status ddc_c_and(ddc_c *out, const ddc_c *a, const ddc_c *b);
ddc_c_status ddc_c_or(ddc_c *out, const ddc_c *a, const ddc_c *b);
ddc_c_status ddc_c_xor(ddc_c *out, const ddc_c *a, const ddc_c *b);
ddc_c_status ddc_c_not(ddc_c *out, const ddc_c *in);
ddc_c_status ddc_c_and_inplace(ddc_c *lhs, const ddc_c *rhs);
ddc_c_status ddc_c_or_inplace(ddc_c *lhs, const ddc_c *rhs);
ddc_c_status ddc_c_xor_inplace(ddc_c *lhs, const ddc_c *rhs);
ddc_c_status ddc_c_not_inplace(ddc_c *ddc);
ddc_c_status ddc_c_and_no_bypass(ddc_c *out, const ddc_c *a,
                                  const ddc_c *b);

/* Queries and traversal. */
size_t ddc_c_btv_popcount(const ddc_c_btv *btv);
size_t ddc_c_popcount(const ddc_c *ddc);
size_t ddc_c_btv_popcount_and(const ddc_c_btv *a, const ddc_c_btv *b);
size_t ddc_c_popcount_and(const ddc_c *a, const ddc_c *b);
uint8_t ddc_c_btv_is_all_zero(const ddc_c_btv *btv);
uint8_t ddc_c_btv_is_all_ones(const ddc_c_btv *btv);
size_t ddc_c_btv_num_fills(const ddc_c_btv *btv);
void ddc_c_btv_size_breakdown(const ddc_c_btv *btv,
                              ddc_c_size_info *out);
void ddc_c_size_breakdown(const ddc_c *ddc, ddc_c_size_info *out);
double ddc_c_btv_compression_ratio(const ddc_c_btv *btv);
double ddc_c_compression_ratio(const ddc_c *ddc);
ddc_c_status ddc_c_for_each_literal(const ddc_c *ddc,
                                    ddc_c_literal_fn fn, void *ctx);
/* Reports each logical set-bit position in ascending order. */
ddc_c_status ddc_c_for_each_set_bit(const ddc_c *ddc,
                                    ddc_c_set_bit_fn fn, void *ctx);
/* Writes ascending set-bit positions. `position_count` always receives the
 * required count; no positions are written when `positions_capacity` is too
 * small. `positions` may be NULL only when `positions_capacity` is zero. */
ddc_c_status ddc_c_set_bit_positions(const ddc_c *ddc, size_t *positions,
                                     size_t positions_capacity,
                                     size_t *position_count);

/* Multi-way and sparse operations. */
ddc_c_status ddc_c_or_many(ddc_c *out, size_t count, const ddc_c *const *ddcs);
ddc_c_status ddc_c_sparse_from_positions(ddc_c_sparse *out,
                                          const uint32_t *positions,
                                          size_t position_count,
                                          size_t num_rows,
                                          size_t segment_bits);
ddc_c_status ddc_c_sparse_apply_or_to(const ddc_c_sparse *sparse, ddc_c *dst);
ddc_c_status ddc_c_sparse_or_many(ddc_c *out, size_t count,
                                   const ddc_c_sparse *const *sparses,
                                   size_t num_rows, size_t segment_bits);
size_t ddc_c_sparse_storage_bytes(const ddc_c_sparse *sparse);

/* Exact C++ wire-format-compatible serializers. */
ddc_c_status ddc_c_serialize(const ddc_c *ddc, uint8_t *out,
                              size_t out_capacity, size_t *written);
ddc_c_status ddc_c_deserialize(ddc_c *out, const uint8_t *data,
                                size_t data_len, size_t *consumed);
ddc_c_status ddc_c_deserialize_trusted(ddc_c *out, const uint8_t *data,
                                        size_t data_len, size_t *consumed);
ddc_c_status ddc_c_serialize_packed(const ddc_c *ddc, uint8_t *out,
                                     size_t out_capacity, size_t *written);
ddc_c_status ddc_c_deserialize_packed(ddc_c *out, const uint8_t *data,
                                       size_t data_len, size_t *consumed);
ddc_c_status ddc_c_deserialize_packed_trusted(ddc_c *out, const uint8_t *data,
                                               size_t data_len, size_t *consumed);
ddc_c_status ddc_c_serialize_v4(const ddc_c *ddc, uint8_t *out,
                                 size_t out_capacity, size_t *written);
ddc_c_status ddc_c_deserialize_v4(ddc_c *out, const uint8_t *data,
                                   size_t data_len, size_t *consumed);

#ifdef __cplusplus
}
#endif

#endif
