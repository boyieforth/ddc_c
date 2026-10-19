#ifndef DDC_C_INTERNAL_H
#define DDC_C_INTERNAL_H

#include "../include/ddc_c.h"

#include <stdlib.h>
#include <string.h>

#ifdef __AVX512F__
#include <immintrin.h>
#endif

#define DDC_C_BITS_TO_BYTES(bits) (((bits) + 7u) / 8u)

static inline unsigned ddc_c_popcount8(uint8_t value) {
    return (unsigned)__builtin_popcount((unsigned)value);
}

static inline size_t ddc_c_popcount64(uint64_t value) {
    return (size_t)__builtin_popcountll(value);
}

static inline uint8_t ddc_c_bit_lsb(const uint8_t *bytes, size_t index) {
    return (uint8_t)((bytes[index / 8u] >> (index % 8u)) & 1u);
}

static inline void ddc_c_set_bit_lsb(uint8_t *bytes, size_t index) {
    bytes[index / 8u] |= (uint8_t)(1u << (index % 8u));
}

static inline uint8_t ddc_c_byte_fill(uint8_t ones) {
    return ones ? UINT8_MAX : 0;
}

void ddc_c_bytes_init(ddc_c_bytes *bytes);
void ddc_c_bytes_destroy(ddc_c_bytes *bytes);
/* Borrow [data, data + len) from a buffer owned elsewhere; cap stays 0 so
 * ddc_c_bytes_destroy will not free it. See ddc_c.h's ddc_c::l1_arena. */
void ddc_c_bytes_borrow(ddc_c_bytes *bytes, uint8_t *data, size_t len);
ddc_c_status ddc_c_bytes_resize(ddc_c_bytes *bytes, size_t len);
ddc_c_status ddc_c_bytes_assign(ddc_c_bytes *bytes, const uint8_t *data,
                                 size_t len);
void ddc_c_bytes_clear(ddc_c_bytes *bytes);
void ddc_c_bytes_shrink_to_fit(ddc_c_bytes *bytes);

ddc_c_status ddc_c_btv_copy(ddc_c_btv *dst, const ddc_c_btv *src);
ddc_c_status ddc_c_btv_copy_fresh(ddc_c_btv *dst, const ddc_c_btv *src);
/* Overwrite dst with src, reusing dst's existing buffer capacity (no malloc
 * when dst->*.cap >= src->*.len). dst's old data is replaced but its heap
 * buffers are kept. Safe to call on a non-empty dst. */
ddc_c_status ddc_c_btv_overwrite(ddc_c_btv *dst, const ddc_c_btv *src);
/* O(1) move: transfers all heap buffers from src to dst (a fresh slot).
 * src is left zeroed. Mirrors C++ move construction / std::move semantics. */
void ddc_c_btv_move_fresh(ddc_c_btv *dst, ddc_c_btv *src);
ddc_c_status ddc_c_make_all_fill(ddc_c_btv *out, size_t bit_count,
                                  size_t l2_count, uint8_t l1_fill_ones);
void ddc_c_btv_init_all_fill(ddc_c_btv *out, size_t bit_count,
                             size_t l2_count, uint8_t l1_fill_ones);
ddc_c_status ddc_c_make_decompressed_zero(ddc_c_btv *out, size_t bit_count,
                                           size_t l2_count);
ddc_c_status ddc_c_btv_expand_l3(const ddc_c_btv *btv, ddc_c_bytes *out);
ddc_c_status ddc_c_btv_expand_l2(const ddc_c_btv *btv, ddc_c_bytes *out);
ddc_c_status ddc_c_btv_compact_l2_l3(ddc_c_btv *btv,
                                      size_t actual_l1_count);
ddc_c_status ddc_c_btv_compress_l3_to_l4(ddc_c_btv *btv);
uint8_t ddc_c_btv_is_last_word_literal(const ddc_c_btv *btv);

typedef enum ddc_c_op {
    DDC_C_OP_AND,
    DDC_C_OP_OR,
    DDC_C_OP_XOR
} ddc_c_op;

/* l1_slice, when non-NULL, lets the caller supply a pre-sized arena region
 * (exactly a->l2_count bytes, decompressed-result case only) for the L1
 * output buffer instead of a per-call allocation; pass NULL to allocate. */
ddc_c_status ddc_c_btv_binary(ddc_c_btv *out, const ddc_c_btv *a,
                               const ddc_c_btv *b, ddc_c_op op,
                               uint8_t bypass, ddc_c_bytes *l1_slice);
ddc_c_status ddc_c_btv_binary_inplace(ddc_c_btv *lhs,
                                       const ddc_c_btv *rhs, ddc_c_op op);
ddc_c_status ddc_c_binary(ddc_c *out, const ddc_c *a, const ddc_c *b,
                           ddc_c_op op, uint8_t bypass);
ddc_c_status ddc_c_binary_inplace(ddc_c *lhs, const ddc_c *rhs,
                                   ddc_c_op op);

ddc_c_status ddc_c_reserve_segments(ddc_c *ddc, size_t count);
ddc_c_status ddc_c_push_segment_copy(ddc_c *ddc, const ddc_c_btv *segment);
ddc_c_status ddc_c_push_segment_move(ddc_c *ddc, ddc_c_btv *segment);

#endif
