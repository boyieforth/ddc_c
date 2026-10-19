#include "ddc_c_internal.h"

#include <limits.h>

static uint8_t ddc_c_compress_results = 0;

void ddc_c_bytes_init(ddc_c_bytes *bytes) {
    if (bytes == NULL) return;
    bytes->data = NULL;
    bytes->len = 0;
    bytes->cap = 0;
}

void ddc_c_bytes_destroy(ddc_c_bytes *bytes) {
    if (bytes == NULL) return;
    /* cap == 0 with non-NULL data marks a borrowed slice into someone else's
       arena (see ddc_c_bytes_borrow); the arena owner frees it, not us. */
    if (bytes->data != NULL && bytes->cap != 0) free(bytes->data);
    ddc_c_bytes_init(bytes);
}

/* Point bytes at [data, data + len) inside a buffer owned elsewhere, without
 * taking ownership. cap is left at 0 so ddc_c_bytes_destroy will not free it
 * and ddc_c_bytes_resize will refuse in-place growth past the slice. */
void ddc_c_bytes_borrow(ddc_c_bytes *bytes, uint8_t *data, size_t len) {
    if (bytes == NULL) return;
    bytes->data = data;
    bytes->len = len;
    bytes->cap = 0;
}

ddc_c_status ddc_c_bytes_resize(ddc_c_bytes *bytes, size_t len) {
    uint8_t *data;
    if (bytes == NULL) return DDC_C_INVALID_ARGUMENT;
    if (bytes->cap == 0 && bytes->data != NULL) {
        /* Borrowed slice: only shrinking within the existing extent is safe;
           growth would need a real allocation the caller does not own. */
        if (len <= bytes->len) { bytes->len = len; return DDC_C_OK; }
        return DDC_C_STATE_ERROR;
    }
    if (len <= bytes->cap) {
        bytes->len = len;
        return DDC_C_OK;
    }
    if (len > SIZE_MAX / 2u && bytes->cap != 0) return DDC_C_ALLOCATION_FAILED;
    data = (uint8_t *)realloc(bytes->data, len);
    if (data == NULL) return DDC_C_ALLOCATION_FAILED;
    bytes->data = data;
    bytes->len = len;
    bytes->cap = len;
    return DDC_C_OK;
}

ddc_c_status ddc_c_bytes_assign(ddc_c_bytes *bytes, const uint8_t *data,
                                 size_t len) {
    ddc_c_status status;
    if (bytes == NULL || (len != 0 && data == NULL)) return DDC_C_INVALID_ARGUMENT;
    status = ddc_c_bytes_resize(bytes, len);
    if (status != DDC_C_OK) return status;
    if (len != 0) memcpy(bytes->data, data, len);
    return DDC_C_OK;
}

void ddc_c_bytes_clear(ddc_c_bytes *bytes) {
    if (bytes != NULL) bytes->len = 0;
}

void ddc_c_bytes_shrink_to_fit(ddc_c_bytes *bytes) {
    uint8_t *data;
    if (bytes == NULL || bytes->len == bytes->cap) return;
    if (bytes->len == 0) {
        free(bytes->data);
        ddc_c_bytes_init(bytes);
        return;
    }
    data = (uint8_t *)realloc(bytes->data, bytes->len);
    if (data != NULL) {
        bytes->data = data;
        bytes->cap = bytes->len;
    }
}

void ddc_c_btv_init(ddc_c_btv *btv, uint8_t l1_fill_ones,
                    uint8_t l2_fill_ones, ddc_c_state state) {
    if (btv == NULL) return;
    /* The complete structure is zeroed first, including every ddc_c_bytes
       member. Re-initializing those six already-zero byte buffers would only
       repeat stores on the hot segment-construction path. */
    memset(btv, 0, sizeof(*btv));
    btv->state = state;
    btv->l1_fill_ones = l1_fill_ones != 0;
    btv->l2_fill_ones = l2_fill_ones != 0;
}

void ddc_c_btv_destroy(ddc_c_btv *btv) {
    if (btv == NULL) return;
    ddc_c_bytes_destroy(&btv->l2_flat);
    ddc_c_bytes_destroy(&btv->l3_bits);
    ddc_c_bytes_destroy(&btv->l2_lits);
    ddc_c_bytes_destroy(&btv->l4_bits);
    ddc_c_bytes_destroy(&btv->l3_lits);
    ddc_c_bytes_destroy(&btv->l1_lits);
    memset(btv, 0, sizeof(*btv));
}

void ddc_c_init(ddc_c *ddc) {
    if (ddc == NULL) return;
    memset(ddc, 0, sizeof(*ddc));
    ddc->segment_bits = DDC_C_DEFAULT_SEGMENT_BITS;
}

void ddc_c_destroy(ddc_c *ddc) {
    size_t i;
    if (ddc == NULL) return;
    for (i = 0; i < ddc->segment_count; ++i)
        ddc_c_btv_destroy(&ddc->segments[i]);
    free(ddc->segments);
    /* Segments above may hold l1_lits borrowed from this arena (cap == 0),
       which ddc_c_btv_destroy above deliberately did not free. */
    ddc_c_bytes_destroy(&ddc->l1_arena);
    ddc_c_init(ddc);
}

void ddc_c_sparse_init(ddc_c_sparse *sparse) {
    if (sparse == NULL) return;
    memset(sparse, 0, sizeof(*sparse));
    sparse->segment_bits = DDC_C_DEFAULT_SEGMENT_BITS;
}

void ddc_c_sparse_destroy(ddc_c_sparse *sparse) {
    size_t i;
    if (sparse == NULL) return;
    for (i = 0; i < sparse->segment_count; ++i)
        ddc_c_btv_destroy(&sparse->seg_data[i]);
    free(sparse->seg_indices);
    free(sparse->seg_data);
    ddc_c_sparse_init(sparse);
}

void ddc_c_set_compress_results(uint8_t enabled) {
    ddc_c_compress_results = enabled != 0;
}

uint8_t ddc_c_get_compress_results(void) {
    return ddc_c_compress_results;
}

static ddc_c_status ddc_c_btv_copy_into_empty(ddc_c_btv *dst,
                                                const ddc_c_btv *src) {
    ddc_c_status status;
    if (dst == NULL || src == NULL || dst == src) return DDC_C_INVALID_ARGUMENT;
    ddc_c_btv_init(dst, src->l1_fill_ones, src->l2_fill_ones, src->state);
    dst->bit_count = src->bit_count;
    dst->l2_count = src->l2_count;
    dst->l3_count = src->l3_count;
    dst->l2_lit_count = src->l2_lit_count;
    dst->l3_fill_ones = src->l3_fill_ones;
    dst->l4_count = src->l4_count;
    dst->l3_lit_count = src->l3_lit_count;
    dst->l1_lit_count = src->l1_lit_count;

    status = ddc_c_bytes_assign(&dst->l2_flat, src->l2_flat.data, src->l2_flat.len);
    if (status == DDC_C_OK) status = ddc_c_bytes_assign(&dst->l3_bits, src->l3_bits.data, src->l3_bits.len);
    if (status == DDC_C_OK) status = ddc_c_bytes_assign(&dst->l2_lits, src->l2_lits.data, src->l2_lits.len);
    if (status == DDC_C_OK) status = ddc_c_bytes_assign(&dst->l4_bits, src->l4_bits.data, src->l4_bits.len);
    if (status == DDC_C_OK) status = ddc_c_bytes_assign(&dst->l3_lits, src->l3_lits.data, src->l3_lits.len);
    if (status == DDC_C_OK) status = ddc_c_bytes_assign(&dst->l1_lits, src->l1_lits.data, src->l1_lits.len);
    if (status != DDC_C_OK) ddc_c_btv_destroy(dst);
    return status;
}

/* Overwrite dst with a copy of src, reusing dst's existing buffer capacity
 * to avoid malloc when dst already has sufficient cap for each layer.
 * dst must be a valid (possibly non-empty) btv; its existing data is
 * discarded but its heap buffers are kept and resized in place. */
ddc_c_status ddc_c_btv_overwrite(ddc_c_btv *dst, const ddc_c_btv *src) {
    ddc_c_status status;
    if (dst == NULL || src == NULL || dst == src) return DDC_C_INVALID_ARGUMENT;
    /* Update scalar metadata without touching heap pointers. */
    dst->l1_fill_ones = src->l1_fill_ones;
    dst->l2_fill_ones = src->l2_fill_ones;
    dst->l3_fill_ones = src->l3_fill_ones;
    dst->state        = src->state;
    dst->bit_count    = src->bit_count;
    dst->l2_count     = src->l2_count;
    dst->l3_count     = src->l3_count;
    dst->l4_count     = src->l4_count;
    dst->l2_lit_count = src->l2_lit_count;
    dst->l3_lit_count = src->l3_lit_count;
    dst->l1_lit_count = src->l1_lit_count;
    /* Resize (reuses cap if sufficient) then memcpy each layer. */
    status = ddc_c_bytes_assign(&dst->l2_flat,  src->l2_flat.data,  src->l2_flat.len);
    if (status == DDC_C_OK) status = ddc_c_bytes_assign(&dst->l3_bits,  src->l3_bits.data,  src->l3_bits.len);
    if (status == DDC_C_OK) status = ddc_c_bytes_assign(&dst->l2_lits,  src->l2_lits.data,  src->l2_lits.len);
    if (status == DDC_C_OK) status = ddc_c_bytes_assign(&dst->l4_bits,  src->l4_bits.data,  src->l4_bits.len);
    if (status == DDC_C_OK) status = ddc_c_bytes_assign(&dst->l3_lits,  src->l3_lits.data,  src->l3_lits.len);
    if (status == DDC_C_OK) status = ddc_c_bytes_assign(&dst->l1_lits,  src->l1_lits.data,  src->l1_lits.len);
    return status;
}

ddc_c_status ddc_c_btv_copy_fresh(ddc_c_btv *dst, const ddc_c_btv *src) {
    return ddc_c_btv_copy_into_empty(dst, src);
}

/* Transfer all heap buffers from src into dst without copying.
 * dst must be a freshly reserved (zeroed) slot. src is left zeroed.
 * Equivalent to C++ move construction: O(1) regardless of data size. */
void ddc_c_btv_move_fresh(ddc_c_btv *dst, ddc_c_btv *src) {
    *dst = *src;
    memset(src, 0, sizeof(*src));
}

ddc_c_status ddc_c_btv_copy(ddc_c_btv *dst, const ddc_c_btv *src) {
    ddc_c_status status;
    ddc_c_btv tmp;
    if (dst == NULL || src == NULL || dst == src) return DDC_C_INVALID_ARGUMENT;
    status = ddc_c_btv_copy_into_empty(&tmp, src);
    if (status != DDC_C_OK) return status;
    ddc_c_btv_destroy(dst);
    *dst = tmp;
    return DDC_C_OK;
}

void ddc_c_btv_init_all_fill(ddc_c_btv *out, size_t bit_count,
                             size_t l2_count, uint8_t l1_fill_ones) {
    if (out == NULL) return;
    ddc_c_btv_init(out, l1_fill_ones, 0, DDC_C_COMPRESSED);
    out->bit_count = bit_count;
    out->l2_count = l2_count;
    out->l3_count = DDC_C_BITS_TO_BYTES(l2_count);
}

ddc_c_status ddc_c_make_all_fill(ddc_c_btv *out, size_t bit_count,
                                  size_t l2_count, uint8_t l1_fill_ones) {
    if (out == NULL) return DDC_C_INVALID_ARGUMENT;
    ddc_c_btv_destroy(out);
    ddc_c_btv_init_all_fill(out, bit_count, l2_count, l1_fill_ones);
    return DDC_C_OK;
}

ddc_c_status ddc_c_make_decompressed_zero(ddc_c_btv *out, size_t bit_count,
                                           size_t l2_count) {
    ddc_c_status status;
    if (out == NULL) return DDC_C_INVALID_ARGUMENT;
    ddc_c_btv_destroy(out);
    ddc_c_btv_init(out, 0, 1, DDC_C_DECOMPRESSED);
    out->bit_count = bit_count;
    out->l2_count = l2_count;
    out->l3_count = DDC_C_BITS_TO_BYTES(l2_count);
    status = ddc_c_bytes_resize(&out->l1_lits, l2_count);
    if (status != DDC_C_OK) return status;
    memset(out->l1_lits.data, 0, l2_count);
    out->l1_lit_count = l2_count;
    return DDC_C_OK;
}

ddc_c_status ddc_c_btv_expand_l3(const ddc_c_btv *btv, ddc_c_bytes *out) {
    size_t i, lit_idx = 0, l3_byte_count;
    uint8_t fill;
    ddc_c_status status;
    if (btv == NULL || out == NULL) return DDC_C_INVALID_ARGUMENT;
    l3_byte_count = DDC_C_BITS_TO_BYTES(btv->l3_count);
    if (btv->state != DDC_C_COMPRESSED) {
        if (btv->l3_bits.len == l3_byte_count)
            return ddc_c_bytes_assign(out, btv->l3_bits.data, l3_byte_count);
        status = ddc_c_bytes_resize(out, l3_byte_count);
        if (status == DDC_C_OK) memset(out->data, 0, l3_byte_count);
        return status;
    }
    status = ddc_c_bytes_resize(out, l3_byte_count);
    if (status != DDC_C_OK) return status;
    fill = ddc_c_byte_fill(btv->l3_fill_ones);
    memset(out->data, fill, l3_byte_count);
    for (i = 0; i < btv->l4_count; ++i) {
        if (ddc_c_bit_lsb(btv->l4_bits.data, i)) {
            if (i < l3_byte_count && lit_idx < btv->l3_lits.len)
                out->data[i] = btv->l3_lits.data[lit_idx];
            ++lit_idx;
        }
    }
    return DDC_C_OK;
}

ddc_c_status ddc_c_btv_expand_l2(const ddc_c_btv *btv, ddc_c_bytes *out) {
    ddc_c_bytes l3;
    size_t i, lit_idx = 0, l2_byte_count;
    uint8_t fill;
    ddc_c_status status;
    if (btv == NULL || out == NULL) return DDC_C_INVALID_ARGUMENT;
    l2_byte_count = DDC_C_BITS_TO_BYTES(btv->l2_count);
    ddc_c_bytes_init(&l3);
    status = ddc_c_btv_expand_l3(btv, &l3);
    if (status != DDC_C_OK) return status;
    status = ddc_c_bytes_resize(out, l2_byte_count);
    if (status != DDC_C_OK) {
        ddc_c_bytes_destroy(&l3);
        return status;
    }
    fill = ddc_c_byte_fill(btv->l2_fill_ones);
    memset(out->data, fill, l2_byte_count);
    for (i = 0; i < btv->l3_count; ++i) {
        if (ddc_c_bit_lsb(l3.data, i)) {
            if (i < l2_byte_count && lit_idx < btv->l2_lits.len)
                out->data[i] = btv->l2_lits.data[lit_idx];
            ++lit_idx;
        }
    }
    ddc_c_bytes_destroy(&l3);
    return DDC_C_OK;
}

uint8_t ddc_c_btv_is_last_word_literal(const ddc_c_btv *btv) {
    ddc_c_bytes l3;
    size_t last_word, l2_byte_idx, l3_byte_pos, l3_bit_pos, l2_lit_idx, b;
    uint8_t l2_byte, literal;
    if (btv == NULL || btv->l2_count == 0) return 0;
    ddc_c_bytes_init(&l3);
    if (ddc_c_btv_expand_l3(btv, &l3) != DDC_C_OK) return 0;
    last_word = btv->l2_count - 1;
    l2_byte_idx = last_word / 8u;
    l3_byte_pos = l2_byte_idx / 8u;
    l3_bit_pos = l2_byte_idx % 8u;
    literal = ddc_c_bit_lsb(l3.data, l3_byte_pos);
    if (literal) {
        l2_lit_idx = 0;
        for (b = 0; b < l3_byte_pos; ++b) l2_lit_idx += ddc_c_popcount8(l3.data[b]);
        if (l3_bit_pos != 0)
            l2_lit_idx += ddc_c_popcount8((uint8_t)(l3.data[l3_byte_pos] & ((1u << l3_bit_pos) - 1u)));
        l2_byte = l2_lit_idx < btv->l2_lits.len ? btv->l2_lits.data[l2_lit_idx] : 0;
    } else {
        l2_byte = ddc_c_byte_fill(btv->l2_fill_ones);
    }
    ddc_c_bytes_destroy(&l3);
    return (uint8_t)((l2_byte >> (last_word % 8u)) & 1u);
}

ddc_c_status ddc_c_btv_compress_l3_to_l4(ddc_c_btv *btv) {
    size_t i, nonzero = 0, non_ff = 0, l3_byte_count, l4_byte_count;
    uint8_t fill;
    ddc_c_status status;
    if (btv == NULL) return DDC_C_INVALID_ARGUMENT;
    l3_byte_count = btv->l3_bits.len;
    for (i = 0; i < l3_byte_count; ++i) {
        if (btv->l3_bits.data[i] != 0) ++nonzero;
        if (btv->l3_bits.data[i] != UINT8_MAX) ++non_ff;
    }
    btv->l3_fill_ones = non_ff < nonzero;
    fill = ddc_c_byte_fill(btv->l3_fill_ones);
    l4_byte_count = DDC_C_BITS_TO_BYTES(l3_byte_count);
    status = ddc_c_bytes_resize(&btv->l4_bits, l4_byte_count);
    if (status != DDC_C_OK) return status;
    memset(btv->l4_bits.data, 0, l4_byte_count);
    status = ddc_c_bytes_resize(&btv->l3_lits,
                                btv->l3_fill_ones ? non_ff : nonzero);
    if (status != DDC_C_OK) return status;
    {
        size_t literal = 0;
        for (i = 0; i < l3_byte_count; ++i) {
            if (btv->l3_bits.data[i] != fill) {
                ddc_c_set_bit_lsb(btv->l4_bits.data, i);
                btv->l3_lits.data[literal++] = btv->l3_bits.data[i];
            }
        }
    }
    btv->l4_count = l3_byte_count;
    btv->l3_lit_count = btv->l3_lits.len;
    ddc_c_bytes_destroy(&btv->l3_bits);
    ddc_c_bytes_shrink_to_fit(&btv->l3_lits);
    return DDC_C_OK;
}

ddc_c_status ddc_c_btv_compact_l2_l3(ddc_c_btv *btv,
                                      size_t actual_l1_count) {
    size_t i, nonzero = 0, non_ff = 0, l2_byte_count, l3_byte_count;
    uint8_t fill;
    ddc_c_status status;
    if (btv == NULL || actual_l1_count > btv->l1_lits.len) return DDC_C_INVALID_ARGUMENT;
    btv->l1_lits.len = actual_l1_count;
    btv->l1_lit_count = actual_l1_count;
    l2_byte_count = DDC_C_BITS_TO_BYTES(btv->l2_count);
    for (i = 0; i < l2_byte_count; ++i) {
        if (btv->l2_flat.data[i] != 0) ++nonzero;
        if (btv->l2_flat.data[i] != UINT8_MAX) ++non_ff;
    }
    btv->l2_fill_ones = non_ff < nonzero;
    fill = ddc_c_byte_fill(btv->l2_fill_ones);
    l3_byte_count = DDC_C_BITS_TO_BYTES(l2_byte_count);
    status = ddc_c_bytes_resize(&btv->l3_bits, l3_byte_count);
    if (status != DDC_C_OK) return status;
    memset(btv->l3_bits.data, 0, l3_byte_count);
    status = ddc_c_bytes_resize(&btv->l2_lits,
                                btv->l2_fill_ones ? non_ff : nonzero);
    if (status != DDC_C_OK) return status;
    {
        size_t literal = 0;
        for (i = 0; i < l2_byte_count; ++i) {
            if (btv->l2_flat.data[i] != fill) {
                ddc_c_set_bit_lsb(btv->l3_bits.data, i);
                btv->l2_lits.data[literal++] = btv->l2_flat.data[i];
            }
        }
    }
    btv->l3_count = l2_byte_count;
    btv->l2_lit_count = btv->l2_lits.len;
    ddc_c_bytes_destroy(&btv->l2_flat);
    status = ddc_c_btv_compress_l3_to_l4(btv);
    if (status != DDC_C_OK) return status;
    btv->state = DDC_C_COMPRESSED;
    return DDC_C_OK;
}

ddc_c_status ddc_c_btv_compress(ddc_c_btv *out, const uint8_t *bits,
                                 size_t bit_count, uint8_t l1_fill_ones) {
    ddc_c_btv tmp;
    ddc_c_status status;
    size_t i, b, num_words, l2_byte_count;
    uint8_t fill;
    if (out == NULL || (bit_count != 0 && bits == NULL)) return DDC_C_INVALID_ARGUMENT;
    ddc_c_btv_init(&tmp, l1_fill_ones, 0, DDC_C_COMPRESSED);
    tmp.bit_count = bit_count;
    num_words = DDC_C_BITS_TO_BYTES(bit_count);
    if (num_words == 0) {
        ddc_c_btv_destroy(out);
        *out = tmp;
        return DDC_C_OK;
    }
    tmp.l2_count = num_words;
    l2_byte_count = DDC_C_BITS_TO_BYTES(num_words);
    status = ddc_c_bytes_resize(&tmp.l2_flat, l2_byte_count);
    if (status != DDC_C_OK) goto fail;
    memset(tmp.l2_flat.data, 0, l2_byte_count);
    /* The compressed form cannot contain more than one L1 literal per word;
       reserve that bounded maximum once instead of growing per literal. */
    status = ddc_c_bytes_resize(&tmp.l1_lits, num_words);
    if (status != DDC_C_OK) goto fail;
    fill = ddc_c_byte_fill(tmp.l1_fill_ones);
    {
        size_t literal = 0;
        for (i = 0; i < num_words; ++i) {
            uint8_t word = 0;
            for (b = 0; b < 8 && i * 8u + b < bit_count; ++b) {
                if (bits[i * 8u + b] != 0) word |= (uint8_t)(1u << (7u - b));
            }
            if (word != fill) {
                ddc_c_set_bit_lsb(tmp.l2_flat.data, i);
                tmp.l1_lits.data[literal++] = word;
            }
        }
        tmp.l1_lits.len = literal;
    }
    tmp.l1_lit_count = tmp.l1_lits.len;
    status = ddc_c_btv_compact_l2_l3(&tmp, tmp.l1_lits.len);
    if (status != DDC_C_OK) goto fail;
    ddc_c_bytes_shrink_to_fit(&tmp.l1_lits);
    ddc_c_bytes_shrink_to_fit(&tmp.l2_lits);
    ddc_c_btv_destroy(out);
    *out = tmp;
    return DDC_C_OK;
fail:
    ddc_c_btv_destroy(&tmp);
    return status;
}

ddc_c_status ddc_c_btv_compress_sparse_segment(
    ddc_c_btv *out, const uint16_t *sorted_positions, size_t position_count,
    size_t segment_bits, uint8_t l1_fill_ones) {
    uint8_t *bits;
    ddc_c_status status;
    size_t i;
    if (out == NULL || (position_count != 0 && sorted_positions == NULL)) return DDC_C_INVALID_ARGUMENT;
    if (l1_fill_ones) {
        bits = (uint8_t *)malloc(segment_bits == 0 ? 1 : segment_bits);
        if (bits == NULL && segment_bits != 0) return DDC_C_ALLOCATION_FAILED;
        memset(bits, 1, segment_bits);
        for (i = 0; i < position_count; ++i) {
            if (sorted_positions[i] >= segment_bits) { free(bits); return DDC_C_INVALID_ARGUMENT; }
            bits[sorted_positions[i]] = 0;
        }
        status = ddc_c_btv_compress(out, bits, segment_bits, 1);
        free(bits);
        return status;
    }
    /* The direct sparse construction in C++ has the same final representation
       as compress() for valid positions. Building the logical segment keeps
       its control-stream decisions byte-for-byte identical. */
    bits = (uint8_t *)calloc(segment_bits == 0 ? 1 : segment_bits, 1);
    if (bits == NULL && segment_bits != 0) return DDC_C_ALLOCATION_FAILED;
    for (i = 0; i < position_count; ++i) {
        if (sorted_positions[i] >= segment_bits) { free(bits); return DDC_C_INVALID_ARGUMENT; }
        bits[sorted_positions[i]] = 1;
    }
    status = ddc_c_btv_compress(out, bits, segment_bits, 0);
    free(bits);
    return status;
}

ddc_c_status ddc_c_btv_decompress(const ddc_c_btv *btv, uint8_t *out_bits,
                                   size_t out_capacity) {
    ddc_c_bytes l2;
    size_t i, b, lit_idx = 0;
    if (btv == NULL || (btv->bit_count != 0 && out_bits == NULL)) return DDC_C_INVALID_ARGUMENT;
    if (out_capacity < btv->bit_count) return DDC_C_BUFFER_TOO_SMALL;
    if (btv->state == DDC_C_UNCOMPRESSED) return DDC_C_STATE_ERROR;
    ddc_c_bytes_init(&l2);
    if (ddc_c_btv_expand_l2(btv, &l2) != DDC_C_OK) return DDC_C_ALLOCATION_FAILED;
    for (i = 0; i < btv->l2_count; ++i) {
        uint8_t literal = ddc_c_bit_lsb(l2.data, i);
        uint8_t word = literal && lit_idx < btv->l1_lits.len
            ? btv->l1_lits.data[lit_idx++] : ddc_c_byte_fill(btv->l1_fill_ones);
        for (b = 0; b < 8 && i * 8u + b < btv->bit_count; ++b)
            out_bits[i * 8u + b] = (uint8_t)((word >> (7u - b)) & 1u);
    }
    ddc_c_bytes_destroy(&l2);
    return DDC_C_OK;
}

size_t ddc_c_btv_popcount(const ddc_c_btv *btv) {
    size_t fill_count, count, i = 0;
    if (btv == NULL || btv->l2_count == 0 || btv->state == DDC_C_UNCOMPRESSED) return 0;
    fill_count = btv->l2_count - btv->l1_lit_count;
    count = btv->l1_fill_ones ? fill_count * 8u : 0;
#ifdef __AVX512VPOPCNTDQ__
    {
        __m512i acc = _mm512_setzero_si512();
        for (; i + 64 <= btv->l1_lits.len; i += 64) {
            __m512i chunk = _mm512_loadu_si512((const void *)(btv->l1_lits.data + i));
            acc = _mm512_add_epi64(acc, _mm512_popcnt_epi64(chunk));
        }
        count += (size_t)_mm512_reduce_add_epi64(acc);
    }
#endif
    for (; i + 8 <= btv->l1_lits.len; i += 8) {
        uint64_t word;
        memcpy(&word, btv->l1_lits.data + i, sizeof(word));
        count += ddc_c_popcount64(word);
    }
    for (; i < btv->l1_lits.len; ++i) count += ddc_c_popcount8(btv->l1_lits.data[i]);
    if (btv->bit_count % 8u != 0 && btv->l1_fill_ones && !ddc_c_btv_is_last_word_literal(btv))
        count -= 8u - (btv->bit_count % 8u);
    return count;
}

uint8_t ddc_c_btv_is_all_zero(const ddc_c_btv *btv) {
    return btv != NULL && btv->l1_lit_count == 0 && !btv->l1_fill_ones;
}

uint8_t ddc_c_btv_is_all_ones(const ddc_c_btv *btv) {
    return btv != NULL && btv->l1_lit_count == 0 && btv->l1_fill_ones;
}

size_t ddc_c_btv_num_fills(const ddc_c_btv *btv) {
    return btv == NULL || btv->l2_count < btv->l1_lit_count ? 0 : btv->l2_count - btv->l1_lit_count;
}

void ddc_c_btv_size_breakdown(const ddc_c_btv *btv,
                              ddc_c_size_info *out) {
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    if (btv == NULL) return;
    out->l3_bits = btv->l3_count;
    out->l4_bits = btv->l4_count;
    out->l3_literal_bits = btv->l3_lit_count * 8u;
    out->l2_literal_bits = btv->l2_lit_count * 8u;
    out->l1_literal_bits = btv->l1_lit_count * 8u;
    out->total_bits = out->l4_bits + out->l3_literal_bits +
                      out->l2_literal_bits + out->l1_literal_bits;
}

double ddc_c_btv_compression_ratio(const ddc_c_btv *btv) {
    ddc_c_size_info size;
    if (btv == NULL) return 0.0;
    ddc_c_btv_size_breakdown(btv, &size);
    return size.total_bits == 0 ? 0.0 : (double)btv->bit_count / (double)size.total_bits;
}

ddc_c_status ddc_c_reserve_segments(ddc_c *ddc, size_t count) {
    ddc_c_btv *segments;
    size_t i, new_cap = count;
    if (ddc == NULL) return DDC_C_INVALID_ARGUMENT;
    if (count <= ddc->segment_cap) return DDC_C_OK;
    if (ddc->segment_cap != 0 && ddc->segment_cap <= SIZE_MAX / 2u &&
        new_cap < ddc->segment_cap * 2u)
        new_cap = ddc->segment_cap * 2u;
    if (new_cap > SIZE_MAX / sizeof(*segments)) return DDC_C_ALLOCATION_FAILED;
    segments = (ddc_c_btv *)realloc(ddc->segments, new_cap * sizeof(*segments));
    if (segments == NULL) return DDC_C_ALLOCATION_FAILED;
    ddc->segments = segments;
    for (i = ddc->segment_cap; i < new_cap; ++i)
        ddc_c_btv_init(&ddc->segments[i], 0, 0, DDC_C_COMPRESSED);
    ddc->segment_cap = new_cap;
    return DDC_C_OK;
}

ddc_c_status ddc_c_push_segment_copy(ddc_c *ddc, const ddc_c_btv *segment) {
    ddc_c_status status;
    if (ddc == NULL || segment == NULL) return DDC_C_INVALID_ARGUMENT;
    status = ddc_c_reserve_segments(ddc, ddc->segment_count + 1);
    if (status != DDC_C_OK) return status;
    status = ddc_c_btv_copy(&ddc->segments[ddc->segment_count], segment);
    if (status == DDC_C_OK) ++ddc->segment_count;
    return status;
}

ddc_c_status ddc_c_push_segment_move(ddc_c *ddc, ddc_c_btv *segment) {
    ddc_c_status status;
    if (ddc == NULL || segment == NULL) return DDC_C_INVALID_ARGUMENT;
    status = ddc_c_reserve_segments(ddc, ddc->segment_count + 1);
    if (status != DDC_C_OK) return status;
    ddc_c_btv_destroy(&ddc->segments[ddc->segment_count]);
    ddc->segments[ddc->segment_count] = *segment;
    ++ddc->segment_count;
    ddc_c_btv_init(segment, 0, 0, DDC_C_COMPRESSED);
    return DDC_C_OK;
}

ddc_c_status ddc_c_compress(ddc_c *out, const uint8_t *bits,
                             size_t bit_count, uint8_t l1_fill_ones,
                             size_t segment_bits) {
    ddc_c tmp;
    size_t offset = 0;
    ddc_c_status status;
    if (out == NULL || (bit_count != 0 && bits == NULL)) return DDC_C_INVALID_ARGUMENT;
    if (segment_bits == 0) segment_bits = DDC_C_DEFAULT_SEGMENT_BITS;
    ddc_c_init(&tmp);
    tmp.bit_count = bit_count;
    tmp.segment_bits = segment_bits;
    while (offset < bit_count) {
        ddc_c_btv segment;
        size_t seg_len = segment_bits < bit_count - offset ? segment_bits : bit_count - offset;
        ddc_c_btv_init(&segment, 0, 0, DDC_C_COMPRESSED);
        status = ddc_c_btv_compress(&segment, bits + offset, seg_len, l1_fill_ones);
        if (status == DDC_C_OK) status = ddc_c_push_segment_move(&tmp, &segment);
        ddc_c_btv_destroy(&segment);
        if (status != DDC_C_OK) { ddc_c_destroy(&tmp); return status; }
        offset += seg_len;
    }
    ddc_c_destroy(out);
    *out = tmp;
    return DDC_C_OK;
}

ddc_c_status ddc_c_from_sparse_positions(ddc_c *out,
                                          const uint32_t *positions,
                                          size_t position_count,
                                          size_t num_rows,
                                          size_t segment_bits) {
    ddc_c tmp;
    uint8_t *present = NULL;
    size_t i, seg_count, s;
    ddc_c_status status;
    if (out == NULL || (position_count != 0 && positions == NULL)) return DDC_C_INVALID_ARGUMENT;
    if (segment_bits == 0) segment_bits = DDC_C_DEFAULT_SEGMENT_BITS;
    seg_count = (num_rows + segment_bits - 1u) / segment_bits;
    present = (uint8_t *)calloc(num_rows == 0 ? 1 : num_rows, 1);
    if (present == NULL && num_rows != 0) return DDC_C_ALLOCATION_FAILED;
    for (i = 0; i < position_count; ++i) {
        if ((size_t)positions[i] >= num_rows) { free(present); return DDC_C_INVALID_ARGUMENT; }
        present[positions[i]] = 1;
    }
    ddc_c_init(&tmp);
    tmp.bit_count = num_rows;
    tmp.segment_bits = segment_bits;
    status = ddc_c_reserve_segments(&tmp, seg_count);
    if (status != DDC_C_OK) { free(present); ddc_c_destroy(&tmp); return status; }
    for (s = 0; s < seg_count; ++s) {
        size_t seg_start = s * segment_bits;
        size_t seg_len = segment_bits < num_rows - seg_start ? segment_bits : num_rows - seg_start;
        size_t l2_count = DDC_C_BITS_TO_BYTES(seg_len);
        uint8_t nonempty = 0;
        ddc_c_btv segment;
        for (i = 0; i < seg_len; ++i) {
            if (present[seg_start + i]) { nonempty = 1; break; }
        }
        ddc_c_btv_init(&segment, 0, 0, DDC_C_COMPRESSED);
        if (nonempty)
            status = ddc_c_btv_compress(&segment, present + seg_start, seg_len, 0);
        else
            status = ddc_c_make_decompressed_zero(&segment, seg_len, l2_count);
        if (status == DDC_C_OK) status = ddc_c_push_segment_move(&tmp, &segment);
        ddc_c_btv_destroy(&segment);
        if (status != DDC_C_OK) { free(present); ddc_c_destroy(&tmp); return status; }
    }
    free(present);
    ddc_c_destroy(out);
    *out = tmp;
    return DDC_C_OK;
}

ddc_c_status ddc_c_decompress(const ddc_c *ddc, uint8_t *out_bits,
                               size_t out_capacity) {
    size_t i, offset = 0;
    ddc_c_status status;
    if (ddc == NULL || (ddc->bit_count != 0 && out_bits == NULL)) return DDC_C_INVALID_ARGUMENT;
    if (out_capacity < ddc->bit_count) return DDC_C_BUFFER_TOO_SMALL;
    for (i = 0; i < ddc->segment_count; ++i) {
        status = ddc_c_btv_decompress(&ddc->segments[i], out_bits + offset,
                                      out_capacity - offset);
        if (status != DDC_C_OK) return status;
        offset += ddc->segments[i].bit_count;
    }
    return offset == ddc->bit_count ? DDC_C_OK : DDC_C_MALFORMED_INPUT;
}

size_t ddc_c_popcount(const ddc_c *ddc) {
    size_t i, total = 0;
    if (ddc == NULL) return 0;
    for (i = 0; i < ddc->segment_count; ++i) total += ddc_c_btv_popcount(&ddc->segments[i]);
    return total;
}

void ddc_c_size_breakdown(const ddc_c *ddc, ddc_c_size_info *out) {
    size_t i;
    if (out == NULL) return;
    memset(out, 0, sizeof(*out));
    if (ddc == NULL) return;
    for (i = 0; i < ddc->segment_count; ++i) {
        ddc_c_size_info part;
        ddc_c_btv_size_breakdown(&ddc->segments[i], &part);
        out->l3_bits += part.l3_bits;
        out->l4_bits += part.l4_bits;
        out->l3_literal_bits += part.l3_literal_bits;
        out->l2_literal_bits += part.l2_literal_bits;
        out->l1_literal_bits += part.l1_literal_bits;
        out->total_bits += part.total_bits;
    }
}

double ddc_c_compression_ratio(const ddc_c *ddc) {
    ddc_c_size_info size;
    if (ddc == NULL) return 0.0;
    ddc_c_size_breakdown(ddc, &size);
    return size.total_bits == 0 ? 0.0 : (double)ddc->bit_count / (double)size.total_bits;
}

static ddc_c_status ddc_c_btv_for_each_set_bit(const ddc_c_btv *btv,
                                                size_t base,
                                                ddc_c_set_bit_fn fn,
                                                void *ctx) {
    uint8_t *bits;
    size_t position;
    ddc_c_status status;

    if (btv == NULL || fn == NULL) return DDC_C_INVALID_ARGUMENT;
    if (btv->bit_count == 0) return DDC_C_OK;
    bits = (uint8_t *)malloc(btv->bit_count);
    if (bits == NULL) return DDC_C_ALLOCATION_FAILED;
    status = ddc_c_btv_decompress(btv, bits, btv->bit_count);
    if (status == DDC_C_OK) {
        for (position = 0; position < btv->bit_count; ++position) {
            if (bits[position] != 0) fn(ctx, base + position);
        }
    }
    free(bits);
    return status;
}

ddc_c_status ddc_c_for_each_set_bit(const ddc_c *ddc,
                                    ddc_c_set_bit_fn fn, void *ctx) {
    size_t segment, offset = 0;
    if (ddc == NULL || fn == NULL ||
        (ddc->segment_count != 0 && ddc->segments == NULL))
        return DDC_C_INVALID_ARGUMENT;
    for (segment = 0; segment < ddc->segment_count; ++segment) {
        ddc_c_status status;
        const ddc_c_btv *btv = &ddc->segments[segment];
        if (btv->bit_count > ddc->bit_count - offset)
            return DDC_C_MALFORMED_INPUT;
        status = ddc_c_btv_for_each_set_bit(btv, offset, fn, ctx);
        if (status != DDC_C_OK) return status;
        offset += btv->bit_count;
    }
    return offset == ddc->bit_count ? DDC_C_OK : DDC_C_MALFORMED_INPUT;
}

typedef struct ddc_c_position_count_ctx {
    size_t count;
} ddc_c_position_count_ctx;

static void ddc_c_count_set_bit(void *ctx, size_t position) {
    ddc_c_position_count_ctx *count = (ddc_c_position_count_ctx *)ctx;
    (void)position;
    ++count->count;
}

typedef struct ddc_c_position_write_ctx {
    size_t *positions;
    size_t index;
} ddc_c_position_write_ctx;

static void ddc_c_write_set_bit(void *ctx, size_t position) {
    ddc_c_position_write_ctx *write = (ddc_c_position_write_ctx *)ctx;
    write->positions[write->index++] = position;
}

ddc_c_status ddc_c_set_bit_positions(const ddc_c *ddc, size_t *positions,
                                     size_t positions_capacity,
                                     size_t *position_count) {
    ddc_c_position_count_ctx count = {0};
    ddc_c_position_write_ctx write;
    ddc_c_status status;
    if (ddc == NULL || position_count == NULL ||
        (positions_capacity != 0 && positions == NULL))
        return DDC_C_INVALID_ARGUMENT;
    status = ddc_c_for_each_set_bit(ddc, ddc_c_count_set_bit, &count);
    if (status != DDC_C_OK) return status;
    *position_count = count.count;
    if (positions_capacity < count.count) return DDC_C_BUFFER_TOO_SMALL;
    write.positions = positions;
    write.index = 0;
    return ddc_c_for_each_set_bit(ddc, ddc_c_write_set_bit, &write);
}

ddc_c_status ddc_c_for_each_literal(const ddc_c *ddc,
                                    ddc_c_literal_fn fn, void *ctx) {
    size_t s, word_off = 0;
    if (ddc == NULL || fn == NULL) return DDC_C_INVALID_ARGUMENT;
    for (s = 0; s < ddc->segment_count; ++s) {
        const ddc_c_btv *seg = &ddc->segments[s];
        ddc_c_bytes l2;
        size_t w, lit_idx = 0;
        ddc_c_status status;
        if (ddc_c_btv_is_all_zero(seg)) { word_off += seg->l2_count; continue; }
        if (seg->state == DDC_C_DECOMPRESSED) {
            for (w = 0; w < seg->l2_count; ++w)
                if (seg->l1_lits.data[w] != 0) fn(ctx, (uint32_t)(word_off + w), seg->l1_lits.data[w]);
            word_off += seg->l2_count;
            continue;
        }
        ddc_c_bytes_init(&l2);
        status = ddc_c_btv_expand_l2(seg, &l2);
        if (status != DDC_C_OK) return status;
        for (w = 0; w < seg->l2_count; ++w) {
            uint8_t value = ddc_c_byte_fill(seg->l1_fill_ones);
            if (ddc_c_bit_lsb(l2.data, w)) value = seg->l1_lits.data[lit_idx++];
            if (value != 0) fn(ctx, (uint32_t)(word_off + w), value);
        }
        ddc_c_bytes_destroy(&l2);
        word_off += seg->l2_count;
    }
    return DDC_C_OK;
}
