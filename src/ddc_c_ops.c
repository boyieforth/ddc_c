#include "ddc_c_internal.h"

static uint8_t ddc_c_apply_byte_op(uint8_t a, uint8_t b, ddc_c_op op) {
    switch (op) {
    case DDC_C_OP_AND: return (uint8_t)(a & b);
    case DDC_C_OP_OR:  return (uint8_t)(a | b);
    default:           return (uint8_t)(a ^ b);
    }
}

/* l1_slice, when non-NULL, must point to an untouched arena region of
 * exactly l2_count bytes; the result's L1 buffer borrows it instead of
 * allocating its own (see ddc_c_binary's arena setup). Only decompressed
 * results are arena-backed; compressed results always allocate their own
 * L1 buffer since compaction later shrinks it to a data-dependent size. */
static ddc_c_status ddc_c_make_binary_result(ddc_c_btv *out,
                                              size_t bit_count,
                                              size_t l2_count,
                                              uint8_t compressed,
                                              ddc_c_bytes *l1_slice) {
    ddc_c_status status;
    if (out == NULL) return DDC_C_INVALID_ARGUMENT;
    ddc_c_btv_destroy(out);
    ddc_c_btv_init(out, 0, compressed ? 0 : 1,
                   compressed ? DDC_C_COMPRESSED : DDC_C_DECOMPRESSED);
    out->bit_count = bit_count;
    out->l2_count = l2_count;
    if (compressed) {
        status = ddc_c_bytes_resize(&out->l2_flat, DDC_C_BITS_TO_BYTES(l2_count));
        if (status != DDC_C_OK) return status;
        memset(out->l2_flat.data, 0, out->l2_flat.len);
    } else {
        out->l3_count = DDC_C_BITS_TO_BYTES(l2_count);
        out->l2_lit_count = 0;
    }
    if (!compressed && l1_slice != NULL) {
        ddc_c_bytes_borrow(&out->l1_lits, l1_slice->data, l2_count);
    } else {
        status = ddc_c_bytes_resize(&out->l1_lits, l2_count);
        if (status != DDC_C_OK) return status;
    }
    out->l1_lit_count = l2_count;
    return DDC_C_OK;
}

#ifdef __AVX512VBMI2__
typedef struct ddc_c_side_ctx {
    const uint8_t *l4_bits;
    const uint8_t *l3_lits;
    const uint8_t *l2_lits;
    const uint8_t *l1_lits;
    uint8_t l3_fill_ones;
    uint8_t l2_fill_ones;
    uint8_t l1_fill_ones;
    __m512i l3_fill_vec;
    __m512i l2_fill_vec;
    __m512i l1_fill_vec;
    size_t l3_lit_off;
    size_t l2_lit_off;
    size_t l1_lit_off;
} ddc_c_side_ctx;

static ddc_c_side_ctx ddc_c_make_side(const ddc_c_btv *btv) {
    ddc_c_side_ctx side;
    side.l4_bits = btv->l4_bits.data;
    side.l3_lits = btv->l3_lits.data;
    side.l2_lits = btv->l2_lits.data;
    side.l1_lits = btv->l1_lits.data;
    side.l3_fill_ones = btv->l3_fill_ones;
    side.l2_fill_ones = btv->l2_fill_ones;
    side.l1_fill_ones = btv->l1_fill_ones;
    side.l3_fill_vec = side.l3_fill_ones ? _mm512_set1_epi8((char)-1) : _mm512_setzero_si512();
    side.l2_fill_vec = side.l2_fill_ones ? _mm512_set1_epi8((char)-1) : _mm512_setzero_si512();
    side.l1_fill_vec = side.l1_fill_ones ? _mm512_set1_epi8((char)-1) : _mm512_setzero_si512();
    side.l3_lit_off = 0;
    side.l2_lit_off = 0;
    side.l1_lit_off = 0;
    return side;
}

/* Exact L4 -> L3 -> L2 -> L1 descent used by the C++ VBMI2 kernels.
 * Processes 64 regions per batch (matching C++ batch geometry).  A batch
 * whose combined L4 mask is all-zero is skipped entirely for OR/XOR; AND
 * additionally skips when either side's L4 is all-zero (all-zero result). */
static ddc_c_status ddc_c_btv_binary_vbmi2(ddc_c_btv *out,
                                            const ddc_c_btv *a,
                                            const ddc_c_btv *b,
                                            ddc_c_op op,
                                            ddc_c_bytes *l1_slice) {
    ddc_c_side_ctx sa, sb;
    uint8_t l3_buf_a[64] __attribute__((aligned(64)));
    uint8_t l3_buf_b[64] __attribute__((aligned(64)));
    const uint8_t compress = ddc_c_get_compress_results();
    const size_t regions = a->l2_count / DDC_C_WORDS_PER_REG;
    const size_t batch_count = (regions + 63u) / 64u;
    size_t batch, r_off = 0;
    ddc_c_status status;

    /* Structural-zero flags used for batch-level bypass (mirrors C++ OR/AND). */
    const uint8_t a_zero_when_l3z = !a->l1_fill_ones && !a->l2_fill_ones;
    const uint8_t b_zero_when_l3z = !b->l1_fill_ones && !b->l2_fill_ones;
    const uint8_t a_struct_zero   = a_zero_when_l3z  && !a->l3_fill_ones;
    const uint8_t b_struct_zero   = b_zero_when_l3z  && !b->l3_fill_ones;

    if (a->l4_count == 0 || b->l4_count == 0) return DDC_C_STATE_ERROR;
    status = ddc_c_make_binary_result(out, a->bit_count, a->l2_count, compress, l1_slice);
    if (status != DDC_C_OK) return status;
    sa = ddc_c_make_side(a);
    sb = ddc_c_make_side(b);

    for (batch = 0; batch < batch_count; ++batch) {
        const size_t batch_start = batch * 64u;
        const size_t batch_end   = batch_start + 64u < regions ? batch_start + 64u : regions;
        const size_t batch_size  = batch_end - batch_start;
        uint64_t a_l4_mask = 0, b_l4_mask = 0;
        size_t region;

        /* Read up to 8 bytes of L4 bits covering this batch. */
        memcpy(&a_l4_mask, sa.l4_bits + batch_start / 8u, (batch_size + 7u) / 8u);
        memcpy(&b_l4_mask, sb.l4_bits + batch_start / 8u, (batch_size + 7u) / 8u);
        if (batch_size < 64u) {
            const uint64_t valid = (uint64_t)1u << batch_size;
            const uint64_t mask  = valid - 1u;
            a_l4_mask &= mask;
            b_l4_mask &= mask;
        }

        /* Batch-level skip: for OR/XOR both sides empty → zero result (no-op
           in compressed mode, zero-fill in decompressed mode).
           For AND: either side empty → zero result, skip the other side too. */
        if (op == DDC_C_OP_OR || op == DDC_C_OP_XOR) {
            if (a_struct_zero && b_struct_zero && a_l4_mask == 0 && b_l4_mask == 0) {
                if (!compress) {
                    memset(out->l1_lits.data + r_off, 0, batch_size * 64u);
                    r_off += batch_size * 64u;
                }
                continue;
            }
        } else { /* AND */
            const uint8_t a_batch_zero = a_struct_zero && a_l4_mask == 0;
            const uint8_t b_batch_zero = b_struct_zero && b_l4_mask == 0;
            if (a_batch_zero || b_batch_zero) {
                /* Advance the non-zero side's literal cursors, then skip. */
                if (!a_batch_zero) {
                    __m512i l3a_chunk = _mm512_mask_expandloadu_epi8(
                        _mm512_setzero_si512(), (__mmask64)a_l4_mask,
                        sa.l3_lits + sa.l3_lit_off);
                    sa.l3_lit_off += ddc_c_popcount64(a_l4_mask);
                    _mm512_store_si512((void *)l3_buf_a, l3a_chunk);
                    {
                        size_t r;
                        for (r = 0; r < batch_size; ++r) {
                            __m512i l2v = _mm512_mask_expandloadu_epi8(
                                sa.l2_fill_vec, (__mmask64)l3_buf_a[r],
                                sa.l2_lits + sa.l2_lit_off);
                            sa.l2_lit_off += ddc_c_popcount8(l3_buf_a[r]);
                            sa.l1_lit_off += ddc_c_popcount64((uint64_t)
                                (__mmask64)_mm_cvtsi128_si64(_mm512_castsi512_si128(l2v)));
                        }
                    }
                }
                if (!b_batch_zero) {
                    __m512i l3b_chunk = _mm512_mask_expandloadu_epi8(
                        _mm512_setzero_si512(), (__mmask64)b_l4_mask,
                        sb.l3_lits + sb.l3_lit_off);
                    sb.l3_lit_off += ddc_c_popcount64(b_l4_mask);
                    _mm512_store_si512((void *)l3_buf_b, l3b_chunk);
                    {
                        size_t r;
                        for (r = 0; r < batch_size; ++r) {
                            __m512i l2v = _mm512_mask_expandloadu_epi8(
                                sb.l2_fill_vec, (__mmask64)l3_buf_b[r],
                                sb.l2_lits + sb.l2_lit_off);
                            sb.l2_lit_off += ddc_c_popcount8(l3_buf_b[r]);
                            sb.l1_lit_off += ddc_c_popcount64((uint64_t)
                                (__mmask64)_mm_cvtsi128_si64(_mm512_castsi512_si128(l2v)));
                        }
                    }
                }
                if (!compress) {
                    memset(out->l1_lits.data + r_off, 0, batch_size * 64u);
                    r_off += batch_size * 64u;
                }
                continue;
            }
        }

        /* Expand L3 for the whole batch at once using VBMI2 mask-expand,
         * exactly mirroring the C++ SideCtx batch geometry.
         * This avoids 64 individual ddc_c_bit_lsb() calls per batch. */
        {
            __m512i l3a_chunk = _mm512_mask_expandloadu_epi8(sa.l3_fill_vec,
                (__mmask64)a_l4_mask, sa.l3_lits + sa.l3_lit_off);
            __m512i l3b_chunk = _mm512_mask_expandloadu_epi8(sb.l3_fill_vec,
                (__mmask64)b_l4_mask, sb.l3_lits + sb.l3_lit_off);
            sa.l3_lit_off += ddc_c_popcount64(a_l4_mask);
            sb.l3_lit_off += ddc_c_popcount64(b_l4_mask);
            _mm512_store_si512((void *)l3_buf_a, l3a_chunk);
            _mm512_store_si512((void *)l3_buf_b, l3b_chunk);
        }

        /* Per-region work within this batch. */
        for (region = 0; region < batch_size; ++region) {
            const uint8_t l3a = l3_buf_a[region];
            const uint8_t l3b = l3_buf_b[region];
            __m512i l2a = _mm512_mask_expandloadu_epi8(sa.l2_fill_vec, (__mmask64)l3a,
                                                        sa.l2_lits + sa.l2_lit_off);
            __m512i l2b = _mm512_mask_expandloadu_epi8(sb.l2_fill_vec, (__mmask64)l3b,
                                                        sb.l2_lits + sb.l2_lit_off);
            __mmask64 ma, mb;
            __m512i va, vb, vr;
            sa.l2_lit_off += ddc_c_popcount8(l3a);
            sb.l2_lit_off += ddc_c_popcount8(l3b);
            ma = (__mmask64)_mm_cvtsi128_si64(_mm512_castsi512_si128(l2a));
            mb = (__mmask64)_mm_cvtsi128_si64(_mm512_castsi512_si128(l2b));
            va = _mm512_mask_expandloadu_epi8(sa.l1_fill_vec, ma, sa.l1_lits + sa.l1_lit_off);
            vb = _mm512_mask_expandloadu_epi8(sb.l1_fill_vec, mb, sb.l1_lits + sb.l1_lit_off);
            sa.l1_lit_off += ddc_c_popcount64((uint64_t)ma);
            sb.l1_lit_off += ddc_c_popcount64((uint64_t)mb);
            if (op == DDC_C_OP_AND) vr = _mm512_and_si512(va, vb);
            else if (op == DDC_C_OP_OR) vr = _mm512_or_si512(va, vb);
            else vr = _mm512_xor_si512(va, vb);
            if (compress) {
                __mmask64 literals = _mm512_test_epi8_mask(vr, vr);
                uint64_t marker = (uint64_t)literals;
                memcpy(out->l2_flat.data + (batch_start + region) * 8u, &marker, sizeof(marker));
                _mm512_mask_compressstoreu_epi8(out->l1_lits.data + r_off, literals, vr);
                r_off += ddc_c_popcount64(marker);
            } else {
                _mm512_storeu_si512((void *)(out->l1_lits.data + r_off), vr);
                r_off += 64u;
            }
        }
    }
    /* Scalar tail for the partial region (< 64 words) after the last full region. */
    if (regions * DDC_C_WORDS_PER_REG < a->l2_count) {
        const uint8_t l3a = ddc_c_bit_lsb(sa.l4_bits, regions)
            ? sa.l3_lits[sa.l3_lit_off++] : ddc_c_byte_fill(sa.l3_fill_ones);
        const uint8_t l3b = ddc_c_bit_lsb(sb.l4_bits, regions)
            ? sb.l3_lits[sb.l3_lit_off++] : ddc_c_byte_fill(sb.l3_fill_ones);
        size_t word = regions * DDC_C_WORDS_PER_REG;
        size_t group;
        for (group = 0; word < a->l2_count; ++group) {
            const uint8_t l2a_byte = (l3a >> group) & 1u
                ? sa.l2_lits[sa.l2_lit_off++] : ddc_c_byte_fill(sa.l2_fill_ones);
            const uint8_t l2b_byte = (l3b >> group) & 1u
                ? sb.l2_lits[sb.l2_lit_off++] : ddc_c_byte_fill(sb.l2_fill_ones);
            size_t bit;
            for (bit = 0; bit < 8 && word < a->l2_count; ++bit, ++word) {
                const uint8_t va = (l2a_byte >> bit) & 1u
                    ? sa.l1_lits[sa.l1_lit_off++] : ddc_c_byte_fill(sa.l1_fill_ones);
                const uint8_t vb = (l2b_byte >> bit) & 1u
                    ? sb.l1_lits[sb.l1_lit_off++] : ddc_c_byte_fill(sb.l1_fill_ones);
                const uint8_t vr = ddc_c_apply_byte_op(va, vb, op);
                if (compress) {
                    if (vr != 0) {
                        ddc_c_set_bit_lsb(out->l2_flat.data, word);
                        out->l1_lits.data[r_off++] = vr;
                    }
                } else out->l1_lits.data[r_off++] = vr;
            }
        }
    }
    if (compress) return ddc_c_btv_compact_l2_l3(out, r_off);
    out->l1_lit_count = out->l1_lits.len;
    return DDC_C_OK;
}
#endif

/*
 * Scalar reference kernel. It expands precisely the same L3/L2 marker
 * hierarchy as the C++ scalar fallback and emits either a dense decompressed
 * result or a compacted control hierarchy according to ddc_compress_results.
 */
ddc_c_status ddc_c_btv_binary(ddc_c_btv *out, const ddc_c_btv *a,
                               const ddc_c_btv *b, ddc_c_op op,
                               uint8_t bypass, ddc_c_bytes *l1_slice) {
    ddc_c_bytes l2a, l2b;
    ddc_c_status status;
    size_t word, a_lit_off = 0, b_lit_off = 0, r_off = 0;
    uint8_t compress;
    (void)bypass;
    if (out == NULL || a == NULL || b == NULL || out == a || out == b)
        return DDC_C_INVALID_ARGUMENT;
    if (a->bit_count != b->bit_count || a->l2_count != b->l2_count)
        return DDC_C_INVALID_ARGUMENT;
    if (a->state == DDC_C_UNCOMPRESSED || b->state == DDC_C_UNCOMPRESSED)
        return DDC_C_STATE_ERROR;
    if (a->bit_count == 0) {
        ddc_c_btv_destroy(out);
        ddc_c_btv_init(out, 0, 0, DDC_C_COMPRESSED);
        return DDC_C_OK;
    }
#ifdef __AVX512VBMI2__
    if (a->state == DDC_C_COMPRESSED && b->state == DDC_C_COMPRESSED)
        return ddc_c_btv_binary_vbmi2(out, a, b, op, l1_slice);
#endif
    compress = ddc_c_get_compress_results();
    status = ddc_c_make_binary_result(out, a->bit_count, a->l2_count, compress, l1_slice);
    if (status != DDC_C_OK) return status;
    ddc_c_bytes_init(&l2a);
    ddc_c_bytes_init(&l2b);
    status = ddc_c_btv_expand_l2(a, &l2a);
    if (status == DDC_C_OK) status = ddc_c_btv_expand_l2(b, &l2b);
    if (status != DDC_C_OK) goto fail;

#ifdef __AVX512F__
    /* On fully decompressed operands use the same 64-byte AVX-512 geometry
       as the C++ in-place fast path; the scalar loop below handles tails and
       compressed source hierarchies. */
    if (!compress && a->state == DDC_C_DECOMPRESSED && b->state == DDC_C_DECOMPRESSED) {
        const size_t region_count = a->l2_count / DDC_C_WORDS_PER_REG;
        size_t region;
        for (region = 0; region < region_count; ++region) {
            const uint8_t *pa = a->l1_lits.data + region * 64u;
            const uint8_t *pb = b->l1_lits.data + region * 64u;
            uint8_t *pr = out->l1_lits.data + region * 64u;
            __m512i va = _mm512_loadu_si512((const void *)pa);
            __m512i vb = _mm512_loadu_si512((const void *)pb);
            __m512i vr;
            if (op == DDC_C_OP_AND) vr = _mm512_and_si512(va, vb);
            else if (op == DDC_C_OP_OR) vr = _mm512_or_si512(va, vb);
            else vr = _mm512_xor_si512(va, vb);
            _mm512_storeu_si512((void *)pr, vr);
        }
        r_off = region_count * 64u;
        a_lit_off = r_off;
        b_lit_off = r_off;
        word = r_off;
        for (; word < a->l2_count; ++word)
            out->l1_lits.data[r_off++] = ddc_c_apply_byte_op(
                a->l1_lits.data[a_lit_off++], b->l1_lits.data[b_lit_off++], op);
        out->l1_lit_count = out->l1_lits.len;
        ddc_c_bytes_destroy(&l2a);
        ddc_c_bytes_destroy(&l2b);
        return DDC_C_OK;
    }
#endif

    for (word = 0; word < a->l2_count; ++word) {
        uint8_t wa = ddc_c_byte_fill(a->l1_fill_ones);
        uint8_t wb = ddc_c_byte_fill(b->l1_fill_ones);
        uint8_t wr;
        if (ddc_c_bit_lsb(l2a.data, word)) wa = a->l1_lits.data[a_lit_off++];
        if (ddc_c_bit_lsb(l2b.data, word)) wb = b->l1_lits.data[b_lit_off++];
        wr = ddc_c_apply_byte_op(wa, wb, op);
        if (compress) {
            if (wr != 0) {
                ddc_c_set_bit_lsb(out->l2_flat.data, word);
                out->l1_lits.data[r_off++] = wr;
            }
        } else {
            out->l1_lits.data[r_off++] = wr;
        }
    }
    if (compress) {
        status = ddc_c_btv_compact_l2_l3(out, r_off);
        if (status != DDC_C_OK) goto fail;
    } else {
        out->l1_lit_count = out->l1_lits.len;
    }
    ddc_c_bytes_destroy(&l2a);
    ddc_c_bytes_destroy(&l2b);
    return DDC_C_OK;
fail:
    ddc_c_bytes_destroy(&l2a);
    ddc_c_bytes_destroy(&l2b);
    ddc_c_btv_destroy(out);
    return status;
}

static ddc_c_status ddc_c_btv_or_inplace_fast(ddc_c_btv *lhs,
                                                const ddc_c_btv *rhs) {
    size_t word = 0;
    if (lhs == NULL || rhs == NULL) return DDC_C_INVALID_ARGUMENT;
    if (lhs->state != DDC_C_DECOMPRESSED || rhs->state == DDC_C_UNCOMPRESSED)
        return DDC_C_STATE_ERROR;
    if (lhs->bit_count != rhs->bit_count || lhs->l2_count != rhs->l2_count)
        return DDC_C_INVALID_ARGUMENT;
    if (lhs->l1_lits.len < lhs->l2_count) return DDC_C_STATE_ERROR;

    if (rhs->state == DDC_C_DECOMPRESSED) {
        if (rhs->l1_lits.len < rhs->l2_count) return DDC_C_STATE_ERROR;
#ifdef __AVX512F__
        for (; word + 64u <= lhs->l2_count; word += 64u) {
            __m512i a = _mm512_loadu_si512((const void *)(lhs->l1_lits.data + word));
            __m512i b = _mm512_loadu_si512((const void *)(rhs->l1_lits.data + word));
            _mm512_storeu_si512((void *)(lhs->l1_lits.data + word),
                                _mm512_or_si512(a, b));
        }
#endif
        for (; word < lhs->l2_count; ++word)
            lhs->l1_lits.data[word] |= rhs->l1_lits.data[word];
        return DDC_C_OK;
    }

#ifdef __AVX512VBMI2__
    {
        const size_t regions = lhs->l2_count / DDC_C_WORDS_PER_REG;
        ddc_c_side_ctx side = ddc_c_make_side(rhs);
        const uint8_t zero_when_l3_zero = !side.l1_fill_ones && !side.l2_fill_ones;
        const uint8_t l4_skipable = zero_when_l3_zero && !side.l3_fill_ones;
        const uint8_t l3_fill = ddc_c_byte_fill(side.l3_fill_ones);
        size_t region;

        if (rhs->l4_count == 0) return DDC_C_STATE_ERROR;
        for (region = 0; region < regions; ++region) {
            uint8_t l3;
            __m512i l2v;
            __mmask64 mask;
            __m512i right;
            __m512i left;

            if (l4_skipable && (region & 63u) == 0 && region + 64u <= regions) {
                uint64_t l4_chunk = 0;
                memcpy(&l4_chunk, side.l4_bits + region / 8u, sizeof(l4_chunk));
                if (l4_chunk == 0) {
                    region += 63u;
                    continue;
                }
            }

            l3 = ddc_c_bit_lsb(side.l4_bits, region)
                ? side.l3_lits[side.l3_lit_off++] : l3_fill;
            if (zero_when_l3_zero && l3 == 0) continue;

            l2v = _mm512_mask_expandloadu_epi8(side.l2_fill_vec,
                                               (__mmask64)l3,
                                               side.l2_lits + side.l2_lit_off);
            side.l2_lit_off += ddc_c_popcount8(l3);
            mask = (__mmask64)_mm_cvtsi128_si64(_mm512_castsi512_si128(l2v));
            right = _mm512_mask_expandloadu_epi8(side.l1_fill_vec, mask,
                                                 side.l1_lits + side.l1_lit_off);
            side.l1_lit_off += ddc_c_popcount64((uint64_t)mask);

            left = _mm512_loadu_si512((const void *)(lhs->l1_lits.data + region * 64u));
            _mm512_storeu_si512((void *)(lhs->l1_lits.data + region * 64u),
                                _mm512_or_si512(left, right));
        }

        word = regions * DDC_C_WORDS_PER_REG;
        if (word < lhs->l2_count) {
            uint8_t l3 = ddc_c_bit_lsb(side.l4_bits, regions)
                ? side.l3_lits[side.l3_lit_off++] : l3_fill;
            size_t group = 0;
            for (; word < lhs->l2_count; ++group) {
                uint8_t l2 = ((l3 >> group) & 1u)
                    ? side.l2_lits[side.l2_lit_off++]
                    : ddc_c_byte_fill(side.l2_fill_ones);
                size_t bit;
                for (bit = 0; bit < 8u && word < lhs->l2_count; ++bit, ++word) {
                    uint8_t value = ((l2 >> bit) & 1u)
                        ? side.l1_lits[side.l1_lit_off++]
                        : ddc_c_byte_fill(side.l1_fill_ones);
                    lhs->l1_lits.data[word] |= value;
                }
            }
        }
        return DDC_C_OK;
    }
#else
    {
        ddc_c_bytes l2;
        size_t lit_off = 0;
        ddc_c_status status;
        uint8_t fill = ddc_c_byte_fill(rhs->l1_fill_ones);
        ddc_c_bytes_init(&l2);
        status = ddc_c_btv_expand_l2(rhs, &l2);
        if (status != DDC_C_OK) return status;
        for (word = 0; word < lhs->l2_count; ++word) {
            uint8_t value = fill;
            if (ddc_c_bit_lsb(l2.data, word)) value = rhs->l1_lits.data[lit_off++];
            lhs->l1_lits.data[word] |= value;
        }
        ddc_c_bytes_destroy(&l2);
        return DDC_C_OK;
    }
#endif
}

ddc_c_status ddc_c_btv_binary_inplace(ddc_c_btv *lhs,
                                       const ddc_c_btv *rhs, ddc_c_op op) {
    ddc_c_btv tmp;
    ddc_c_status status;
    uint8_t saved_compress_results;
    if (lhs == NULL || rhs == NULL) return DDC_C_INVALID_ARGUMENT;
    if (op == DDC_C_OP_OR && lhs->state == DDC_C_DECOMPRESSED)
        return ddc_c_btv_or_inplace_fast(lhs, rhs);
    if (lhs->state != DDC_C_DECOMPRESSED || rhs->state == DDC_C_UNCOMPRESSED)
        return DDC_C_STATE_ERROR;
    /* C++ in-place kernels always retain the decompressed lhs form, without
       mutating the process-global result-form setting. */
    ddc_c_btv_init(&tmp, 0, 0, DDC_C_COMPRESSED);
    saved_compress_results = ddc_c_get_compress_results();
    ddc_c_set_compress_results(0);
    status = ddc_c_btv_binary(&tmp, lhs, rhs, op, 1, NULL);
    ddc_c_set_compress_results(saved_compress_results);
    if (status != DDC_C_OK) return status;
    ddc_c_btv_destroy(lhs);
    *lhs = tmp;
    return DDC_C_OK;
}

ddc_c_status ddc_c_btv_and(ddc_c_btv *out, const ddc_c_btv *a,
                            const ddc_c_btv *b) {
    return ddc_c_btv_binary(out, a, b, DDC_C_OP_AND, 1, NULL);
}

ddc_c_status ddc_c_btv_or(ddc_c_btv *out, const ddc_c_btv *a,
                           const ddc_c_btv *b) {
    return ddc_c_btv_binary(out, a, b, DDC_C_OP_OR, 1, NULL);
}

ddc_c_status ddc_c_btv_xor(ddc_c_btv *out, const ddc_c_btv *a,
                            const ddc_c_btv *b) {
    return ddc_c_btv_binary(out, a, b, DDC_C_OP_XOR, 1, NULL);
}

ddc_c_status ddc_c_btv_and_no_bypass(ddc_c_btv *out, const ddc_c_btv *a,
                                      const ddc_c_btv *b) {
    return ddc_c_btv_binary(out, a, b, DDC_C_OP_AND, 0, NULL);
}

ddc_c_status ddc_c_btv_and_inplace(ddc_c_btv *lhs, const ddc_c_btv *rhs) {
    return ddc_c_btv_binary_inplace(lhs, rhs, DDC_C_OP_AND);
}

ddc_c_status ddc_c_btv_or_inplace(ddc_c_btv *lhs, const ddc_c_btv *rhs) {
    return ddc_c_btv_binary_inplace(lhs, rhs, DDC_C_OP_OR);
}

ddc_c_status ddc_c_btv_xor_inplace(ddc_c_btv *lhs, const ddc_c_btv *rhs) {
    return ddc_c_btv_binary_inplace(lhs, rhs, DDC_C_OP_XOR);
}

ddc_c_status ddc_c_btv_not_inplace(ddc_c_btv *btv) {
    size_t i;
    if (btv == NULL) return DDC_C_INVALID_ARGUMENT;
    if (btv->state == DDC_C_UNCOMPRESSED) return DDC_C_STATE_ERROR;
    /* Mirror C++ negate_inplace(): no-op on empty BTV. */
    if (btv->bit_count == 0) return DDC_C_OK;
    /* Only L1 is a value fill. L2/L3 encode literal-vs-fill control and are
       unchanged by complementing logical data. */
    btv->l1_fill_ones = !btv->l1_fill_ones;
#ifdef __AVX512F__
    for (i = 0; i + 64 <= btv->l1_lits.len; i += 64) {
        __m512i v = _mm512_loadu_si512((const void *)(btv->l1_lits.data + i));
        _mm512_storeu_si512((void *)(btv->l1_lits.data + i),
                            _mm512_xor_si512(v, _mm512_set1_epi8((char)-1)));
    }
#else
    i = 0;
#endif
    for (; i < btv->l1_lits.len; ++i) btv->l1_lits.data[i] = (uint8_t)~btv->l1_lits.data[i];
    if (btv->bit_count % 8u != 0 && btv->l1_lit_count > 0 &&
        ddc_c_btv_is_last_word_literal(btv)) {
        /* L1 literals are stored in increasing word order. If the final word
           is literal, it is therefore the final L1 literal; no full L2
           materialization is needed merely to locate this byte. */
        const size_t literal = btv->l1_lit_count - 1u;
        if (literal < btv->l1_lits.len) {
            uint8_t mask = (uint8_t)(UINT8_MAX << (8u - (btv->bit_count % 8u)));
            btv->l1_lits.data[literal] &= mask;
        }
    }
    return DDC_C_OK;
}

ddc_c_status ddc_c_btv_not(ddc_c_btv *out, const ddc_c_btv *in) {
    ddc_c_status status;
    if (out == NULL || in == NULL || out == in) return DDC_C_INVALID_ARGUMENT;
    /* Mirror C++ operator~(): empty BTV yields an empty BTV. */
    if (in->bit_count == 0) {
        ddc_c_btv_destroy(out);
        ddc_c_btv_init(out, 0, 0, DDC_C_COMPRESSED);
        return DDC_C_OK;
    }
    /* Match DDCBtv::operator~(): a pure fill result is canonicalized rather
       than retaining the input's empty control hierarchy. In-place NOT keeps
       its existing representation, just as C++ negate_inplace() does. */
    if (in->l1_lit_count == 0)
        return ddc_c_make_all_fill(out, in->bit_count, in->l2_count,
                                   (uint8_t)!in->l1_fill_ones);

    /* Copy in -> out (reusing out's existing buffer capacity to avoid
     * malloc), then invert in-place.  This matches C++ operator~() which
     * copies the control hierarchy and XORs L1 literals in a single pass. */
    status = ddc_c_btv_copy(out, in);
    if (status != DDC_C_OK) return status;
    return ddc_c_btv_not_inplace(out);
}

size_t ddc_c_btv_popcount_and(const ddc_c_btv *a, const ddc_c_btv *b) {
    ddc_c_bytes l2a, l2b;
    size_t i, a_off = 0, b_off = 0, total = 0;
    if (a == NULL || b == NULL || a->bit_count != b->bit_count ||
        a->state == DDC_C_UNCOMPRESSED || b->state == DDC_C_UNCOMPRESSED) return 0;
    ddc_c_bytes_init(&l2a);
    ddc_c_bytes_init(&l2b);
    if (ddc_c_btv_expand_l2(a, &l2a) != DDC_C_OK ||
        ddc_c_btv_expand_l2(b, &l2b) != DDC_C_OK) {
        ddc_c_bytes_destroy(&l2a);
        ddc_c_bytes_destroy(&l2b);
        return 0;
    }
    for (i = 0; i < a->l2_count; ++i) {
        uint8_t av = ddc_c_byte_fill(a->l1_fill_ones);
        uint8_t bv = ddc_c_byte_fill(b->l1_fill_ones);
        if (ddc_c_bit_lsb(l2a.data, i)) av = a->l1_lits.data[a_off++];
        if (ddc_c_bit_lsb(l2b.data, i)) bv = b->l1_lits.data[b_off++];
        total += ddc_c_popcount8((uint8_t)(av & bv));
    }
    if (a->bit_count % 8u != 0) {
        const size_t valid = a->bit_count % 8u;
        const size_t extra = 8u - valid;
        uint8_t tail_a = ddc_c_byte_fill(a->l1_fill_ones);
        uint8_t tail_b = ddc_c_byte_fill(b->l1_fill_ones);
        /* The loop already included padded tail bits. Reconstruct only the
           final byte to remove their contribution exactly. */
        a_off = 0;
        b_off = 0;
        for (i = 0; i < a->l2_count; ++i) {
            if (ddc_c_bit_lsb(l2a.data, i)) tail_a = a->l1_lits.data[a_off++];
            else tail_a = ddc_c_byte_fill(a->l1_fill_ones);
            if (ddc_c_bit_lsb(l2b.data, i)) tail_b = b->l1_lits.data[b_off++];
            else tail_b = ddc_c_byte_fill(b->l1_fill_ones);
        }
        total -= ddc_c_popcount8((uint8_t)(tail_a & tail_b & ((1u << extra) - 1u)));
    }
    ddc_c_bytes_destroy(&l2a);
    ddc_c_bytes_destroy(&l2b);
    return total;
}

ddc_c_status ddc_c_binary(ddc_c *out, const ddc_c *a, const ddc_c *b,
                           ddc_c_op op, uint8_t bypass) {
    ddc_c tmp;
    size_t i, arena_off = 0;
    uint8_t compress;
    ddc_c_status status;
    if (out == NULL || a == NULL || b == NULL || out == a || out == b) return DDC_C_INVALID_ARGUMENT;
    if (a->bit_count != b->bit_count || a->segment_count != b->segment_count ||
        a->segment_bits != b->segment_bits) return DDC_C_INVALID_ARGUMENT;
    ddc_c_init(&tmp);
    tmp.bit_count = a->bit_count;
    tmp.segment_bits = a->segment_bits;
    status = ddc_c_reserve_segments(&tmp, a->segment_count);
    if (status != DDC_C_OK) return status;
    /* Decompressed results share one contiguous L1 arena across segments so
       the per-segment generic BTV kernel below does not leave one malloc'd
       block per segment (100M rows/w8 has 1526) scattered across the heap;
       that fragmentation is what caused post-warmup direct/multi-way OR
       slowdowns (see Phase 03 Remaining Work #1). Compressed results keep
       individual allocation: compaction shrinks each L1 buffer afterward to
       a data-dependent size that is not known before the segment runs. */
    compress = ddc_c_get_compress_results();
    if (!compress) {
        size_t total = 0;
        for (i = 0; i < a->segment_count; ++i) total += a->segments[i].l2_count;
        status = ddc_c_bytes_resize(&tmp.l1_arena, total);
        if (status != DDC_C_OK) { ddc_c_destroy(&tmp); return status; }
    }
    for (i = 0; i < a->segment_count; ++i) {
        const ddc_c_btv *sa = &a->segments[i];
        const ddc_c_btv *sb = &b->segments[i];
        ddc_c_btv *result = &tmp.segments[i];
        ddc_c_bytes arena_slice;
        ddc_c_bytes *slice = NULL;
        /* Direct fill/copy helpers initialize the empty reserved slot;
           generic BTV kernels retain their transactional replacement path. */
        ++tmp.segment_count;
        if (!compress) {
            /* Reserved regardless of which branch below actually runs: the
               offset must advance by sa->l2_count every iteration so later
               segments' slices stay non-overlapping, even though only the
               generic-kernel branch consumes its slice. */
            ddc_c_bytes_borrow(&arena_slice, tmp.l1_arena.data + arena_off, sa->l2_count);
            slice = &arena_slice;
        }
        if (bypass && op == DDC_C_OP_AND &&
            (ddc_c_btv_is_all_zero(sa) || ddc_c_btv_is_all_zero(sb))) {
            ddc_c_btv_init_all_fill(result, sa->bit_count, sa->l2_count, 0);
            status = DDC_C_OK;
        } else if (bypass && op == DDC_C_OP_AND && ddc_c_btv_is_all_ones(sa)) {
            status = ddc_c_btv_copy_fresh(result, sb);
        } else if (bypass && op == DDC_C_OP_AND && ddc_c_btv_is_all_ones(sb)) {
            status = ddc_c_btv_copy_fresh(result, sa);
        } else if (bypass && op == DDC_C_OP_OR && ddc_c_btv_is_all_zero(sa)) {
            status = ddc_c_btv_copy_fresh(result, sb);
        } else if (bypass && op == DDC_C_OP_OR && ddc_c_btv_is_all_zero(sb)) {
            status = ddc_c_btv_copy_fresh(result, sa);
        } else if (bypass && op == DDC_C_OP_OR && ddc_c_btv_is_all_ones(sa)) {
            status = ddc_c_btv_copy_fresh(result, sa);
        } else if (bypass && op == DDC_C_OP_OR && ddc_c_btv_is_all_ones(sb)) {
            status = ddc_c_btv_copy_fresh(result, sb);
        } else if (bypass && op == DDC_C_OP_XOR && ddc_c_btv_is_all_zero(sa)) {
            status = ddc_c_btv_copy_fresh(result, sb);
        } else if (bypass && op == DDC_C_OP_XOR && ddc_c_btv_is_all_zero(sb)) {
            status = ddc_c_btv_copy_fresh(result, sa);
        } else if (bypass && op == DDC_C_OP_XOR && ddc_c_btv_is_all_ones(sa)) {
            status = ddc_c_btv_not(result, sb);
        } else if (bypass && op == DDC_C_OP_XOR && ddc_c_btv_is_all_ones(sb)) {
            status = ddc_c_btv_not(result, sa);
        } else {
            /* Generic BTV kernels retain transactional replacement semantics;
               their destination must therefore stay in the initialized-slot
               state supplied by segment reservation. */
            status = ddc_c_btv_binary(result, sa, sb, op, bypass, slice);
        }
        if (!compress) arena_off += sa->l2_count;
        if (status != DDC_C_OK) { ddc_c_destroy(&tmp); return status; }
    }
    ddc_c_destroy(out);
    *out = tmp;
    return DDC_C_OK;
}

ddc_c_status ddc_c_binary_inplace(ddc_c *lhs, const ddc_c *rhs,
                                   ddc_c_op op) {
    size_t i;
    ddc_c_status status;
    if (lhs == NULL || rhs == NULL || lhs->bit_count != rhs->bit_count ||
        lhs->segment_count != rhs->segment_count ||
        lhs->segment_bits != rhs->segment_bits) return DDC_C_INVALID_ARGUMENT;
    for (i = 0; i < lhs->segment_count; ++i) {
        const ddc_c_btv *left  = &lhs->segments[i];
        const ddc_c_btv *right = &rhs->segments[i];
        /* Mirror C++ operator&=, operator|=, and operator^= bypass logic. */
        if (op == DDC_C_OP_AND) {
            if (ddc_c_btv_is_all_ones(right)) continue;
            if (ddc_c_btv_is_all_zero(right)) {
                status = ddc_c_make_all_fill(&lhs->segments[i], left->bit_count,
                                             left->l2_count, 0);
                if (status != DDC_C_OK) return status;
                continue;
            }
            if (ddc_c_btv_is_all_zero(left)) continue;
            if (ddc_c_btv_is_all_ones(left)) {
                status = ddc_c_btv_copy(&lhs->segments[i], right);
                if (status != DDC_C_OK) return status;
                continue;
            }
        } else if (op == DDC_C_OP_OR) {
            if (ddc_c_btv_is_all_zero(right)) continue;
            if (ddc_c_btv_is_all_ones(right)) {
                status = ddc_c_make_all_fill(&lhs->segments[i], left->bit_count,
                                             left->l2_count, 1);
                if (status != DDC_C_OK) return status;
                continue;
            }
            if (ddc_c_btv_is_all_ones(left)) continue;
            if (ddc_c_btv_is_all_zero(left)) {
                status = ddc_c_btv_copy(&lhs->segments[i], right);
                if (status != DDC_C_OK) return status;
                continue;
            }
        } else { /* DDC_C_OP_XOR */
            /* C++ operator^=: rhs==0 → skip; rhs==all-ones → NOT(lhs);
               lhs==0 → copy rhs; lhs==all-ones → NOT(rhs). */
            if (ddc_c_btv_is_all_zero(right)) continue;
            if (ddc_c_btv_is_all_ones(right)) {
                /* XOR with all-ones == NOT in place. */
                status = ddc_c_btv_not_inplace(&lhs->segments[i]);
                if (status != DDC_C_OK) return status;
                continue;
            }
            if (ddc_c_btv_is_all_zero(left)) {
                status = ddc_c_btv_copy(&lhs->segments[i], right);
                if (status != DDC_C_OK) return status;
                continue;
            }
            if (ddc_c_btv_is_all_ones(left)) {
                /* lhs = ~rhs */
                status = ddc_c_btv_not(&lhs->segments[i], right);
                if (status != DDC_C_OK) return status;
                continue;
            }
        }
        if (left->state != DDC_C_DECOMPRESSED) {
            ddc_c_btv tmp;
            ddc_c_btv_init(&tmp, 0, 0, DDC_C_COMPRESSED);
            status = ddc_c_btv_binary(&tmp, left, right, op, 1, NULL);
            if (status != DDC_C_OK) return status;
            ddc_c_btv_destroy(&lhs->segments[i]);
            lhs->segments[i] = tmp;
        } else {
            status = ddc_c_btv_binary_inplace(&lhs->segments[i], right, op);
            if (status != DDC_C_OK) return status;
        }
    }
    return DDC_C_OK;
}

ddc_c_status ddc_c_and(ddc_c *out, const ddc_c *a, const ddc_c *b) { return ddc_c_binary(out, a, b, DDC_C_OP_AND, 1); }
ddc_c_status ddc_c_or(ddc_c *out, const ddc_c *a, const ddc_c *b) { return ddc_c_binary(out, a, b, DDC_C_OP_OR, 1); }
ddc_c_status ddc_c_xor(ddc_c *out, const ddc_c *a, const ddc_c *b) { return ddc_c_binary(out, a, b, DDC_C_OP_XOR, 1); }
ddc_c_status ddc_c_and_no_bypass(ddc_c *out, const ddc_c *a, const ddc_c *b) { return ddc_c_binary(out, a, b, DDC_C_OP_AND, 0); }
ddc_c_status ddc_c_and_inplace(ddc_c *lhs, const ddc_c *rhs) { return ddc_c_binary_inplace(lhs, rhs, DDC_C_OP_AND); }
ddc_c_status ddc_c_or_inplace(ddc_c *lhs, const ddc_c *rhs) { return ddc_c_binary_inplace(lhs, rhs, DDC_C_OP_OR); }
ddc_c_status ddc_c_xor_inplace(ddc_c *lhs, const ddc_c *rhs) { return ddc_c_binary_inplace(lhs, rhs, DDC_C_OP_XOR); }

ddc_c_status ddc_c_not(ddc_c *out, const ddc_c *in) {
    ddc_c_status status;
    size_t i;

    if (out == NULL || in == NULL || out == in) return DDC_C_INVALID_ARGUMENT;

    /* Fast path: if out already has the same shape, overwrite segments in-place
     * reusing existing buffer capacity (avoids malloc for each btv layer). */
    if (out->segment_count == in->segment_count &&
        out->bit_count == in->bit_count &&
        out->segment_bits == in->segment_bits) {
        for (i = 0; i < in->segment_count; ++i) {
            status = ddc_c_btv_overwrite(&out->segments[i], &in->segments[i]);
            if (status != DDC_C_OK) return status;
            status = ddc_c_btv_not_inplace(&out->segments[i]);
            if (status != DDC_C_OK) return status;
        }
        return DDC_C_OK;
    }

    /* General path: build result in tmp, then swap. */
    {
        ddc_c tmp;
        ddc_c_init(&tmp);
        tmp.bit_count = in->bit_count;
        tmp.segment_bits = in->segment_bits;
        status = ddc_c_reserve_segments(&tmp, in->segment_count);
        if (status != DDC_C_OK) return status;
        for (i = 0; i < in->segment_count; ++i) {
            const ddc_c_btv *source = &in->segments[i];
            ++tmp.segment_count;
            if (source->l1_lit_count == 0) {
                ddc_c_btv_init_all_fill(&tmp.segments[i], source->bit_count,
                                        source->l2_count,
                                        (uint8_t)!source->l1_fill_ones);
            } else {
                status = ddc_c_btv_not(&tmp.segments[i], source);
                if (status != DDC_C_OK) { ddc_c_destroy(&tmp); return status; }
            }
        }
        ddc_c_destroy(out);
        *out = tmp;
    }
    return DDC_C_OK;
}

ddc_c_status ddc_c_not_inplace(ddc_c *ddc) {
    size_t i;
    ddc_c_status status;
    if (ddc == NULL) return DDC_C_INVALID_ARGUMENT;
    for (i = 0; i < ddc->segment_count; ++i) {
        status = ddc_c_btv_not_inplace(&ddc->segments[i]);
        if (status != DDC_C_OK) return status;
    }
    return DDC_C_OK;
}

size_t ddc_c_popcount_and(const ddc_c *a, const ddc_c *b) {
    size_t i, total = 0;
    if (a == NULL || b == NULL || a->bit_count != b->bit_count ||
        a->segment_count != b->segment_count ||
        a->segment_bits != b->segment_bits) return 0;
    for (i = 0; i < a->segment_count; ++i) {
        if (ddc_c_btv_is_all_zero(&b->segments[i])) continue;
        if (ddc_c_btv_is_all_ones(&b->segments[i])) total += ddc_c_btv_popcount(&a->segments[i]);
        else total += ddc_c_btv_popcount_and(&a->segments[i], &b->segments[i]);
    }
    return total;
}
