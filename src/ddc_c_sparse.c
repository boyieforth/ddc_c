#include "ddc_c_internal.h"

static int ddc_c_compare_u32(const void *a, const void *b) {
    const uint32_t lhs = *(const uint32_t *)a;
    const uint32_t rhs = *(const uint32_t *)b;
    return lhs > rhs ? 1 : lhs < rhs ? -1 : 0;
}

static ddc_c_status ddc_c_sparse_reserve(ddc_c_sparse *sparse, size_t count) {
    uint32_t *indices;
    ddc_c_btv *segments;
    size_t i, new_cap = count;
    if (count <= sparse->segment_cap) return DDC_C_OK;
    if (sparse->segment_cap != 0 && sparse->segment_cap <= SIZE_MAX / 2u &&
        new_cap < sparse->segment_cap * 2u)
        new_cap = sparse->segment_cap * 2u;
    if (new_cap > SIZE_MAX / sizeof(*indices) ||
        new_cap > SIZE_MAX / sizeof(*segments))
        return DDC_C_ALLOCATION_FAILED;
    indices = (uint32_t *)malloc(new_cap * sizeof(*indices));
    segments = (ddc_c_btv *)malloc(new_cap * sizeof(*segments));
    if (indices == NULL || segments == NULL) {
        free(indices);
        free(segments);
        return DDC_C_ALLOCATION_FAILED;
    }
    for (i = 0; i < sparse->segment_count; ++i) {
        indices[i] = sparse->seg_indices[i];
        segments[i] = sparse->seg_data[i];
    }
    for (; i < new_cap; ++i) ddc_c_btv_init(&segments[i], 0, 0, DDC_C_COMPRESSED);
    free(sparse->seg_indices);
    free(sparse->seg_data);
    sparse->seg_indices = indices;
    sparse->seg_data = segments;
    sparse->segment_cap = new_cap;
    return DDC_C_OK;
}

ddc_c_status ddc_c_sparse_from_positions(ddc_c_sparse *out,
                                          const uint32_t *positions,
                                          size_t position_count,
                                          size_t num_rows,
                                          size_t segment_bits) {
    uint32_t *sorted = NULL;
    ddc_c_sparse tmp;
    size_t i, start;
    ddc_c_status status;
    if (out == NULL || (position_count != 0 && positions == NULL)) return DDC_C_INVALID_ARGUMENT;
    if (segment_bits == 0) segment_bits = DDC_C_DEFAULT_SEGMENT_BITS;
    if (segment_bits > UINT16_MAX + 1u) return DDC_C_INVALID_ARGUMENT;
    if (position_count != 0) {
        sorted = (uint32_t *)malloc(position_count * sizeof(*sorted));
        if (sorted == NULL) return DDC_C_ALLOCATION_FAILED;
        for (i = 0; i < position_count; ++i) {
            if ((size_t)positions[i] >= num_rows) { free(sorted); return DDC_C_INVALID_ARGUMENT; }
            sorted[i] = positions[i];
        }
        qsort(sorted, position_count, sizeof(*sorted), ddc_c_compare_u32);
    }
    ddc_c_sparse_init(&tmp);
    tmp.bit_count = num_rows;
    tmp.segment_bits = segment_bits;
    tmp.num_set_bits = position_count;
    start = 0;
    while (start < position_count) {
        const uint32_t seg_index = sorted[start] / (uint32_t)segment_bits;
        size_t end = start;
        size_t seg_offset = (size_t)seg_index * segment_bits;
        size_t seg_len = segment_bits < num_rows - seg_offset ? segment_bits : num_rows - seg_offset;
        uint16_t *local;
        ddc_c_btv seg;
        while (end < position_count && sorted[end] / (uint32_t)segment_bits == seg_index) ++end;
        local = (uint16_t *)malloc((end - start) * sizeof(*local));
        if (local == NULL) { status = DDC_C_ALLOCATION_FAILED; goto fail; }
        for (i = start; i < end; ++i) local[i - start] = (uint16_t)(sorted[i] % segment_bits);
        status = ddc_c_sparse_reserve(&tmp, tmp.segment_count + 1);
        if (status != DDC_C_OK) { free(local); goto fail; }
        ddc_c_btv_init(&seg, 0, 0, DDC_C_COMPRESSED);
        status = ddc_c_btv_compress_sparse_segment(&seg, local, end - start, seg_len, 0);
        free(local);
        if (status != DDC_C_OK) { ddc_c_btv_destroy(&seg); goto fail; }
        tmp.seg_indices[tmp.segment_count] = seg_index;
        ddc_c_btv_destroy(&tmp.seg_data[tmp.segment_count]);
        tmp.seg_data[tmp.segment_count] = seg;
        ++tmp.segment_count;
        start = end;
    }
    free(sorted);
    ddc_c_sparse_destroy(out);
    *out = tmp;
    return DDC_C_OK;
fail:
    free(sorted);
    ddc_c_sparse_destroy(&tmp);
    return status;
}

ddc_c_status ddc_c_sparse_apply_or_to(const ddc_c_sparse *sparse, ddc_c *dst) {
    size_t i;
    ddc_c_status status;
    if (sparse == NULL || dst == NULL || sparse->bit_count != dst->bit_count ||
        sparse->segment_bits != dst->segment_bits) return DDC_C_INVALID_ARGUMENT;
    for (i = 0; i < sparse->segment_count; ++i) {
        const uint32_t index = sparse->seg_indices[i];
        ddc_c_btv *target;
        const ddc_c_btv *source;
        if (index >= dst->segment_count) return DDC_C_MALFORMED_INPUT;
        target = &dst->segments[index];
        source = &sparse->seg_data[i];
        if (ddc_c_btv_is_all_zero(source) || ddc_c_btv_is_all_ones(target)) continue;
        if (ddc_c_btv_is_all_zero(target)) {
            if (target->state == DDC_C_DECOMPRESSED)
                status = ddc_c_btv_or_inplace(target, source);
            else {
                ddc_c_btv replacement;
                ddc_c_btv_init(&replacement, 0, 0, DDC_C_COMPRESSED);
                status = ddc_c_btv_copy(&replacement, source);
                if (status == DDC_C_OK) {
                    ddc_c_btv_destroy(target);
                    *target = replacement;
                }
            }
        } else if (ddc_c_btv_is_all_ones(source)) {
            status = ddc_c_make_all_fill(target, source->bit_count, source->l2_count, 1);
        } else if (target->state == DDC_C_DECOMPRESSED) {
            status = ddc_c_btv_or_inplace(target, source);
        } else {
            ddc_c_btv replacement;
            ddc_c_btv_init(&replacement, 0, 0, DDC_C_COMPRESSED);
            status = ddc_c_btv_or(&replacement, target, source);
            if (status == DDC_C_OK) {
                ddc_c_btv_destroy(target);
                *target = replacement;
            }
        }
        if (status != DDC_C_OK) return status;
    }
    return DDC_C_OK;
}

size_t ddc_c_sparse_storage_bytes(const ddc_c_sparse *sparse) {
    size_t i, total;
    if (sparse == NULL) return 0;
    total = sizeof(*sparse) + sparse->segment_cap * sizeof(uint32_t) +
            sparse->segment_cap * sizeof(ddc_c_btv);
    for (i = 0; i < sparse->segment_count; ++i) {
        ddc_c_size_info size;
        ddc_c_btv_size_breakdown(&sparse->seg_data[i], &size);
        total += size.total_bits / 8u;
    }
    return total;
}

ddc_c_status ddc_c_sparse_or_many(ddc_c *out, size_t count,
                                   const ddc_c_sparse *const *sparses,
                                   size_t num_rows, size_t segment_bits) {
    ddc_c_status status;
    size_t i;
    if (out == NULL || (count != 0 && sparses == NULL)) return DDC_C_INVALID_ARGUMENT;
    status = ddc_c_from_sparse_positions(out, NULL, 0, num_rows, segment_bits);
    if (status != DDC_C_OK) return status;
    for (i = 0; i < count; ++i) {
        if (sparses[i] == NULL) return DDC_C_INVALID_ARGUMENT;
        status = ddc_c_sparse_apply_or_to(sparses[i], out);
        if (status != DDC_C_OK) return status;
    }
    return DDC_C_OK;
}

typedef struct ddc_c_scatter_ctx {
    ddc_c *out;
    size_t current_segment;
    size_t segment_start;
    size_t segment_end;
} ddc_c_scatter_ctx;

static void ddc_c_scatter_literal(void *opaque, uint32_t word_pos, uint8_t value) {
    ddc_c_scatter_ctx *ctx = (ddc_c_scatter_ctx *)opaque;
    while ((size_t)word_pos >= ctx->segment_end && ctx->current_segment + 1 < ctx->out->segment_count) {
        ++ctx->current_segment;
        ctx->segment_start = ctx->segment_end;
        ctx->segment_end += ctx->out->segments[ctx->current_segment].l2_count;
    }
    ctx->out->segments[ctx->current_segment].l1_lits.data[word_pos - ctx->segment_start] |= value;
}

ddc_c_status ddc_c_or_many(ddc_c *out, size_t count, const ddc_c *const *ddcs) {
    size_t i, s, total_nonzero = 0, total_slots = 0;
    uint8_t use_scatter;
    ddc_c tmp;
    ddc_c_status status;
    if (out == NULL || ddcs == NULL || count == 0 || ddcs[0] == NULL) return DDC_C_INVALID_ARGUMENT;
    for (i = 0; i < count; ++i) {
        if (ddcs[i] == NULL || ddcs[i]->bit_count != ddcs[0]->bit_count ||
            ddcs[i]->segment_count != ddcs[0]->segment_count ||
            ddcs[i]->segment_bits != ddcs[0]->segment_bits) return DDC_C_INVALID_ARGUMENT;
        for (s = 0; s < ddcs[i]->segment_count; ++s) {
            total_nonzero += ddcs[i]->segments[s].l1_lit_count;
            total_slots += ddcs[i]->segments[s].l2_count;
        }
    }
    if (count == 1) {
        ddc_c_init(&tmp);
        tmp.bit_count = ddcs[0]->bit_count;
        tmp.segment_bits = ddcs[0]->segment_bits;
        status = ddc_c_reserve_segments(&tmp, ddcs[0]->segment_count);
        if (status != DDC_C_OK) return status;
        for (s = 0; s < ddcs[0]->segment_count; ++s) {
            status = ddc_c_push_segment_copy(&tmp, &ddcs[0]->segments[s]);
            if (status != DDC_C_OK) { ddc_c_destroy(&tmp); return status; }
        }
        ddc_c_destroy(out);
        *out = tmp;
        return DDC_C_OK;
    }
    use_scatter = total_nonzero <= SIZE_MAX / 20u && total_nonzero * 20u < total_slots;
    ddc_c_init(&tmp);
    tmp.bit_count = ddcs[0]->bit_count;
    tmp.segment_bits = ddcs[0]->segment_bits;
    status = ddc_c_reserve_segments(&tmp, ddcs[0]->segment_count);
    if (status != DDC_C_OK) return status;
    if (use_scatter) {
        /* Build a decompressed-zero accumulator per segment, then OR-inplace
         * each input using the VBMI2 fast path instead of byte-scatter.
         * This is faster than ddc_c_for_each_literal scatter for all but the
         * most trivially sparse inputs, because VBMI2 processes 64 regions
         * per iteration rather than visiting each literal byte individually. */
        for (s = 0; s < ddcs[0]->segment_count; ++s) {
            ddc_c_btv dense;
            ddc_c_btv_init(&dense, 0, 0, DDC_C_COMPRESSED);
            status = ddc_c_make_decompressed_zero(&dense, ddcs[0]->segments[s].bit_count,
                                                  ddcs[0]->segments[s].l2_count);
            if (status == DDC_C_OK) status = ddc_c_push_segment_move(&tmp, &dense);
            ddc_c_btv_destroy(&dense);
            if (status != DDC_C_OK) { ddc_c_destroy(&tmp); return status; }
        }
        for (i = 0; i < count; ++i) {
            for (s = 0; s < ddcs[0]->segment_count; ++s) {
                status = ddc_c_btv_or_inplace(&tmp.segments[s], &ddcs[i]->segments[s]);
                if (status != DDC_C_OK) { ddc_c_destroy(&tmp); return status; }
            }
        }
    } else {
        /* Pairwise path: produce a decompressed accumulator on the first OR so
         * that all subsequent rounds use ddc_c_btv_or_inplace fast path
         * (dense SIMD OR into the existing decompressed buffer) instead of
         * allocating a new compressed result each time. */
        uint8_t saved_compress = ddc_c_get_compress_results();
        ddc_c_set_compress_results(0);
        for (s = 0; s < ddcs[0]->segment_count; ++s) {
            ddc_c_btv accumulator;
            ddc_c_btv_init(&accumulator, 0, 0, DDC_C_COMPRESSED);
            /* First OR: compress_results==0 forces decompressed output. */
            status = ddc_c_btv_or(&accumulator, &ddcs[0]->segments[s], &ddcs[1]->segments[s]);
            /* Subsequent rounds: inplace OR directly into decompressed lhs. */
            for (i = 2; status == DDC_C_OK && i < count; ++i)
                status = ddc_c_btv_or_inplace(&accumulator, &ddcs[i]->segments[s]);
            if (status == DDC_C_OK) status = ddc_c_push_segment_move(&tmp, &accumulator);
            ddc_c_btv_destroy(&accumulator);
            if (status != DDC_C_OK) {
                ddc_c_set_compress_results(saved_compress);
                ddc_c_destroy(&tmp);
                return status;
            }
        }
        ddc_c_set_compress_results(saved_compress);
    }
    ddc_c_destroy(out);
    *out = tmp;
    return DDC_C_OK;
}
