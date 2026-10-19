#include "ddc_c_internal.h"

#define DDC_C_FMT_V1 8u
#define DDC_C_FMT_V2 9u
#define DDC_C_FMT_V3_MAGIC 0xCBu
#define DDC_C_FMT_V4_MAGIC 0xC4u

typedef struct ddc_c_writer {
    uint8_t *data;
    size_t capacity;
    size_t pos;
    uint8_t overflow;
} ddc_c_writer;

typedef struct ddc_c_reader {
    const uint8_t *data;
    size_t length;
    size_t pos;
} ddc_c_reader;

static uint8_t ddc_c_u64_to_size(uint64_t value, size_t *out) {
    if (value > (uint64_t)SIZE_MAX) return 0;
    *out = (size_t)value;
    return 1;
}

static uint8_t ddc_c_bits_to_bytes(size_t bits, size_t *out) {
    *out = bits / 8u + (bits % 8u != 0u);
    return 1;
}

static uint8_t ddc_c_expected_level_counts(size_t bit_count, size_t *l2_count,
                                           size_t *l3_count, size_t *l4_count) {
    ddc_c_bits_to_bytes(bit_count, l2_count);
    ddc_c_bits_to_bytes(*l2_count, l3_count);
    ddc_c_bits_to_bytes(*l3_count, l4_count);
    return 1;
}

static uint8_t ddc_c_expected_segment_count(size_t bit_count, size_t segment_bits,
                                            size_t *out) {
    if (segment_bits == 0) return 0;
    *out = bit_count / segment_bits + (bit_count % segment_bits != 0u);
    return 1;
}

static uint8_t ddc_c_expected_segment_bits(size_t bit_count, size_t segment_bits,
                                           size_t index, size_t count, size_t *out) {
    size_t remainder;
    if (index >= count || segment_bits == 0) return 0;
    if (index + 1u < count) {
        *out = segment_bits;
    } else {
        remainder = bit_count % segment_bits;
        *out = remainder == 0 ? segment_bits : remainder;
    }
    return *out != 0;
}

static ddc_c_status ddc_c_validate_container_header(size_t bit_count,
                                                     size_t segment_bits,
                                                     size_t segment_count) {
    size_t expected_segment_count;
    if (!ddc_c_expected_segment_count(bit_count, segment_bits,
                                      &expected_segment_count) ||
        segment_count != expected_segment_count)
        return DDC_C_MALFORMED_INPUT;
    return DDC_C_OK;
}

static uint8_t ddc_c_unused_bits_are_zero(const uint8_t *data, size_t bit_count) {
    size_t byte_count = bit_count / 8u;
    uint8_t mask;
    if (bit_count % 8u == 0u) return 1;
    mask = (uint8_t)((1u << (bit_count % 8u)) - 1u);
    return (data[byte_count] & (uint8_t)~mask) == 0;
}

static size_t ddc_c_popcount_bits(const uint8_t *data, size_t bit_count) {
    size_t byte_count = bit_count / 8u;
    size_t count = 0;
    size_t i;
    for (i = 0; i < byte_count; ++i) count += ddc_c_popcount8(data[i]);
    if (bit_count % 8u != 0u)
        count += ddc_c_popcount8((uint8_t)(data[byte_count] &
                                            ((1u << (bit_count % 8u)) - 1u)));
    return count;
}

static ddc_c_status ddc_c_validate_l2_and_l1(const ddc_c_btv *btv,
                                               const uint8_t *l3_bits,
                                               size_t l3_byte_count) {
    size_t l2_lit_index = 0;
    size_t expected_l1_lits = 0;
    size_t l3_byte_index;
    for (l3_byte_index = 0; l3_byte_index < l3_byte_count; ++l3_byte_index) {
        uint8_t l3_byte = l3_bits[l3_byte_index];
        size_t bit;
        for (bit = 0; bit < 8u; ++bit) {
            size_t l3_index = l3_byte_index * 8u + bit;
            uint8_t l2_byte;
            size_t valid_l2_bits;
            if (l3_index >= btv->l3_count) break;
            if ((l3_byte >> bit) & 1u) {
                if (l2_lit_index >= btv->l2_lits.len) return DDC_C_MALFORMED_INPUT;
                l2_byte = btv->l2_lits.data[l2_lit_index++];
            } else {
                l2_byte = ddc_c_byte_fill(btv->l2_fill_ones);
            }
            valid_l2_bits = btv->l2_count - l3_index * 8u;
            if (valid_l2_bits > 8u) valid_l2_bits = 8u;
            expected_l1_lits += ddc_c_popcount_bits(&l2_byte, valid_l2_bits);
        }
    }
    if (l2_lit_index != btv->l2_lit_count ||
        expected_l1_lits != btv->l1_lit_count)
        return DDC_C_MALFORMED_INPUT;
    return DDC_C_OK;
}

static ddc_c_status ddc_c_validate_compressed_btv(const ddc_c_btv *btv) {
    size_t expected_l2_count, expected_l3_count, expected_l4_count, expected_l4_bytes;
    size_t l3_lit_index = 0;
    size_t l3_byte_index;
    uint8_t l3_fill;
    ddc_c_status status;

    ddc_c_expected_level_counts(btv->bit_count, &expected_l2_count,
                                &expected_l3_count, &expected_l4_count);
    ddc_c_bits_to_bytes(expected_l4_count, &expected_l4_bytes);
    if (btv->state != DDC_C_COMPRESSED || btv->l2_count != expected_l2_count ||
        btv->l3_count != expected_l3_count || btv->l4_count != expected_l4_count ||
        btv->l4_bits.len != expected_l4_bytes ||
        !ddc_c_unused_bits_are_zero(btv->l4_bits.data, btv->l4_count) ||
        btv->l3_lits.len != btv->l3_lit_count ||
        btv->l2_lits.len != btv->l2_lit_count ||
        btv->l1_lits.len != btv->l1_lit_count)
        return DDC_C_MALFORMED_INPUT;

    l3_fill = ddc_c_byte_fill(btv->l3_fill_ones);
    for (l3_byte_index = 0; l3_byte_index < btv->l4_count; ++l3_byte_index) {
        if (ddc_c_bit_lsb(btv->l4_bits.data, l3_byte_index)) {
            if (l3_lit_index >= btv->l3_lits.len) return DDC_C_MALFORMED_INPUT;
            ++l3_lit_index;
        }
    }
    if (l3_lit_index != btv->l3_lit_count) return DDC_C_MALFORMED_INPUT;

    {
        uint8_t *l3_flat = NULL;
        if (btv->l4_count != 0) {
            l3_flat = (uint8_t *)malloc(btv->l4_count);
            if (l3_flat == NULL) return DDC_C_ALLOCATION_FAILED;
        }
        l3_lit_index = 0;
        for (l3_byte_index = 0; l3_byte_index < btv->l4_count; ++l3_byte_index) {
            if (ddc_c_bit_lsb(btv->l4_bits.data, l3_byte_index))
                l3_flat[l3_byte_index] = btv->l3_lits.data[l3_lit_index++];
            else
                l3_flat[l3_byte_index] = l3_fill;
        }
        status = ddc_c_validate_l2_and_l1(btv, l3_flat, btv->l4_count);
        free(l3_flat);
    }
    return status;
}

static ddc_c_status ddc_c_validate_v1_btv(const ddc_c_btv *btv) {
    size_t expected_l2_count, expected_l3_count, expected_l4_count;
    if (btv->state != DDC_C_COMPRESSED) return DDC_C_MALFORMED_INPUT;
    ddc_c_expected_level_counts(btv->bit_count, &expected_l2_count,
                                &expected_l3_count, &expected_l4_count);
    if (btv->l2_count != expected_l2_count || btv->l3_count != expected_l3_count ||
        btv->l3_bits.len != expected_l4_count ||
        !ddc_c_unused_bits_are_zero(btv->l3_bits.data, btv->l3_count) ||
        btv->l2_lits.len != btv->l2_lit_count ||
        btv->l1_lits.len != btv->l1_lit_count ||
        ddc_c_popcount_bits(btv->l3_bits.data, btv->l3_count) != btv->l2_lit_count)
        return DDC_C_MALFORMED_INPUT;
    return ddc_c_validate_l2_and_l1(btv, btv->l3_bits.data, btv->l3_bits.len);
}

static void ddc_c_write_raw(ddc_c_writer *writer, const void *value, size_t len) {
    if (writer->overflow || len > writer->capacity - writer->pos) {
        writer->overflow = 1;
        return;
    }
    if (len != 0) memcpy(writer->data + writer->pos, value, len);
    writer->pos += len;
}

static void ddc_c_write_u8(ddc_c_writer *writer, uint8_t value) { ddc_c_write_raw(writer, &value, sizeof(value)); }
static void ddc_c_write_u32(ddc_c_writer *writer, uint32_t value) { ddc_c_write_raw(writer, &value, sizeof(value)); }
static void ddc_c_write_u64(ddc_c_writer *writer, uint64_t value) { ddc_c_write_raw(writer, &value, sizeof(value)); }

static uint8_t ddc_c_read_u8(ddc_c_reader *reader, uint8_t *value) {
    if (sizeof(*value) > reader->length - reader->pos) return 0;
    memcpy(value, reader->data + reader->pos, sizeof(*value));
    reader->pos += sizeof(*value);
    return 1;
}
static uint8_t ddc_c_read_u32(ddc_c_reader *reader, uint32_t *value) {
    if (sizeof(*value) > reader->length - reader->pos) return 0;
    memcpy(value, reader->data + reader->pos, sizeof(*value));
    reader->pos += sizeof(*value);
    return 1;
}
static uint8_t ddc_c_read_u64(ddc_c_reader *reader, uint64_t *value) {
    if (sizeof(*value) > reader->length - reader->pos) return 0;
    memcpy(value, reader->data + reader->pos, sizeof(*value));
    reader->pos += sizeof(*value);
    return 1;
}
static uint8_t ddc_c_read_bytes(ddc_c_reader *reader, ddc_c_bytes *out, size_t len) {
    if (len > reader->length - reader->pos || ddc_c_bytes_resize(out, len) != DDC_C_OK) return 0;
    if (len != 0) memcpy(out->data, reader->data + reader->pos, len);
    reader->pos += len;
    return 1;
}

static uint8_t ddc_c_read_size(ddc_c_reader *reader, size_t *out) {
    uint64_t value;
    return ddc_c_read_u64(reader, &value) && ddc_c_u64_to_size(value, out);
}

static uint8_t ddc_c_u32_to_size(uint32_t value, size_t *out) {
    if ((uint64_t)value > (uint64_t)SIZE_MAX) return 0;
    *out = (size_t)value;
    return 1;
}

static ddc_c_status ddc_c_btv_write_normal(ddc_c_writer *writer,
                                            const ddc_c_btv *btv) {
    if (btv == NULL || btv->state != DDC_C_COMPRESSED) return DDC_C_STATE_ERROR;
    ddc_c_write_u8(writer, DDC_C_FMT_V2);
    ddc_c_write_u8(writer, btv->l1_fill_ones);
    ddc_c_write_u8(writer, btv->l2_fill_ones);
    ddc_c_write_u8(writer, btv->l3_fill_ones);
    ddc_c_write_u64(writer, (uint64_t)btv->bit_count);
    ddc_c_write_u64(writer, (uint64_t)btv->l2_count);
    ddc_c_write_u64(writer, (uint64_t)btv->l3_count);
    ddc_c_write_u64(writer, (uint64_t)btv->l4_count);
    ddc_c_write_u64(writer, (uint64_t)btv->l4_bits.len);
    ddc_c_write_raw(writer, btv->l4_bits.data, btv->l4_bits.len);
    ddc_c_write_u64(writer, (uint64_t)btv->l3_lit_count);
    ddc_c_write_raw(writer, btv->l3_lits.data, btv->l3_lit_count);
    ddc_c_write_u64(writer, (uint64_t)btv->l2_lit_count);
    ddc_c_write_raw(writer, btv->l2_lits.data, btv->l2_lit_count);
    ddc_c_write_u64(writer, (uint64_t)btv->l1_lit_count);
    ddc_c_write_raw(writer, btv->l1_lits.data, btv->l1_lit_count);
    return writer->overflow ? DDC_C_BUFFER_TOO_SMALL : DDC_C_OK;
}

static ddc_c_status ddc_c_btv_read_normal(ddc_c_reader *reader, ddc_c_btv *out,
                                           uint8_t trusted) {
    uint8_t fmt, l1_fill, l2_fill, l3_fill = 0;
    size_t payload_len;
    ddc_c_btv tmp;
    ddc_c_status status;
    if (!ddc_c_read_u8(reader, &fmt) || (fmt != DDC_C_FMT_V1 && fmt != DDC_C_FMT_V2) ||
        !ddc_c_read_u8(reader, &l1_fill) || !ddc_c_read_u8(reader, &l2_fill))
        return DDC_C_MALFORMED_INPUT;
    ddc_c_btv_init(&tmp, l1_fill != 0, l2_fill != 0, DDC_C_COMPRESSED);
    if (fmt == DDC_C_FMT_V2) {
        if (!ddc_c_read_u8(reader, &l3_fill) ||
            !ddc_c_read_size(reader, &tmp.bit_count) ||
            !ddc_c_read_size(reader, &tmp.l2_count) ||
            !ddc_c_read_size(reader, &tmp.l3_count) ||
            !ddc_c_read_size(reader, &tmp.l4_count) ||
            !ddc_c_read_size(reader, &payload_len) ||
            !ddc_c_read_bytes(reader, &tmp.l4_bits, payload_len) ||
            !ddc_c_read_size(reader, &tmp.l3_lit_count) ||
            !ddc_c_read_bytes(reader, &tmp.l3_lits, tmp.l3_lit_count))
            goto malformed;
        tmp.l3_fill_ones = l3_fill != 0;
    } else {
        if (!ddc_c_read_size(reader, &tmp.bit_count) ||
            !ddc_c_read_size(reader, &tmp.l2_count) ||
            !ddc_c_read_size(reader, &tmp.l3_count) ||
            !ddc_c_read_size(reader, &payload_len) ||
            !ddc_c_read_bytes(reader, &tmp.l3_bits, payload_len))
            goto malformed;
    }
    if (!ddc_c_read_size(reader, &tmp.l2_lit_count) ||
        !ddc_c_read_bytes(reader, &tmp.l2_lits, tmp.l2_lit_count) ||
        !ddc_c_read_size(reader, &tmp.l1_lit_count) ||
        !ddc_c_read_bytes(reader, &tmp.l1_lits, tmp.l1_lit_count))
        goto malformed;

    if (fmt == DDC_C_FMT_V1) {
        status = ddc_c_validate_v1_btv(&tmp);
        if (status != DDC_C_OK) goto fail;
        status = ddc_c_btv_compress_l3_to_l4(&tmp);
        if (status != DDC_C_OK) goto fail;
    } else if (!trusted) {
        status = ddc_c_validate_compressed_btv(&tmp);
        if (status != DDC_C_OK) goto fail;
    }
    ddc_c_btv_destroy(out);
    *out = tmp;
    return DDC_C_OK;
malformed:
    ddc_c_btv_destroy(&tmp);
    return DDC_C_MALFORMED_INPUT;
fail:
    ddc_c_btv_destroy(&tmp);
    return status;
}

ddc_c_status ddc_c_serialize(const ddc_c *ddc, uint8_t *out,
                              size_t out_capacity, size_t *written) {
    ddc_c_writer writer = { out, out_capacity, 0, 0 };
    size_t i;
    ddc_c_status status;
    if (ddc == NULL || (out == NULL && out_capacity != 0)) return DDC_C_INVALID_ARGUMENT;
    ddc_c_write_u64(&writer, (uint64_t)ddc->bit_count);
    ddc_c_write_u64(&writer, (uint64_t)ddc->segment_bits);
    ddc_c_write_u64(&writer, (uint64_t)ddc->segment_count);
    for (i = 0; i < ddc->segment_count; ++i) {
        status = ddc_c_btv_write_normal(&writer, &ddc->segments[i]);
        if (status != DDC_C_OK) { if (written != NULL) *written = writer.pos; return status; }
    }
    if (written != NULL) *written = writer.pos;
    return DDC_C_OK;
}

ddc_c_status ddc_c_deserialize(ddc_c *out, const uint8_t *data,
                                size_t data_len, size_t *consumed) {
    ddc_c_reader reader = { data, data_len, 0 };
    ddc_c tmp;
    size_t count, i, expected_bits;
    ddc_c_status status;
    if (out == NULL || (data == NULL && data_len != 0)) return DDC_C_INVALID_ARGUMENT;
    ddc_c_init(&tmp);
    if (!ddc_c_read_size(&reader, &tmp.bit_count) ||
        !ddc_c_read_size(&reader, &tmp.segment_bits) ||
        !ddc_c_read_size(&reader, &count))
        goto malformed;
    status = ddc_c_validate_container_header(tmp.bit_count, tmp.segment_bits, count);
    if (status != DDC_C_OK) goto fail;
    status = ddc_c_reserve_segments(&tmp, count);
    if (status != DDC_C_OK) goto fail;
    for (i = 0; i < count; ++i) {
        if (!ddc_c_expected_segment_bits(tmp.bit_count, tmp.segment_bits, i, count,
                                         &expected_bits))
            goto malformed;
        status = ddc_c_btv_read_normal(&reader, &tmp.segments[tmp.segment_count], 0);
        if (status != DDC_C_OK) goto fail;
        ++tmp.segment_count;
        if (tmp.segments[tmp.segment_count - 1u].bit_count != expected_bits) {
            status = DDC_C_MALFORMED_INPUT;
            goto fail;
        }
    }
    ddc_c_destroy(out);
    *out = tmp;
    if (consumed != NULL) *consumed = reader.pos;
    return DDC_C_OK;
malformed:
    status = DDC_C_MALFORMED_INPUT;
fail:
    ddc_c_destroy(&tmp);
    return status;
}

static ddc_c_status ddc_c_btv_write_packed(ddc_c_writer *writer,
                                            const ddc_c_btv *btv,
                                            uint8_t is_last) {
    uint8_t flags = 0;
    if (btv == NULL || btv->state != DDC_C_COMPRESSED) return DDC_C_STATE_ERROR;
    if (btv->l1_fill_ones) flags |= 0x01u;
    if (btv->l2_fill_ones) flags |= 0x02u;
    if (btv->l3_fill_ones) flags |= 0x04u;
    if (is_last) flags |= 0x08u;
    ddc_c_write_u8(writer, flags);
    if (is_last) ddc_c_write_u32(writer, (uint32_t)btv->bit_count);
    ddc_c_write_u32(writer, (uint32_t)btv->l4_bits.len);
    ddc_c_write_u32(writer, (uint32_t)btv->l3_lit_count);
    ddc_c_write_u32(writer, (uint32_t)btv->l2_lit_count);
    ddc_c_write_u32(writer, (uint32_t)btv->l1_lit_count);
    ddc_c_write_raw(writer, btv->l4_bits.data, btv->l4_bits.len);
    ddc_c_write_raw(writer, btv->l3_lits.data, btv->l3_lit_count);
    ddc_c_write_raw(writer, btv->l2_lits.data, btv->l2_lit_count);
    ddc_c_write_raw(writer, btv->l1_lits.data, btv->l1_lit_count);
    return writer->overflow ? DDC_C_BUFFER_TOO_SMALL : DDC_C_OK;
}

static ddc_c_status ddc_c_btv_read_packed(ddc_c_reader *reader, ddc_c_btv *out,
                                           size_t bit_count, uint8_t is_last,
                                           uint8_t trusted) {
    uint8_t flags;
    uint32_t encoded_bit_count, l4_len, l3_len, l2_len, l1_len;
    size_t encoded_size;
    ddc_c_btv tmp;
    ddc_c_status status;
    if (!ddc_c_read_u8(reader, &flags) || (flags & 0xF0u) != 0u ||
        ((flags & 0x08u) != 0u) != (is_last != 0u))
        return DDC_C_MALFORMED_INPUT;
    if (is_last) {
        if (!ddc_c_read_u32(reader, &encoded_bit_count) ||
            !ddc_c_u32_to_size(encoded_bit_count, &encoded_size) ||
            encoded_size != bit_count)
            return DDC_C_MALFORMED_INPUT;
    }
    if (!ddc_c_read_u32(reader, &l4_len) || !ddc_c_read_u32(reader, &l3_len) ||
        !ddc_c_read_u32(reader, &l2_len) || !ddc_c_read_u32(reader, &l1_len) ||
        !ddc_c_u32_to_size(l3_len, &encoded_size)) return DDC_C_MALFORMED_INPUT;
    ddc_c_btv_init(&tmp, flags & 0x01u, flags & 0x02u, DDC_C_COMPRESSED);
    tmp.l3_fill_ones = (flags & 0x04u) != 0;
    tmp.bit_count = bit_count;
    ddc_c_expected_level_counts(tmp.bit_count, &tmp.l2_count, &tmp.l3_count,
                                &tmp.l4_count);
    tmp.l3_lit_count = (size_t)l3_len;
    tmp.l2_lit_count = (size_t)l2_len;
    tmp.l1_lit_count = (size_t)l1_len;
    if (!ddc_c_read_bytes(reader, &tmp.l4_bits, (size_t)l4_len) ||
        !ddc_c_read_bytes(reader, &tmp.l3_lits, tmp.l3_lit_count) ||
        !ddc_c_read_bytes(reader, &tmp.l2_lits, tmp.l2_lit_count) ||
        !ddc_c_read_bytes(reader, &tmp.l1_lits, tmp.l1_lit_count)) {
        ddc_c_btv_destroy(&tmp);
        return DDC_C_MALFORMED_INPUT;
    }
    status = trusted ? DDC_C_OK : ddc_c_validate_compressed_btv(&tmp);
    if (status != DDC_C_OK) {
        ddc_c_btv_destroy(&tmp);
        return status;
    }
    ddc_c_btv_destroy(out);
    *out = tmp;
    return DDC_C_OK;
}

ddc_c_status ddc_c_serialize_packed(const ddc_c *ddc, uint8_t *out,
                                     size_t out_capacity, size_t *written) {
    ddc_c_writer writer = { out, out_capacity, 0, 0 };
    size_t i;
    ddc_c_status status;
    if (ddc == NULL || (out == NULL && out_capacity != 0)) return DDC_C_INVALID_ARGUMENT;
    ddc_c_write_u8(&writer, DDC_C_FMT_V3_MAGIC);
    ddc_c_write_u64(&writer, (uint64_t)ddc->bit_count);
    ddc_c_write_u64(&writer, (uint64_t)ddc->segment_bits);
    ddc_c_write_u64(&writer, (uint64_t)ddc->segment_count);
    for (i = 0; i < ddc->segment_count; ++i) {
        status = ddc_c_btv_write_packed(&writer, &ddc->segments[i], i + 1 == ddc->segment_count);
        if (status != DDC_C_OK) { if (written != NULL) *written = writer.pos; return status; }
    }
    if (written != NULL) *written = writer.pos;
    return DDC_C_OK;
}

ddc_c_status ddc_c_deserialize_packed(ddc_c *out, const uint8_t *data,
                                       size_t data_len, size_t *consumed) {
    ddc_c_reader reader = { data, data_len, 0 };
    ddc_c tmp;
    uint8_t magic;
    size_t count, i, expected_bits;
    ddc_c_status status;
    if (out == NULL || (data == NULL && data_len != 0)) return DDC_C_INVALID_ARGUMENT;
    ddc_c_init(&tmp);
    if (!ddc_c_read_u8(&reader, &magic) || magic != DDC_C_FMT_V3_MAGIC ||
        !ddc_c_read_size(&reader, &tmp.bit_count) ||
        !ddc_c_read_size(&reader, &tmp.segment_bits) ||
        !ddc_c_read_size(&reader, &count))
        goto malformed;
    status = ddc_c_validate_container_header(tmp.bit_count, tmp.segment_bits, count);
    if (status != DDC_C_OK) goto fail;
    status = ddc_c_reserve_segments(&tmp, count);
    if (status != DDC_C_OK) goto fail;
    for (i = 0; i < count; ++i) {
        if (!ddc_c_expected_segment_bits(tmp.bit_count, tmp.segment_bits, i, count,
                                         &expected_bits))
            goto malformed;
        status = ddc_c_btv_read_packed(&reader, &tmp.segments[tmp.segment_count],
                                       expected_bits, i + 1u == count, 0);
        if (status != DDC_C_OK) goto fail;
        ++tmp.segment_count;
    }
    ddc_c_destroy(out);
    *out = tmp;
    if (consumed != NULL) *consumed = reader.pos;
    return DDC_C_OK;
malformed:
    status = DDC_C_MALFORMED_INPUT;
fail:
    ddc_c_destroy(&tmp);
    return status;
}

static ddc_c_status ddc_c_btv_write_v4(ddc_c_writer *writer, const ddc_c_btv *btv,
                                        uint8_t is_last) {
    uint8_t flags = 0;
    if (btv == NULL || btv->state != DDC_C_COMPRESSED) return DDC_C_STATE_ERROR;
    if (btv->l1_fill_ones) flags |= 0x01u;
    if (btv->l2_fill_ones) flags |= 0x02u;
    if (btv->l3_fill_ones) flags |= 0x04u;
    if (is_last) flags |= 0x08u;
    if (btv->l1_lit_count != 0) flags |= 0x10u;
    if (btv->l2_lit_count != 0) flags |= 0x20u;
    if (btv->l3_lit_count != 0) flags |= 0x40u;
    ddc_c_write_u8(writer, flags);
    if (is_last) ddc_c_write_u32(writer, (uint32_t)btv->bit_count);
    if (btv->l3_lit_count != 0) ddc_c_write_u32(writer, (uint32_t)btv->l3_lit_count);
    if (btv->l2_lit_count != 0) ddc_c_write_u32(writer, (uint32_t)btv->l2_lit_count);
    if (btv->l1_lit_count != 0) ddc_c_write_u32(writer, (uint32_t)btv->l1_lit_count);
    ddc_c_write_raw(writer, btv->l4_bits.data, btv->l4_bits.len);
    ddc_c_write_raw(writer, btv->l3_lits.data, btv->l3_lit_count);
    ddc_c_write_raw(writer, btv->l2_lits.data, btv->l2_lit_count);
    ddc_c_write_raw(writer, btv->l1_lits.data, btv->l1_lit_count);
    return writer->overflow ? DDC_C_BUFFER_TOO_SMALL : DDC_C_OK;
}

static ddc_c_status ddc_c_btv_read_v4(ddc_c_reader *reader, ddc_c_btv *out,
                                       size_t bit_count, uint8_t is_last,
                                       uint8_t trusted) {
    uint8_t flags;
    uint32_t encoded_bit_count, l3_len = 0, l2_len = 0, l1_len = 0;
    size_t encoded_size, l4_len;
    ddc_c_btv tmp;
    ddc_c_status status;
    if (!ddc_c_read_u8(reader, &flags) || (flags & 0x80u) != 0u ||
        ((flags & 0x08u) != 0u) != (is_last != 0u))
        return DDC_C_MALFORMED_INPUT;
    if (is_last) {
        if (!ddc_c_read_u32(reader, &encoded_bit_count) ||
            !ddc_c_u32_to_size(encoded_bit_count, &encoded_size) ||
            encoded_size != bit_count)
            return DDC_C_MALFORMED_INPUT;
    }
    if ((flags & 0x40u) && !ddc_c_read_u32(reader, &l3_len)) return DDC_C_MALFORMED_INPUT;
    if ((flags & 0x20u) && !ddc_c_read_u32(reader, &l2_len)) return DDC_C_MALFORMED_INPUT;
    if ((flags & 0x10u) && !ddc_c_read_u32(reader, &l1_len)) return DDC_C_MALFORMED_INPUT;
    ddc_c_btv_init(&tmp, flags & 0x01u, flags & 0x02u, DDC_C_COMPRESSED);
    tmp.l3_fill_ones = (flags & 0x04u) != 0;
    tmp.bit_count = bit_count;
    ddc_c_expected_level_counts(tmp.bit_count, &tmp.l2_count, &tmp.l3_count,
                                &tmp.l4_count);
    ddc_c_bits_to_bytes(tmp.l4_count, &l4_len);
    tmp.l3_lit_count = (size_t)l3_len;
    tmp.l2_lit_count = (size_t)l2_len;
    tmp.l1_lit_count = (size_t)l1_len;
    if (!ddc_c_read_bytes(reader, &tmp.l4_bits, l4_len) ||
        !ddc_c_read_bytes(reader, &tmp.l3_lits, tmp.l3_lit_count) ||
        !ddc_c_read_bytes(reader, &tmp.l2_lits, tmp.l2_lit_count) ||
        !ddc_c_read_bytes(reader, &tmp.l1_lits, tmp.l1_lit_count)) {
        ddc_c_btv_destroy(&tmp);
        return DDC_C_MALFORMED_INPUT;
    }
    status = trusted ? DDC_C_OK : ddc_c_validate_compressed_btv(&tmp);
    if (status != DDC_C_OK) {
        ddc_c_btv_destroy(&tmp);
        return status;
    }
    ddc_c_btv_destroy(out);
    *out = tmp;
    return DDC_C_OK;
}

ddc_c_status ddc_c_serialize_v4(const ddc_c *ddc, uint8_t *out,
                                 size_t out_capacity, size_t *written) {
    ddc_c_writer writer = { out, out_capacity, 0, 0 };
    size_t i;
    ddc_c_status status;
    if (ddc == NULL || (out == NULL && out_capacity != 0)) return DDC_C_INVALID_ARGUMENT;
    ddc_c_write_u8(&writer, DDC_C_FMT_V4_MAGIC);
    ddc_c_write_u64(&writer, (uint64_t)ddc->bit_count);
    ddc_c_write_u64(&writer, (uint64_t)ddc->segment_bits);
    ddc_c_write_u64(&writer, (uint64_t)ddc->segment_count);
    for (i = 0; i < ddc->segment_count; ++i) {
        status = ddc_c_btv_write_v4(&writer, &ddc->segments[i], i + 1 == ddc->segment_count);
        if (status != DDC_C_OK) { if (written != NULL) *written = writer.pos; return status; }
    }
    if (written != NULL) *written = writer.pos;
    return DDC_C_OK;
}

ddc_c_status ddc_c_deserialize_v4(ddc_c *out, const uint8_t *data,
                                   size_t data_len, size_t *consumed) {
    ddc_c_reader reader = { data, data_len, 0 };
    ddc_c tmp;
    uint8_t magic;
    size_t count, i, expected_bits;
    ddc_c_status status;
    if (out == NULL || (data == NULL && data_len != 0)) return DDC_C_INVALID_ARGUMENT;
    ddc_c_init(&tmp);
    if (!ddc_c_read_u8(&reader, &magic) || magic != DDC_C_FMT_V4_MAGIC ||
        !ddc_c_read_size(&reader, &tmp.bit_count) ||
        !ddc_c_read_size(&reader, &tmp.segment_bits) ||
        !ddc_c_read_size(&reader, &count))
        goto malformed;
    status = ddc_c_validate_container_header(tmp.bit_count, tmp.segment_bits, count);
    if (status != DDC_C_OK) goto fail;
    status = ddc_c_reserve_segments(&tmp, count);
    if (status != DDC_C_OK) goto fail;
    for (i = 0; i < count; ++i) {
        if (!ddc_c_expected_segment_bits(tmp.bit_count, tmp.segment_bits, i, count,
                                         &expected_bits))
            goto malformed;
        status = ddc_c_btv_read_v4(&reader, &tmp.segments[tmp.segment_count],
                                   expected_bits, i + 1u == count, 0);
        if (status != DDC_C_OK) goto fail;
        ++tmp.segment_count;
    }
    ddc_c_destroy(out);
    *out = tmp;
    if (consumed != NULL) *consumed = reader.pos;
    return DDC_C_OK;
malformed:
    status = DDC_C_MALFORMED_INPUT;
fail:
    ddc_c_destroy(&tmp);
    return status;
}

/* Trusted variants: skip ddc_c_validate_compressed_btv per segment.
 * Use only when the source data is known to be well-formed (e.g. benchmark
 * files written by the same library). V1 format still validates (requires
 * popcount check to upgrade l3->l4). */

ddc_c_status ddc_c_deserialize_trusted(ddc_c *out, const uint8_t *data,
                                        size_t data_len, size_t *consumed) {
    ddc_c_reader reader = { data, data_len, 0 };
    ddc_c tmp;
    size_t count, i, expected_bits;
    ddc_c_status status;
    if (out == NULL || (data == NULL && data_len != 0)) return DDC_C_INVALID_ARGUMENT;
    ddc_c_init(&tmp);
    if (!ddc_c_read_size(&reader, &tmp.bit_count) ||
        !ddc_c_read_size(&reader, &tmp.segment_bits) ||
        !ddc_c_read_size(&reader, &count))
        goto malformed_t;
    status = ddc_c_validate_container_header(tmp.bit_count, tmp.segment_bits, count);
    if (status != DDC_C_OK) goto fail_t;
    status = ddc_c_reserve_segments(&tmp, count);
    if (status != DDC_C_OK) goto fail_t;
    for (i = 0; i < count; ++i) {
        if (!ddc_c_expected_segment_bits(tmp.bit_count, tmp.segment_bits, i, count,
                                         &expected_bits))
            goto malformed_t;
        status = ddc_c_btv_read_normal(&reader, &tmp.segments[tmp.segment_count], 1);
        if (status != DDC_C_OK) goto fail_t;
        ++tmp.segment_count;
        if (tmp.segments[tmp.segment_count - 1u].bit_count != expected_bits) {
            status = DDC_C_MALFORMED_INPUT;
            goto fail_t;
        }
    }
    ddc_c_destroy(out);
    *out = tmp;
    if (consumed != NULL) *consumed = reader.pos;
    return DDC_C_OK;
malformed_t:
    status = DDC_C_MALFORMED_INPUT;
fail_t:
    ddc_c_destroy(&tmp);
    return status;
}

ddc_c_status ddc_c_deserialize_packed_trusted(ddc_c *out, const uint8_t *data,
                                               size_t data_len, size_t *consumed) {
    ddc_c_reader reader = { data, data_len, 0 };
    ddc_c tmp;
    uint8_t magic;
    size_t count, i, expected_bits;
    ddc_c_status status;
    if (out == NULL || (data == NULL && data_len != 0)) return DDC_C_INVALID_ARGUMENT;
    ddc_c_init(&tmp);
    if (!ddc_c_read_u8(&reader, &magic) || magic != DDC_C_FMT_V3_MAGIC ||
        !ddc_c_read_size(&reader, &tmp.bit_count) ||
        !ddc_c_read_size(&reader, &tmp.segment_bits) ||
        !ddc_c_read_size(&reader, &count))
        goto malformed_pt;
    status = ddc_c_validate_container_header(tmp.bit_count, tmp.segment_bits, count);
    if (status != DDC_C_OK) goto fail_pt;
    status = ddc_c_reserve_segments(&tmp, count);
    if (status != DDC_C_OK) goto fail_pt;
    for (i = 0; i < count; ++i) {
        if (!ddc_c_expected_segment_bits(tmp.bit_count, tmp.segment_bits, i, count,
                                         &expected_bits))
            goto malformed_pt;
        status = ddc_c_btv_read_packed(&reader, &tmp.segments[tmp.segment_count],
                                       expected_bits, i + 1u == count, 1);
        if (status != DDC_C_OK) goto fail_pt;
        ++tmp.segment_count;
    }
    ddc_c_destroy(out);
    *out = tmp;
    if (consumed != NULL) *consumed = reader.pos;
    return DDC_C_OK;
malformed_pt:
    status = DDC_C_MALFORMED_INPUT;
fail_pt:
    ddc_c_destroy(&tmp);
    return status;
}
