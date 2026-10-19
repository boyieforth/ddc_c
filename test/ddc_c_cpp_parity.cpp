#include "ddc.h"
extern "C" {
#include "ddc_c.h"
}

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <array>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

using CSerializer = ddc_c_status (*)(const ddc_c *, uint8_t *, size_t, size_t *);
using CDeserializer = ddc_c_status (*)(ddc_c *, const uint8_t *, size_t, size_t *);
using CppSerializer = void (DDC::*)(std::ostream &) const;
using CppDeserializer = DDC (*)(std::istream &);

bool equal_bits(const std::vector<bool> &cpp, const std::vector<uint8_t> &c) {
    if (cpp.size() != c.size()) return false;
    for (size_t i = 0; i < cpp.size(); ++i)
        if (cpp[i] != (c[i] != 0)) return false;
    return true;
}

bool compare_wire(const std::vector<bool> &bits, const char *name,
                  CppSerializer cpp_serialize, CppDeserializer cpp_deserialize,
                  CSerializer c_serialize, CDeserializer c_deserialize) {
    DDC cpp = DDC::compress(bits);
    ddc_c c{};
    std::vector<uint8_t> bit_values(bits.size());
    std::vector<uint8_t> c_wire(bits.size() * 3 + 1024);
    size_t c_written = 0;
    std::ostringstream cpp_stream(std::ios::binary);

    for (size_t i = 0; i < bits.size(); ++i) bit_values[i] = bits[i] ? 1 : 0;
    if (ddc_c_compress(&c, bit_values.data(), bit_values.size(), 0,
                       DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK) {
        std::cerr << name << ": C compression failed\n";
        return false;
    }

    (cpp.*cpp_serialize)(cpp_stream);
    const std::string cpp_wire = cpp_stream.str();
    if (c_serialize(&c, c_wire.data(), c_wire.size(), &c_written) != DDC_C_OK ||
        c_written != cpp_wire.size() ||
        std::memcmp(c_wire.data(), cpp_wire.data(), c_written) != 0) {
        std::cerr << name << ": C serialization does not match C++ bytes\n";
        ddc_c_destroy(&c);
        return false;
    }

    {
        ddc_c c_from_cpp{};
        size_t consumed = 0;
        if (c_deserialize(&c_from_cpp,
                          reinterpret_cast<const uint8_t *>(cpp_wire.data()),
                          cpp_wire.size(), &consumed) != DDC_C_OK ||
            consumed != cpp_wire.size()) {
            std::cerr << name << ": C cannot read C++ bytes\n";
            ddc_c_destroy(&c_from_cpp);
            ddc_c_destroy(&c);
            return false;
        }
        std::vector<uint8_t> decoded(bits.size());
        if (ddc_c_decompress(&c_from_cpp, decoded.data(), decoded.size()) != DDC_C_OK ||
            !equal_bits(bits, decoded)) {
            std::cerr << name << ": C decode of C++ bytes differs\n";
            ddc_c_destroy(&c_from_cpp);
            ddc_c_destroy(&c);
            return false;
        }
        ddc_c_destroy(&c_from_cpp);
    }

    {
        std::istringstream c_stream(std::string(
            reinterpret_cast<const char *>(c_wire.data()), c_written), std::ios::binary);
        DDC cpp_from_c = cpp_deserialize(c_stream);
        if (cpp_from_c.decompress() != bits) {
            std::cerr << name << ": C++ decode of C bytes differs\n";
            ddc_c_destroy(&c);
            return false;
        }
    }

    ddc_c_destroy(&c);
    return true;
}

bool c_compress(ddc_c *out, const std::vector<bool> &bits, bool fill_ones = false,
                size_t segment_bits = DDC_C_DEFAULT_SEGMENT_BITS) {
    std::vector<uint8_t> values(bits.size());
    for (size_t i = 0; i < bits.size(); ++i) values[i] = bits[i] ? 1 : 0;
    return ddc_c_compress(out, values.data(), values.size(), fill_ones, segment_bits) == DDC_C_OK;
}

bool c_matches(const ddc_c &actual, const std::vector<bool> &expected, const char *name) {
    std::vector<uint8_t> decoded(expected.size());
    if (ddc_c_decompress(&actual, decoded.data(), decoded.size()) != DDC_C_OK ||
        !equal_bits(expected, decoded)) {
        std::cerr << name << ": C result differs\n";
        return false;
    }
    return true;
}

void collect_set_bit(void *ctx, size_t position) {
    static_cast<std::vector<size_t> *>(ctx)->push_back(position);
}

bool compare_set_bit_positions(const std::vector<bool> &bits, bool fill_ones,
                               size_t segment_bits, const char *name) {
    const DDC cpp = DDC::compress(bits, fill_ones, segment_bits);
    const std::vector<size_t> expected = cpp.set_bit_positions();
    ddc_c c{};
    size_t required = 0;
    std::vector<size_t> actual(expected.size());
    std::vector<size_t> callback_positions;
    const auto finish = [&] (bool success) {
        ddc_c_destroy(&c);
        return success;
    };

    if (!c_compress(&c, bits, fill_ones, segment_bits)) {
        std::cerr << name << ": C compression failed\n";
        return finish(false);
    }
    if (ddc_c_for_each_set_bit(&c, collect_set_bit, &callback_positions) != DDC_C_OK ||
        callback_positions != expected) {
        std::cerr << name << ": callback traversal differs\n";
        return finish(false);
    }
    if (ddc_c_for_each_set_bit(&c, nullptr, nullptr) != DDC_C_INVALID_ARGUMENT ||
        ddc_c_set_bit_positions(&c, nullptr, 0, nullptr) != DDC_C_INVALID_ARGUMENT) {
        std::cerr << name << ": null callback/count contract differs\n";
        return finish(false);
    }
    const ddc_c_status probe = ddc_c_set_bit_positions(&c, nullptr, 0, &required);
    if (required != expected.size() ||
        probe != (expected.empty() ? DDC_C_OK : DDC_C_BUFFER_TOO_SMALL)) {
        std::cerr << name << ": position count probe differs\n";
        return finish(false);
    }
    if (!expected.empty()) {
        std::vector<size_t> untouched(expected.size(), SIZE_MAX);
        size_t small_count = 0;
        if (ddc_c_set_bit_positions(&c, untouched.data(), expected.size() - 1,
                                    &small_count) != DDC_C_BUFFER_TOO_SMALL ||
            small_count != expected.size() ||
            std::any_of(untouched.begin(), untouched.end(),
                        [] (size_t value) { return value != SIZE_MAX; })) {
            std::cerr << name << ": insufficient-buffer contract differs\n";
            return finish(false);
        }
    }
    if (ddc_c_set_bit_positions(&c, actual.data(), actual.size(), &required) != DDC_C_OK ||
        required != expected.size() || actual != expected) {
        std::cerr << name << ": materialized positions differ\n";
        return finish(false);
    }
    return finish(true);
}

bool compare_statistics_and_metadata(const std::vector<bool> &a_bits,
                                     const std::vector<bool> &b_bits,
                                     size_t segment_bits) {
    const DDC cpp_a = DDC::compress(a_bits, false, segment_bits);
    const DDC cpp_b = DDC::compress(b_bits, true, segment_bits);
    ddc_c c_a{}, c_b{};
    ddc_c_size_info c_size{};
    const auto finish = [&] (bool success) {
        ddc_c_destroy(&c_a);
        ddc_c_destroy(&c_b);
        return success;
    };
    const auto cpp_size = cpp_a.size_breakdown();
    const size_t expected_popcount_and = (cpp_a & cpp_b).popcount();

    if (!c_compress(&c_a, a_bits, false, segment_bits) ||
        !c_compress(&c_b, b_bits, true, segment_bits))
        return finish(false);
    ddc_c_size_breakdown(&c_a, &c_size);
    if (c_a.bit_count != cpp_a.bit_count() ||
        c_a.segment_bits != cpp_a.segment_bits() ||
        c_a.segment_count != cpp_a.num_segments() ||
        ddc_c_popcount(&c_a) != cpp_a.popcount() ||
        ddc_c_popcount_and(&c_a, &c_b) != expected_popcount_and ||
        c_size.l3_bits != cpp_size.l3_bits || c_size.l4_bits != cpp_size.l4_bits ||
        c_size.l3_literal_bits != cpp_size.l3_literal_bits ||
        c_size.l2_literal_bits != cpp_size.l2_literal_bits ||
        c_size.l1_literal_bits != cpp_size.l1_literal_bits ||
        c_size.total_bits != cpp_size.total_bits ||
        std::fabs(ddc_c_compression_ratio(&c_a) - cpp_a.compression_ratio()) > 1e-12) {
        std::cerr << "statistics or metadata differs: C bits=" << c_a.bit_count
                  << " segbits=" << c_a.segment_bits << " segments=" << c_a.segment_count
                  << " pop=" << ddc_c_popcount(&c_a) << " cpp-pop=" << cpp_a.popcount()
                  << " pc-and=" << ddc_c_popcount_and(&c_a, &c_b)
                  << " cpp-pc-and=" << expected_popcount_and
                  << " size=" << c_size.l3_bits << '/' << c_size.l4_bits << '/'
                  << c_size.l3_literal_bits << '/' << c_size.l2_literal_bits << '/'
                  << c_size.l1_literal_bits << '/' << c_size.total_bits
                  << " cpp-size=" << cpp_size.l3_bits << '/' << cpp_size.l4_bits << '/'
                  << cpp_size.l3_literal_bits << '/' << cpp_size.l2_literal_bits << '/'
                  << cpp_size.l1_literal_bits << '/' << cpp_size.total_bits << '\n';
        return finish(false);
    }
    for (size_t segment = 0; segment < c_a.segment_count; ++segment) {
        const auto &cpp_segment = cpp_a.segment(segment);
        const ddc_c_btv &c_segment = c_a.segments[segment];
        if (c_segment.state != static_cast<ddc_c_state>(cpp_segment.state()) ||
            c_segment.bit_count != cpp_segment.bit_count() ||
            c_segment.l2_count != cpp_segment.l2_count() ||
            c_segment.l3_count != cpp_segment.l3_count() ||
            c_segment.l4_count != cpp_segment.l4_count() ||
            c_segment.l1_fill_ones != cpp_segment.l1_fill_ones() ||
            c_segment.l2_fill_ones != cpp_segment.l2_fill_ones() ||
            c_segment.l3_fill_ones != cpp_segment.l3_fill_ones() ||
            c_segment.l1_lit_count != cpp_segment.num_lits() ||
            c_segment.l2_lit_count != cpp_segment.l2_lit_count() ||
            c_segment.l3_lit_count != cpp_segment.l3_lit_count() ||
            ddc_c_btv_num_fills(&c_segment) != cpp_segment.num_fills() ||
            ddc_c_btv_is_all_zero(&c_segment) != cpp_segment.is_all_zero() ||
            ddc_c_btv_is_all_ones(&c_segment) != cpp_segment.is_all_ones()) {
            std::cerr << "segment metadata differs\n";
            return finish(false);
        }
    }
    return finish(true);
}

bool compare_v1_fixture() {
    const std::vector<bool> bits{true, false, true, false, false, true, false, true,
                                 false, true, false, false, true};
    const DDC cpp = DDC::compress(bits);
    std::ostringstream stream(std::ios::binary);
    ddc_c c{};
    size_t consumed = 0;
    cpp.serialize(stream);
    std::string fixture = stream.str();
    if (fixture.size() < 25) return false;
    /* Each V2 segment starts after the 24-byte container header. Replacing
       its format byte with V1 is a valid fixture only for a V1-compatible
       layout, so construct the expected V1 stream manually below instead. */
    (void)fixture;

    const uint64_t bit_count = bits.size();
    const uint64_t segment_bits = DDC::default_segment_bits;
    const uint64_t segment_count = 1;
    const uint8_t format = 8, l1_fill = 0, l2_fill = 0;
    const uint64_t l2_count = 2, l3_count = 1, l3_bytes = 1;
    const uint8_t l3_data = 0x01, l2_data = 0x03;
    const uint64_t l2_lit_count = 1, l1_lit_count = 2;
    const uint8_t l1_data[] = {0xA5, 0x48};
    std::vector<uint8_t> wire;
    const auto append = [&wire] (const auto &value) {
        const uint8_t *data = reinterpret_cast<const uint8_t *>(&value);
        wire.insert(wire.end(), data, data + sizeof(value));
    };
    append(bit_count); append(segment_bits); append(segment_count);
    append(format); append(l1_fill); append(l2_fill);
    append(bit_count); append(l2_count); append(l3_count); append(l3_bytes);
    append(l3_data); append(l2_lit_count); append(l2_data); append(l1_lit_count);
    wire.insert(wire.end(), l1_data, l1_data + sizeof(l1_data));
    {
        std::istringstream cpp_stream(std::string(
            reinterpret_cast<const char *>(wire.data()), wire.size()), std::ios::binary);
        if (DDC::deserialize(cpp_stream).decompress() != bits) return false;
    }
    if (ddc_c_deserialize(&c, wire.data(), wire.size(), &consumed) != DDC_C_OK ||
        consumed != wire.size() || !c_matches(c, bits, "V1 fixture")) {
        ddc_c_destroy(&c);
        return false;
    }
    ddc_c_destroy(&c);
    return true;
}

bool compare_fill_inputs() {
    constexpr size_t bit_count = DDC::default_segment_bits + 13;
    std::vector<bool> all_zero(bit_count, false), all_one(bit_count, true), mixed(bit_count);
    ddc_c c_zero{}, c_one{}, c_mixed{}, c_result{};
    const auto finish = [&] (bool success) {
        ddc_c_destroy(&c_zero);
        ddc_c_destroy(&c_one);
        ddc_c_destroy(&c_mixed);
        ddc_c_destroy(&c_result);
        return success;
    };

    for (size_t i = 0; i < bit_count; ++i)
        mixed[i] = i % 11 == 0 || (i >= 65520 && i < 65555);
    if (!c_compress(&c_zero, all_zero) || !c_compress(&c_one, all_one, true) ||
        !c_compress(&c_mixed, mixed, true))
        return finish(false);

    for (const uint8_t compress_results : {uint8_t{0}, uint8_t{1}}) {
        ddc_compress_results = compress_results != 0;
        ddc_c_set_compress_results(compress_results);
        struct Input { const std::vector<bool> *bits; ddc_c *c; bool fill_ones; };
        const std::array<Input, 3> inputs{{
            {&all_zero, &c_zero, false}, {&all_one, &c_one, true},
            {&mixed, &c_mixed, true}
        }};
        for (const auto &[cpp_bits_a, c_a, fill_a] : inputs) {
            const DDC cpp_a = DDC::compress(*cpp_bits_a, fill_a);
            for (const auto &[cpp_bits_b, c_b, fill_b] : inputs) {
                const DDC cpp_b = DDC::compress(*cpp_bits_b, fill_b);
                const DDC cpp_and = cpp_a & cpp_b;
                if (ddc_c_and(&c_result, c_a, c_b) != DDC_C_OK ||
                    !c_matches(c_result, cpp_and.decompress(), "fill AND"))
                    return finish(false);
                ddc_c_destroy(&c_result);

                const DDC cpp_or = cpp_a | cpp_b;
                if (ddc_c_or(&c_result, c_a, c_b) != DDC_C_OK ||
                    !c_matches(c_result, cpp_or.decompress(), "fill OR"))
                    return finish(false);
                ddc_c_destroy(&c_result);

                const DDC cpp_xor = cpp_a ^ cpp_b;
                if (ddc_c_xor(&c_result, c_a, c_b) != DDC_C_OK ||
                    !c_matches(c_result, cpp_xor.decompress(), "fill XOR"))
                    return finish(false);
                ddc_c_destroy(&c_result);
            }
        }
    }
    ddc_compress_results = false;
    ddc_c_set_compress_results(0);
    return finish(true);
}

bool compare_sparse() {
    constexpr size_t bit_count = DDC::default_segment_bits * 2 + 19;
    const std::array<std::vector<uint32_t>, 3> position_sets{{
        {0, 3, 5, 65535, 65536, 131090},
        {1, 5, 42, 65534, 65536, 70000, 131090},
        {2, 3, 65535, 90000, 131089, 131090}
    }};
    std::array<ddc_c_sparse, position_sets.size()> sparse{};
    std::array<const ddc_c_sparse *, position_sets.size()> sparse_ptrs{};
    ddc_c sparse_result{};
    ddc_c dense_result{};
    const auto finish = [&] (bool success) {
        for (auto &value : sparse) ddc_c_sparse_destroy(&value);
        ddc_c_destroy(&sparse_result);
        ddc_c_destroy(&dense_result);
        return success;
    };

    std::vector<bool> expected(bit_count, false);
    for (size_t i = 0; i < position_sets.size(); ++i) {
        if (ddc_c_sparse_from_positions(&sparse[i], position_sets[i].data(),
                                        position_sets[i].size(), bit_count,
                                        DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK)
            return finish(false);
        sparse_ptrs[i] = &sparse[i];
        for (const uint32_t position : position_sets[i]) expected[position] = true;
    }
    {
        const ddc_c_status status = ddc_c_sparse_or_many(
            &sparse_result, sparse_ptrs.size(), sparse_ptrs.data(), bit_count,
            DDC_C_DEFAULT_SEGMENT_BITS);
        if (status != DDC_C_OK) {
            std::cerr << "sparse OR-many status " << status << "\n";
            return finish(false);
        }
        if (!c_matches(sparse_result, expected, "sparse OR-many"))
            return finish(false);
    }

    if (!c_compress(&dense_result, std::vector<bool>(bit_count, false))) {
        std::cerr << "sparse dense initialization failed\n";
        return finish(false);
    }
    for (const auto &value : sparse) {
        const ddc_c_status status = ddc_c_sparse_apply_or_to(&value, &dense_result);
        if (status != DDC_C_OK) {
            std::cerr << "sparse apply status " << status << "\n";
            return finish(false);
        }
    }
    {
        const bool success = c_matches(dense_result, expected, "sparse apply");
        return finish(success);
    }
}

bool compare_sparse_edge_cases() {
    constexpr size_t bit_count = DDC::default_segment_bits + 19;
    const std::array<std::vector<uint32_t>, 3> inputs{{
        {}, {65536, 1, 1, 0, 65536, 65554, 7}, {2, 65535, 2, 17, 65536}
    }};
    std::array<ddc_c_sparse, inputs.size()> c_sparse{};
    std::array<const ddc_c_sparse *, inputs.size()> sparse_ptrs{};
    ddc_c c_result{}, c_dense{};
    std::vector<bool> expected(bit_count, false);
    const auto finish = [&] (bool success) {
        for (auto &sparse : c_sparse) ddc_c_sparse_destroy(&sparse);
        ddc_c_destroy(&c_result);
        ddc_c_destroy(&c_dense);
        return success;
    };

    for (size_t i = 0; i < inputs.size(); ++i) {
        const auto &positions = inputs[i];
        if (ddc_c_sparse_from_positions(&c_sparse[i],
                                        positions.empty() ? nullptr : positions.data(),
                                        positions.size(), bit_count,
                                        DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK)
            return finish(false);
        sparse_ptrs[i] = &c_sparse[i];
        for (uint32_t position : positions) expected[position] = true;
        const SparseDDC cpp = SparseDDC::from_positions(
            positions, bit_count, DDC::default_segment_bits);
        if (c_sparse[i].bit_count != cpp.bit_count() ||
            c_sparse[i].segment_bits != cpp.segment_bits() ||
            c_sparse[i].num_set_bits != cpp.num_set_bits() ||
            c_sparse[i].segment_count != cpp.num_non_empty_segments() ||
            ddc_c_sparse_storage_bytes(&c_sparse[i]) < sizeof(ddc_c_sparse)) {
            std::cerr << "sparse metadata differs\n";
            return finish(false);
        }
    }
    if (ddc_c_sparse_or_many(&c_result, sparse_ptrs.size(), sparse_ptrs.data(),
                             bit_count, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK ||
        !c_matches(c_result, expected, "sparse edge OR-many"))
        return finish(false);
    if (!c_compress(&c_dense, std::vector<bool>(bit_count, false))) return finish(false);
    for (const ddc_c_sparse &sparse : c_sparse) {
        if (ddc_c_sparse_apply_or_to(&sparse, &c_dense) != DDC_C_OK)
            return finish(false);
    }
    if (!c_matches(c_dense, expected, "sparse edge apply")) return finish(false);

    const uint32_t out_of_range[] = {static_cast<uint32_t>(bit_count)};
    if (ddc_c_sparse_from_positions(&c_sparse[0], out_of_range, 1, bit_count,
                                    DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_INVALID_ARGUMENT ||
        ddc_c_sparse_or_many(&c_result, 0, nullptr, bit_count,
                             DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK ||
        !c_matches(c_result, std::vector<bool>(bit_count, false), "empty sparse OR-many"))
        return finish(false);
    return finish(true);
}

bool compare_inplace() {
    constexpr size_t bit_count = DDC::default_segment_bits * 2 + 19;
    std::vector<bool> a_bits(bit_count), b_bits(bit_count);
    std::vector<uint32_t> a_positions;
    ddc_c_sparse c_sparse_a{};
    ddc_c c_rhs{}, c_lhs{};
    const auto finish = [&] (bool success) {
        ddc_c_sparse_destroy(&c_sparse_a);
        ddc_c_destroy(&c_rhs);
        ddc_c_destroy(&c_lhs);
        return success;
    };
    for (size_t i = 0; i < bit_count; ++i) {
        a_bits[i] = i % 13 == 0 || (i >= 65525 && i < 65555);
        b_bits[i] = i % 17 == 0 || (i >= 131000 && i < 131090);
        if (a_bits[i]) a_positions.push_back(static_cast<uint32_t>(i));
    }
    if (!c_compress(&c_rhs, b_bits) ||
        ddc_c_sparse_from_positions(&c_sparse_a, a_positions.data(), a_positions.size(),
                                    bit_count, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK)
        return finish(false);

    for (const uint8_t compress_results : {uint8_t{0}, uint8_t{1}}) {
        ddc_compress_results = compress_results != 0;
        ddc_c_set_compress_results(compress_results);
        const DDC cpp_rhs = DDC::compress(b_bits);
        const SparseDDC cpp_sparse_a = SparseDDC::from_positions(
            a_positions, bit_count, DDC::default_segment_bits);
        const std::array<ddc_c_status (*)(ddc_c *, const ddc_c *), 3> c_ops{{
            ddc_c_and_inplace, ddc_c_or_inplace, ddc_c_xor_inplace
        }};
        for (size_t op = 0; op < c_ops.size(); ++op) {
            DDC cpp_lhs = DDC::from_sparse_positions({}, bit_count,
                                                       DDC::default_segment_bits);
            std::vector<bool> expected;
            cpp_sparse_a.apply_or_to(cpp_lhs);
            ddc_c_destroy(&c_lhs);
            if (ddc_c_from_sparse_positions(&c_lhs, nullptr, 0, bit_count,
                                            DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK ||
                ddc_c_sparse_apply_or_to(&c_sparse_a, &c_lhs) != DDC_C_OK)
                return finish(false);
            if (op == 0) {
                cpp_lhs &= cpp_rhs;
                expected = cpp_lhs.decompress();
            } else if (op == 1) {
                cpp_lhs |= cpp_rhs;
                expected = cpp_lhs.decompress();
            } else {
                /* C++ operator^= assumes compressed segment inputs and does
                   not support the decompressed sparse staging form used by
                   its own in-place AND/OR paths. Use its compressed binary
                   reference for the same logical result. */
                expected = (DDC::compress(a_bits) ^ cpp_rhs).decompress();
            }
            if (c_ops[op](&c_lhs, &c_rhs) != DDC_C_OK ||
                !c_matches(c_lhs, expected, "in-place operation") ||
                (op < 2 && c_lhs.segment_count != 0 &&
                 c_lhs.segments[0].state != DDC_C_DECOMPRESSED))
                return finish(false);
        }

        DDC cpp_lhs = DDC::from_sparse_positions({}, bit_count,
                                                   DDC::default_segment_bits);
        cpp_sparse_a.apply_or_to(cpp_lhs);
        ddc_c_destroy(&c_lhs);
        if (ddc_c_from_sparse_positions(&c_lhs, nullptr, 0, bit_count,
                                        DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK ||
            ddc_c_sparse_apply_or_to(&c_sparse_a, &c_lhs) != DDC_C_OK ||
            ddc_c_not_inplace(&c_lhs) != DDC_C_OK ||
            !c_matches(c_lhs, (~cpp_lhs).decompress(), "in-place NOT"))
            return finish(false);
    }
    ddc_compress_results = false;
    ddc_c_set_compress_results(0);
    return finish(true);
}

bool compare_ops(const std::vector<bool> &a_bits, const std::vector<bool> &b_bits) {
    DDC cpp_a = DDC::compress(a_bits);
    DDC cpp_b = DDC::compress(b_bits);
    std::vector<uint8_t> a(a_bits.size()), b(b_bits.size());
    ddc_c c_a{}, c_b{}, c_result{};
    const auto finish = [&c_a, &c_b, &c_result](bool success) {
        ddc_c_destroy(&c_a);
        ddc_c_destroy(&c_b);
        ddc_c_destroy(&c_result);
        return success;
    };

    for (size_t i = 0; i < a.size(); ++i) {
        a[i] = a_bits[i] ? 1 : 0;
        b[i] = b_bits[i] ? 1 : 0;
    }
    if (ddc_c_compress(&c_a, a.data(), a.size(), 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK ||
        ddc_c_compress(&c_b, b.data(), b.size(), 0, DDC_C_DEFAULT_SEGMENT_BITS) != DDC_C_OK)
        return finish(false);

    for (const uint8_t compress_results : {uint8_t{0}, uint8_t{1}}) {
        ddc_compress_results = compress_results != 0;
        ddc_c_set_compress_results(compress_results);

        {
            const DDC cpp_and = cpp_a & cpp_b;
            if (ddc_c_and(&c_result, &c_a, &c_b) != DDC_C_OK ||
                !c_matches(c_result, cpp_and.decompress(), "AND"))
                return finish(false);
            ddc_c_destroy(&c_result);
        }

        {
            const DDC cpp_or = cpp_a | cpp_b;
            if (ddc_c_or(&c_result, &c_a, &c_b) != DDC_C_OK ||
                !c_matches(c_result, cpp_or.decompress(), "OR"))
                return finish(false);
            ddc_c_destroy(&c_result);
        }

        {
            const DDC cpp_xor = cpp_a ^ cpp_b;
            if (ddc_c_xor(&c_result, &c_a, &c_b) != DDC_C_OK ||
                !c_matches(c_result, cpp_xor.decompress(), "XOR"))
                return finish(false);
            ddc_c_destroy(&c_result);
        }
    }
    ddc_compress_results = false;
    ddc_c_set_compress_results(0);

    {
        const DDC cpp_not = ~cpp_a;
        if (ddc_c_not(&c_result, &c_a) != DDC_C_OK ||
            !c_matches(c_result, cpp_not.decompress(), "NOT"))
            return finish(false);
    }
    return finish(true);
}

} // namespace

int main() {
    constexpr size_t bit_count = DDC::default_segment_bits * 2 + 19;
    std::vector<bool> a(bit_count), b(bit_count);
    for (size_t i = 0; i < bit_count; ++i) {
        a[i] = i % 19 == 0 || (i >= 65500 && i < 65690) || i % 997 == 3;
        b[i] = i % 31 == 0 || (i >= 131000 && i < 131090) || i % 643 == 7;
    }

    if (!compare_wire(a, "normal V2", &DDC::serialize, &DDC::deserialize,
                      ddc_c_serialize, ddc_c_deserialize))
        return EXIT_FAILURE;
    if (!compare_v1_fixture()) {
        std::cerr << "V1 fixture parity failed\n";
        return EXIT_FAILURE;
    }
    if (!compare_statistics_and_metadata(a, b, DDC_C_DEFAULT_SEGMENT_BITS) ||
        !compare_statistics_and_metadata(a, b, 19)) {
        std::cerr << "statistics or metadata parity failed\n";
        return EXIT_FAILURE;
    }
    if (!compare_wire(a, "packed V3", &DDC::serialize_packed,
                      &DDC::deserialize_packed, ddc_c_serialize_packed,
                      ddc_c_deserialize_packed))
        return EXIT_FAILURE;
    if (!compare_wire(a, "packed V4", &DDC::serialize_v4, &DDC::deserialize_v4,
                      ddc_c_serialize_v4, ddc_c_deserialize_v4))
        return EXIT_FAILURE;
    if (!compare_fill_inputs()) {
        std::cerr << "fill-input parity failed\n";
        return EXIT_FAILURE;
    }
    if (!compare_sparse() || !compare_sparse_edge_cases()) {
        std::cerr << "sparse parity failed\n";
        return EXIT_FAILURE;
    }
    if (!compare_inplace()) {
        std::cerr << "in-place parity failed\n";
        return EXIT_FAILURE;
    }
    const std::array<std::vector<bool>, 10> traversal_inputs{{
        {}, {false}, {true},
        std::vector<bool>(7, true), std::vector<bool>(8, true),
        std::vector<bool>(9, true),
        std::vector<bool>(65535, false), std::vector<bool>(65536, true),
        std::vector<bool>(65537, false), a
    }};
    for (size_t i = 0; i < traversal_inputs.size(); ++i) {
        std::vector<bool> input = traversal_inputs[i];
        if (i == 6) input.back() = true;
        if (i == 8) input.back() = true;
        if (i == 9) {
            for (size_t pos = 0; pos < input.size(); pos += 113)
                input[pos] = true;
        }
        if (!compare_set_bit_positions(input, i == 5 || i == 7,
                                       i == 9 ? 19 : DDC_C_DEFAULT_SEGMENT_BITS,
                                       "set-bit positions"))
            return EXIT_FAILURE;
    }
    if (!compare_ops(a, b))
        return EXIT_FAILURE;

    std::cout << "ddc_c C++ parity test passed\n";
    return EXIT_SUCCESS;
}
