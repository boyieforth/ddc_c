#include "ddc.h"
#include "ddc_c.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#if defined(__linux__)
#include <sys/utsname.h>
#endif

#ifndef DDC_BENCH_AVX512_VBMI2
#define DDC_BENCH_AVX512_VBMI2 0
#endif

namespace {

using Clock = std::chrono::steady_clock;

constexpr size_t kBitCount = DDC::default_segment_bits * 2 + 37;
constexpr size_t kSegmentBits = DDC::default_segment_bits;
constexpr uint8_t kL1FillOnes = 0;

enum class Operation { And, Or, Xor, Not };
enum class Implementation { Cpp, C };

struct Config {
    size_t warmup = 5;
    size_t rounds = 11;
    size_t iterations = 100;
    bool profile = false;
    Implementation profile_implementation = Implementation::C;
    Operation profile_operation = Operation::And;
    std::string profile_workload;
    bool profile_compress_results = false;
};

struct CResult {
    ddc_c value{};

    CResult() { ddc_c_init(&value); }
    ~CResult() { ddc_c_destroy(&value); }

    CResult(const CResult&) = delete;
    CResult& operator=(const CResult&) = delete;
};

struct Workload {
    std::string name;
    size_t a_set_bits = 0;
    size_t b_set_bits = 0;
    DDC cpp_a;
    DDC cpp_b;
    ddc_c c_a{};
    ddc_c c_b{};

    Workload() {
        ddc_c_init(&c_a);
        ddc_c_init(&c_b);
    }
    ~Workload() {
        ddc_c_destroy(&c_a);
        ddc_c_destroy(&c_b);
    }

    Workload(const Workload&) = delete;
    Workload& operator=(const Workload&) = delete;
};

struct ResultPolicyGuard {
    bool cpp_previous = ddc_compress_results;
    uint8_t c_previous = ddc_c_get_compress_results();

    ~ResultPolicyGuard() {
        ddc_compress_results = cpp_previous;
        ddc_c_set_compress_results(c_previous);
    }
};

struct Summary {
    double min_ns = 0.0;
    double median_ns = 0.0;
    double max_ns = 0.0;
    double stddev_ns = 0.0;
};

[[noreturn]] void fail(const std::string& message) {
    throw std::runtime_error(message);
}

void check_status(ddc_c_status status, const std::string& context) {
    if (status != DDC_C_OK)
        fail(context + " failed with C status " + std::to_string(status));
}

void do_not_optimize(const void* value) {
#if defined(__GNUC__) || defined(__clang__)
    asm volatile("" : : "g"(value) : "memory");
#else
    volatile const void* sink = value;
    (void)sink;
#endif
}

uint64_t mix64(uint64_t value) {
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31);
}

uint32_t probability_sample(size_t position, uint64_t seed) {
    return static_cast<uint32_t>(mix64(static_cast<uint64_t>(position) ^ seed) % 10000U);
}

size_t count_set_bits(const std::vector<bool>& bits) {
    return static_cast<size_t>(std::count(bits.begin(), bits.end(), true));
}

std::vector<uint8_t> as_c_bits(const std::vector<bool>& bits) {
    std::vector<uint8_t> result(bits.size());
    for (size_t i = 0; i < bits.size(); ++i)
        result[i] = bits[i] ? 1 : 0;
    return result;
}

void set_boundary_bits(std::vector<bool>& a, std::vector<bool>& b) {
    const std::array<size_t, 7> positions{{
        0, 1, kSegmentBits - 1, kSegmentBits, kSegmentBits + 1,
        kBitCount - 2, kBitCount - 1
    }};
    for (size_t i = 0; i < positions.size(); ++i) {
        if ((i % 2) == 0)
            a[positions[i]] = true;
        else
            b[positions[i]] = true;
    }
}

void make_workload(Workload& workload, const std::string& name,
                   uint32_t a_threshold, uint32_t b_threshold,
                   bool force_zero, bool force_one) {
    std::vector<bool> a(kBitCount, false);
    std::vector<bool> b(kBitCount, false);

    if (force_one) {
        std::fill(a.begin(), a.end(), true);
        std::fill(b.begin(), b.end(), true);
    } else if (!force_zero) {
        for (size_t position = 0; position < kBitCount; ++position) {
            a[position] = probability_sample(position, 0x243f6a8885a308d3ULL) < a_threshold;
            b[position] = probability_sample(position, 0x13198a2e03707344ULL) < b_threshold;
        }
        set_boundary_bits(a, b);
    }

    const std::vector<uint8_t> c_a_bits = as_c_bits(a);
    const std::vector<uint8_t> c_b_bits = as_c_bits(b);
    workload.name = name;
    workload.a_set_bits = count_set_bits(a);
    workload.b_set_bits = count_set_bits(b);
    workload.cpp_a = DDC::compress(a, kL1FillOnes, kSegmentBits);
    workload.cpp_b = DDC::compress(b, kL1FillOnes, kSegmentBits);
    check_status(ddc_c_compress(&workload.c_a, c_a_bits.data(), c_a_bits.size(),
                                kL1FillOnes, kSegmentBits),
                 name + " C input A compression");
    check_status(ddc_c_compress(&workload.c_b, c_b_bits.data(), c_b_bits.size(),
                                kL1FillOnes, kSegmentBits),
                 name + " C input B compression");
}

const char* operation_name(Operation operation) {
    switch (operation) {
    case Operation::And: return "and";
    case Operation::Or: return "or";
    case Operation::Xor: return "xor";
    case Operation::Not: return "not";
    }
    return "unknown";
}

DDC run_cpp_operation(Operation operation, const Workload& workload) {
    switch (operation) {
    case Operation::And: return workload.cpp_a & workload.cpp_b;
    case Operation::Or: return workload.cpp_a | workload.cpp_b;
    case Operation::Xor: return workload.cpp_a ^ workload.cpp_b;
    case Operation::Not: return ~workload.cpp_a;
    }
    fail("unsupported C++ benchmark operation");
}

void run_c_operation(Operation operation, const Workload& workload, ddc_c* result) {
    ddc_c_status status = DDC_C_STATE_ERROR;
    switch (operation) {
    case Operation::And: status = ddc_c_and(result, &workload.c_a, &workload.c_b); break;
    case Operation::Or: status = ddc_c_or(result, &workload.c_a, &workload.c_b); break;
    case Operation::Xor: status = ddc_c_xor(result, &workload.c_a, &workload.c_b); break;
    case Operation::Not: status = ddc_c_not(result, &workload.c_a); break;
    }
    check_status(status, std::string("C ") + operation_name(operation));
}

bool same_size_info(const ddc_c_size_info& c, const DDC::SizeBreakdown& cpp) {
    return c.l3_bits == cpp.l3_bits && c.l4_bits == cpp.l4_bits &&
           c.l3_literal_bits == cpp.l3_literal_bits &&
           c.l2_literal_bits == cpp.l2_literal_bits &&
           c.l1_literal_bits == cpp.l1_literal_bits && c.total_bits == cpp.total_bits;
}

bool same_segment_metadata(const ddc_c_btv& c, const DDCBtv& cpp) {
    return c.state == static_cast<ddc_c_state>(cpp.state()) &&
           c.bit_count == cpp.bit_count() && c.l2_count == cpp.l2_count() &&
           c.l3_count == cpp.l3_count() && c.l4_count == cpp.l4_count() &&
           c.l1_fill_ones == cpp.l1_fill_ones() &&
           c.l2_fill_ones == cpp.l2_fill_ones() &&
           c.l3_fill_ones == cpp.l3_fill_ones() &&
           c.l1_lit_count == cpp.num_lits() &&
           c.l2_lit_count == cpp.l2_lit_count() &&
           c.l3_lit_count == cpp.l3_lit_count() &&
           ddc_c_btv_num_fills(&c) == cpp.num_fills() &&
           ddc_c_btv_is_all_zero(&c) == cpp.is_all_zero() &&
           ddc_c_btv_is_all_ones(&c) == cpp.is_all_ones();
}

void verify_equivalent(const ddc_c& c, const DDC& cpp, const std::string& label) {
    if (c.bit_count != cpp.bit_count() || c.segment_bits != cpp.segment_bits() ||
        c.segment_count != cpp.num_segments())
        fail(label + ": container metadata differs");
    if (ddc_c_popcount(&c) != cpp.popcount())
        fail(label + ": popcount differs");

    ddc_c_size_info c_size{};
    ddc_c_size_breakdown(&c, &c_size);
    if (!same_size_info(c_size, cpp.size_breakdown()))
        fail(label + ": size breakdown differs");

    std::vector<uint8_t> c_bits(c.bit_count);
    check_status(ddc_c_decompress(&c, c_bits.data(), c_bits.size()), label + " C decompression");
    const std::vector<bool> cpp_bits = cpp.decompress();
    for (size_t position = 0; position < cpp_bits.size(); ++position) {
        if ((c_bits[position] != 0) != cpp_bits[position])
            fail(label + ": logical bit differs at position " + std::to_string(position));
    }

    for (size_t segment = 0; segment < c.segment_count; ++segment) {
        if (!same_segment_metadata(c.segments[segment], cpp.segment(segment)))
            fail(label + ": segment metadata differs at segment " +
                 std::to_string(segment));
    }
}

void validate_operation(Operation operation, const Workload& workload, bool compress_results) {
    ddc_compress_results = compress_results;
    ddc_c_set_compress_results(compress_results ? 1 : 0);

    const DDC cpp_result = run_cpp_operation(operation, workload);
    CResult c_result;
    run_c_operation(operation, workload, &c_result.value);
    verify_equivalent(c_result.value, cpp_result,
                      workload.name + "/" + operation_name(operation) +
                      (compress_results ? "/compressed" : "/decompressed"));
}

double time_cpp_round(Operation operation, const Workload& workload, size_t iterations) {
    const Clock::time_point begin = Clock::now();
    for (size_t iteration = 0; iteration < iterations; ++iteration) {
        DDC result = run_cpp_operation(operation, workload);
        do_not_optimize(&result);
    }
    const Clock::time_point end = Clock::now();
    return std::chrono::duration<double, std::nano>(end - begin).count() /
           static_cast<double>(iterations);
}

double time_c_round(Operation operation, const Workload& workload, size_t iterations) {
    const Clock::time_point begin = Clock::now();
    for (size_t iteration = 0; iteration < iterations; ++iteration) {
        CResult result;
        run_c_operation(operation, workload, &result.value);
        do_not_optimize(&result.value);
    }
    const Clock::time_point end = Clock::now();
    return std::chrono::duration<double, std::nano>(end - begin).count() /
           static_cast<double>(iterations);
}

Summary summarize(std::vector<double> values) {
    if (values.empty()) fail("cannot summarize an empty timing series");
    std::sort(values.begin(), values.end());
    const double mean = std::accumulate(values.begin(), values.end(), 0.0) /
                        static_cast<double>(values.size());
    double squared_error = 0.0;
    for (const double value : values) {
        const double delta = value - mean;
        squared_error += delta * delta;
    }
    const size_t middle = values.size() / 2;
    const double median = (values.size() % 2 == 0)
                              ? (values[middle - 1] + values[middle]) / 2.0
                              : values[middle];
    return {values.front(), median, values.back(),
            std::sqrt(squared_error / static_cast<double>(values.size()))};
}

std::string cpu_model() {
#if defined(__linux__)
    std::ifstream cpuinfo("/proc/cpuinfo");
    std::string line;
    while (std::getline(cpuinfo, line)) {
        const std::string key = "model name";
        if (line.rfind(key, 0) == 0) {
            const size_t separator = line.find(':');
            if (separator != std::string::npos) return line.substr(separator + 2);
        }
    }
#endif
    return "unavailable";
}

std::string architecture() {
#if defined(__linux__)
    struct utsname system_info {};
    if (uname(&system_info) == 0) return system_info.machine;
#endif
    return "unavailable";
}

void print_environment(const Config& config) {
    std::cout << "# benchmark=standard_ddc_c_cpp_boolean\n";
    std::cout << "# scope=DDC_only;DDCN_excluded\n";
    std::cout << "# compiler=" << __VERSION__ << "\n";
    std::cout << "# target_compile_options=-O3"
              << (DDC_BENCH_AVX512_VBMI2 ? ";-mavx512f;-mavx512bw;-mavx512vbmi2" : "")
              << "\n";
    std::cout << "# cmake_avx512_vbmi2_gate=" << DDC_BENCH_AVX512_VBMI2 << "\n";
    std::cout << "# architecture=" << architecture() << "\n";
    std::cout << "# cpu_model=" << cpu_model() << "\n";
    std::cout << "# bit_count=" << kBitCount << ",segment_bits=" << kSegmentBits
              << ",l1_fill_ones=" << static_cast<unsigned>(kL1FillOnes) << "\n";
    std::cout << "# warmup=" << config.warmup << ",rounds=" << config.rounds
              << ",iterations_per_round=" << config.iterations << "\n";
    if (config.profile)
        std::cout << "# execution_order=single_implementation_profile_after_cpp_c_correctness_validation\n";
    else
        std::cout << "# execution_order=cpp_then_c_on_even_rounds;c_then_cpp_on_odd_rounds\n";
    std::cout << "implementation,operation,workload,result_compression,bit_count,segment_bits,"
              << "a_density,b_density,warmup,rounds,iterations,unit,min,median,max,stddev\n";
}

void print_row(const char* implementation, Operation operation, const Workload& workload,
               bool compress_results, const Config& config, const Summary& summary) {
    const double a_density = static_cast<double>(workload.a_set_bits) /
                             static_cast<double>(kBitCount);
    const double b_density = static_cast<double>(workload.b_set_bits) /
                             static_cast<double>(kBitCount);
    std::cout << std::fixed << std::setprecision(6) << implementation << ','
              << operation_name(operation) << ',' << workload.name << ','
              << (compress_results ? "compressed" : "decompressed") << ','
              << kBitCount << ',' << kSegmentBits << ',' << a_density << ',' << b_density
              << ',' << config.warmup << ',' << config.rounds << ',' << config.iterations
              << ",ns_per_operation," << summary.min_ns << ',' << summary.median_ns << ','
              << summary.max_ns << ',' << summary.stddev_ns << '\n';
}

Operation parse_operation(const std::string& value) {
    if (value == "and") return Operation::And;
    if (value == "or") return Operation::Or;
    if (value == "xor") return Operation::Xor;
    if (value == "not") return Operation::Not;
    fail("unsupported profile operation: " + value);
}

Implementation parse_implementation(const std::string& value) {
    if (value == "cpp") return Implementation::Cpp;
    if (value == "c") return Implementation::C;
    fail("unsupported profile implementation: " + value);
}

bool parse_result_compression(const std::string& value) {
    if (value == "decompressed") return false;
    if (value == "compressed") return true;
    fail("unsupported profile result compression: " + value);
}

size_t parse_positive_size(const std::string& value, const char* option) {
    size_t parsed = 0;
    try {
        size_t consumed = 0;
        parsed = static_cast<size_t>(std::stoull(value, &consumed));
        if (consumed != value.size() || parsed == 0)
            fail(std::string(option) + " must be a positive integer");
    } catch (const std::exception&) {
        fail(std::string(option) + " must be a positive integer");
    }
    return parsed;
}

Config parse_config(int argc, char** argv) {
    if (argc == 1) return {};
    if (argc == 2 && std::string(argv[1]) == "--quick")
        return {2, 5, 10};
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage:\n"
                  << "  ddc_c_cpp_boolean_bench [--quick]\n"
                  << "  ddc_c_cpp_boolean_bench --profile <c|cpp> <and|or|xor|not> "
                     "<all_zero|all_one|sparse|balanced|dense> "
                     "<decompressed|compressed> <warmup> <rounds> <iterations>\n"
                  << "  --quick    Run the small smoke matrix (2 warmups, 5 rounds, 10 iterations).\n"
                  << "  --profile  Validate the selected C/C++ pair once, then time only the requested "
                     "implementation. Intended for perf profiling.\n";
        std::exit(EXIT_SUCCESS);
    }
    if (argc == 9 && std::string(argv[1]) == "--profile") {
        Config config;
        config.profile = true;
        config.profile_implementation = parse_implementation(argv[2]);
        config.profile_operation = parse_operation(argv[3]);
        config.profile_workload = argv[4];
        config.profile_compress_results = parse_result_compression(argv[5]);
        config.warmup = parse_positive_size(argv[6], "warmup");
        config.rounds = parse_positive_size(argv[7], "rounds");
        config.iterations = parse_positive_size(argv[8], "iterations");
        return config;
    }
    fail("usage: ddc_c_cpp_boolean_bench [--quick] or --profile <c|cpp> <and|or|xor|not> <workload> <decompressed|compressed> <warmup> <rounds> <iterations>");
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Config config = parse_config(argc, argv);
        ResultPolicyGuard restore_policy;
        std::array<Workload, 5> workloads{};
        make_workload(workloads[0], "all_zero", 0, 0, true, false);
        make_workload(workloads[1], "all_one", 0, 0, false, true);
        make_workload(workloads[2], "sparse", 80, 170, false, false);
        make_workload(workloads[3], "balanced", 5000, 5000, false, false);
        make_workload(workloads[4], "dense", 9200, 8500, false, false);

        print_environment(config);
        const std::array<Operation, 4> operations{{
            Operation::And, Operation::Or, Operation::Xor, Operation::Not
        }};
        if (config.profile) {
            const auto workload = std::find_if(workloads.begin(), workloads.end(),
                                               [&config](const Workload& value) {
                                                   return value.name == config.profile_workload;
                                               });
            if (workload == workloads.end())
                fail("unsupported profile workload: " + config.profile_workload);
            ddc_compress_results = config.profile_compress_results;
            ddc_c_set_compress_results(config.profile_compress_results ? 1 : 0);
            validate_operation(config.profile_operation, *workload,
                               config.profile_compress_results);
            for (size_t warmup = 0; warmup < config.warmup; ++warmup) {
                if (config.profile_implementation == Implementation::Cpp)
                    (void)time_cpp_round(config.profile_operation, *workload, 1);
                else
                    (void)time_c_round(config.profile_operation, *workload, 1);
            }
            std::vector<double> times;
            times.reserve(config.rounds);
            for (size_t round = 0; round < config.rounds; ++round) {
                if (config.profile_implementation == Implementation::Cpp)
                    times.push_back(time_cpp_round(config.profile_operation, *workload,
                                                   config.iterations));
                else
                    times.push_back(time_c_round(config.profile_operation, *workload,
                                                 config.iterations));
            }
            print_row(config.profile_implementation == Implementation::Cpp ? "cpp" : "c",
                      config.profile_operation, *workload, config.profile_compress_results,
                      config, summarize(std::move(times)));
            return EXIT_SUCCESS;
        }
        for (const Workload& workload : workloads) {
            for (const bool compress_results : {false, true}) {
                ddc_compress_results = compress_results;
                ddc_c_set_compress_results(compress_results ? 1 : 0);
                for (const Operation operation : operations) {
                    validate_operation(operation, workload, compress_results);
                    for (size_t warmup = 0; warmup < config.warmup; ++warmup) {
                        (void)time_cpp_round(operation, workload, 1);
                        (void)time_c_round(operation, workload, 1);
                    }

                    std::vector<double> cpp_times;
                    std::vector<double> c_times;
                    cpp_times.reserve(config.rounds);
                    c_times.reserve(config.rounds);
                    for (size_t round = 0; round < config.rounds; ++round) {
                        if ((round % 2) == 0) {
                            cpp_times.push_back(time_cpp_round(operation, workload,
                                                               config.iterations));
                            c_times.push_back(time_c_round(operation, workload,
                                                           config.iterations));
                        } else {
                            c_times.push_back(time_c_round(operation, workload,
                                                           config.iterations));
                            cpp_times.push_back(time_cpp_round(operation, workload,
                                                               config.iterations));
                        }
                    }
                    print_row("cpp", operation, workload, compress_results, config,
                              summarize(std::move(cpp_times)));
                    print_row("c", operation, workload, compress_results, config,
                              summarize(std::move(c_times)));
                }
            }
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "ddc_c_cpp_boolean_bench: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
