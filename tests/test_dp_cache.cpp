#include "probeoptimizer/dp_cache.h" // Adjust path as necessary
#include <cassert>
#include <string>
#include <iostream>
#include <vector> // For custom key test

// Custom struct for testing with DPCache
struct CustomKey {
    int id;
    std::string name;

    bool operator==(const CustomKey& other) const {
        return id == other.id && name == other.name;
    }
};

// Hash specialization for CustomKey
namespace std {
    template <>
    struct hash<CustomKey> {
        size_t operator()(const CustomKey& k) const noexcept {
            size_t h1 = std::hash<int>()(k.id);
            size_t h2 = std::hash<std::string>()(k.name);
            return h1 ^ (h2 << 1); // Simple combine
        }
    };
}

void test_int_key_string_value() {
    std::cout << "Running test_int_key_string_value..." << std::endl;
    ProbeOptimizer::DPCache<int, std::string> cache;
    std::string value;

    // Test store and lookup
    cache.store(1, "one");
    assert(cache.lookup(1, value) && value == "one");
    std::cout << "  Store and lookup: PASSED" << std::endl;

    // Test lookup for non-existent key
    assert(!cache.lookup(2, value));
    std::cout << "  Lookup non-existent: PASSED" << std::endl;

    // Test store overwriting
    cache.store(1, "new_one");
    assert(cache.lookup(1, value) && value == "new_one");
    std::cout << "  Store overwrite: PASSED" << std::endl;

    // Test contains
    assert(cache.contains(1));
    assert(!cache.contains(2));
    std::cout << "  Contains: PASSED" << std::endl;

    // Test size
    assert(cache.size() == 1);
    cache.store(2, "two");
    assert(cache.size() == 2);
    std::cout << "  Size: PASSED" << std::endl;

    // Test erase
    assert(cache.erase(1));
    assert(!cache.contains(1));
    assert(cache.size() == 1);
    assert(!cache.erase(1)); // Erase non-existent
    std::cout << "  Erase: PASSED" << std::endl;

    // Test clear
    cache.clear();
    assert(cache.size() == 0);
    assert(!cache.contains(2));
    std::cout << "  Clear: PASSED" << std::endl;
}

void test_custom_key_double_value() {
    std::cout << "Running test_custom_key_double_value..." << std::endl;
    ProbeOptimizer::DPCache<CustomKey, double> cache;
    CustomKey k1 = {1, "alpha"};
    CustomKey k2 = {2, "beta"};
    CustomKey k3 = {1, "alpha"}; // Same as k1
    double value;

    cache.store(k1, 10.5);
    assert(cache.lookup(k1, value) && value == 10.5);
    assert(cache.lookup(k3, value) && value == 10.5); // Test with equivalent key
    std::cout << "  Store and lookup custom key: PASSED" << std::endl;

    assert(!cache.lookup(k2, value));
    std::cout << "  Lookup non-existent custom key: PASSED" << std::endl;

    cache.store(k2, 20.0);
    assert(cache.contains(k2));
    assert(cache.size() == 2);
    std::cout << "  Store second custom key: PASSED" << std::endl;
}


int main() {
    std::cout << "--- Running DPCache Tests ---" << std::endl;
    test_int_key_string_value();
    test_custom_key_double_value();
    std::cout << "--- DPCache Tests Completed ---" << std::endl;
    return 0;
}
