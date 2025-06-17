#ifndef DP_CACHE_H
#define DP_CACHE_H

#include <unordered_map>
#include <memory> // For std::shared_ptr if ValueType is large or complex

// Forward declaration for KeyType's hash specializations if they are custom structs/classes
// namespace std {
// template <> struct hash<MyKeyType>;
// }

namespace ProbeOptimizer {

/**
 * @brief A generic cache for dynamic programming (memoization).
 *
 * This class provides a simple key-value store to cache the results of
 * computationally expensive subproblems.
 *
 * @tparam KeyType The type used to identify a subproblem (e.g., a tuple of parameters, a custom struct).
 *                 Must be hashable (i.e., std::hash<KeyType> must be defined) and
 *                 equality comparable (i.e., operator== must be defined).
 * @tparam ValueType The type of the computed result for a subproblem.
 */
template <typename KeyType, typename ValueType>
class DPCache {
public:
    DPCache() = default;

    /**
     * @brief Looks up a solution for a given subproblem key.
     *
     * @param key The key identifying the subproblem.
     * @param out_value Reference to store the retrieved value if found.
     * @return true if a solution was found in the cache, false otherwise.
     */
    bool lookup(const KeyType& key, ValueType& out_value) const {
        auto it = cache_.find(key);
        if (it != cache_.end()) {
            out_value = it->second;
            return true;
        }
        return false;
    }

    /**
     * @brief Stores the solution for a given subproblem key.
     * If the key already exists, its value is overwritten.
     *
     * @param key The key identifying the subproblem.
     * @param value The computed solution/value to store.
     */
    void store(const KeyType& key, const ValueType& value) {
        cache_[key] = value;
    }

    /**
     * @brief Stores the solution for a given subproblem key using move semantics.
     * If the key already exists, its value is overwritten.
     *
     * @param key The key identifying the subproblem.
     * @param value The computed solution/value to store (will be moved from).
     */
    void store(const KeyType& key, ValueType&& value) {
        cache_[key] = std::move(value);
    }

    /**
     * @brief Clears all entries from the cache.
     */
    void clear() {
        cache_.clear();
    }

    /**
     * @brief Checks if the cache contains an entry for the given key.
     * @param key The key to check.
     * @return true if the key is found, false otherwise.
     */
    bool contains(const KeyType& key) const {
        return cache_.count(key) > 0;
    }

    /**
     * @brief Returns the number of entries currently in the cache.
     * @return The size of the cache.
     */
    size_t size() const {
        return cache_.size();
    }

    /**
     * @brief Removes a specific entry from the cache.
     * @param key The key of the entry to remove.
     * @return true if an element was removed, false otherwise.
     */
    bool erase(const KeyType& key) {
        return cache_.erase(key) > 0;
    }

private:
    std::unordered_map<KeyType, ValueType> cache_;
    // Note: For KeyType to work with std::unordered_map, it needs:
    // 1. A specialization of std::hash<KeyType>.
    // 2. An equality operator (operator==).
    // For simple types (int, string, std::tuple of hashable types), these are often provided.
    // For custom structs, users will need to provide them.
};

} // namespace ProbeOptimizer

#endif // DP_CACHE_H
