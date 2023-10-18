#ifndef UTILS_RNG_H
#define UTILS_RNG_H

#include <algorithm>
#include <cassert>
#include <random>
#include <vector>
#include <numeric>

namespace utils {
class RandomNumberGenerator {
    // Mersenne Twister random number generator.
    std::mt19937 rng;

public:
    RandomNumberGenerator(); // Seed with a value depending on time and process ID.
    explicit RandomNumberGenerator(int seed);
    explicit RandomNumberGenerator(std::mt19937 &rng);
    RandomNumberGenerator(const RandomNumberGenerator &) = delete;
    RandomNumberGenerator &operator=(const RandomNumberGenerator &) = delete;
    ~RandomNumberGenerator();

    void seed(int seed);

    // Return random double in [0..1).
    double random() {
        std::uniform_real_distribution<double> distribution(0.0, 1.0);
        return distribution(rng);
    }

    // Return random integer in [0..bound).
    int random(int bound) {
        assert(bound > 0);
        std::uniform_int_distribution<int> distribution(0, bound - 1);
        return distribution(rng);
    }

    double operator()() {
        std::uniform_real_distribution<double> distribution(0.0, 1.0);
        return distribution(rng);
    }

    // Return random integer in [0..bound).
    int operator()(int bound) {
        assert(bound > 0);
        std::uniform_int_distribution<int> distribution(0, bound - 1);
        return distribution(rng);
    }

    template<typename T>
    typename std::vector<T>::const_iterator choose(const std::vector<T> &vec) {
        return vec.begin() + random(vec.size());
    }

    template<typename T>
    typename std::vector<T>::iterator choose(std::vector<T> &vec) {
        return vec.begin() + random(vec.size());
    }

    template<typename T>
    void shuffle(std::vector<T> &vec) {
        std::shuffle(vec.begin(), vec.end(), rng);
    }

    /**
     * @brief Compute a vector containing a subset of the indices of vec, i.e., a random subset of 0, 1, ..., vec.size()-1
     * with min(max_size, vec.size()) elements.
     * @note the returned vector is sorted
     */
    template<typename T>
    std::vector<int> select_random_positions(const std::vector<T> &vec, size_t max_size) {
        std::vector<int> vec_indices(vec.size());
        std::iota(vec_indices.begin(), vec_indices.end(), 0);
        // vec indices contains numbers 0, ..., vec.size() - 1

        if (vec.size() <= max_size) {
            return vec_indices;
        }

        // select a sub vector of indices, keeping order intact
        shuffle(vec_indices);
        vec_indices.resize(max_size);
        std::sort(vec_indices.begin(), vec_indices.end());
        return vec_indices;
    }
};
}

#endif
