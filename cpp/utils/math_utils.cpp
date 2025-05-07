#include <vector>
#include <array>

std::array<float, 3> normalise(std::array<float, 3> vec) {
    float norm = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]) + 1e-15;
    return {vec[0] / norm, vec[1] / norm, vec[2] / norm};
}