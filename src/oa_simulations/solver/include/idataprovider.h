#pragma once

#include <vector>
#include <cmath>

#include "common_data_types.h"

namespace Solver
{
constexpr size_t kMaxNumberOfSamples{360};

constexpr double DegreesToRadians(double degrees) {
    return degrees * (M_PI / 180);
}

constexpr double RadiansToDegrees(double radians) {
    return radians * (180.0 / M_PI);
}

constexpr long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Function to reverse part of array
template <typename T>
void reverse(std::vector<T>& arr, int start, int end) {
    while (start < end) {
        std::swap(arr[start], arr[end]);
        start++;
        end--;
    }
}

// Optimized right shift using reversal algorithm
template <typename T>
void circularRightShift(std::vector<T>& arr, int k) {
    int n = arr.size();
    k = k % n;  // Handle cases where k > n
    if (k == 0) return;  // No shift needed

    reverse(arr, 0, n - 1);     // Step 1: Reverse whole array
    reverse(arr, 0, k - 1);     // Step 2: Reverse first k elements
    reverse(arr, k, n - 1);     // Step 3: Reverse remaining elements
}

class IDataProvider
{
public:
/*
*   @brief Fetch data from some source.
*   This should be blocking call.
*/
    virtual std::vector<DistanceSensorData> getSample() = 0;
};

}//  namespace Solver

