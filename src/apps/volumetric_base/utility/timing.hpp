#pragma once

#include <chrono>

NAMESPACE_BEGIN(timing)

inline auto measureTime(const std::string& label, const std::chrono::high_resolution_clock::time_point& start)  {
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    std::cout << label << " execution time: " << duration.count() << " ms" << std::endl;
    return end;  // return the end time for further timing
}

NAMESPACE_END(timing)
