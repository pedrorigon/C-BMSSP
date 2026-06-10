#pragma once

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace test {

inline void require(const bool condition, const std::string_view message) {
  if (!condition) {
    throw std::runtime_error(std::string(message));
  }
}

inline void require_near(const double expected, const double actual, const double tolerance,
                         const std::string_view message) {
  if (std::abs(expected - actual) > tolerance) {
    throw std::runtime_error(std::string(message) + ": expected=" + std::to_string(expected) +
                             " actual=" + std::to_string(actual));
  }
}

template <typename Exception, typename Callable>
void require_throws(Callable&& callable, const std::string_view message) {
  try {
    callable();
  } catch (const Exception&) {
    return;
  }
  throw std::runtime_error(std::string(message));
}

template <typename Callable> int run(Callable&& callable) {
  try {
    callable();
    std::cout << "test passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "test failure: " << error.what() << '\n';
    return 1;
  }
}

} // namespace test
