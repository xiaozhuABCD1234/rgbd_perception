#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "test_support.hpp"

namespace rgbd {
namespace test {

std::vector<TestCase>& Registry() {
  static std::vector<TestCase> registry;
  return registry;
}

void Check(bool condition, const char* expression, const char* file, int line) {
  if (!condition) {
    throw std::runtime_error(std::string(file) + ":" + std::to_string(line) +
                             ": check failed: " + expression);
  }
}

void CheckNear(float actual, float expected, float tolerance,
               const char* expression, const char* file, int line) {
  if (std::fabs(actual - expected) > tolerance) {
    throw std::runtime_error(std::string(file) + ":" + std::to_string(line) +
                             ": near check failed: " + expression +
                             " actual=" + std::to_string(actual) +
                             " expected=" + std::to_string(expected));
  }
}

Registrar::Registrar(std::string name, TestFunction function) {
  Registry().push_back({std::move(name), function});
}

}
}

int main() {
  std::size_t failures = 0;
  const auto& tests = rgbd::test::Registry();
  for (const auto& test : tests) {
    try {
      test.function();
      std::cout << "[PASS] " << test.name << '\n';
    } catch (const std::exception& error) {
      ++failures;
      std::cerr << "[FAIL] " << test.name << ": " << error.what() << '\n';
    }
  }
  std::cout << tests.size() - failures << "/" << tests.size()
            << " tests passed\n";
  return failures == 0 ? 0 : 1;
}
