#ifndef TESTS_TEST_SUPPORT_HPP_
#define TESTS_TEST_SUPPORT_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace rgbd {
namespace test {

using TestFunction = void (*)();

struct TestCase {
  std::string name;
  TestFunction function;
};

std::vector<TestCase>& Registry();
void Check(bool condition, const char* expression, const char* file, int line);
void CheckNear(float actual, float expected, float tolerance,
               const char* expression, const char* file, int line);

class Registrar {
 public:
  Registrar(std::string name, TestFunction function);
};

}
}

#define RGBD_TEST(test_name)                                                   \
  void test_name();                                                            \
  const ::rgbd::test::Registrar registrar_##test_name(#test_name, &test_name); \
  void test_name()

#define RGBD_CHECK(expression) \
  ::rgbd::test::Check((expression), #expression, __FILE__, __LINE__)

#define RGBD_CHECK_NEAR(actual, expected, tolerance)         \
  ::rgbd::test::CheckNear((actual), (expected), (tolerance), \
                          #actual " ~= " #expected, __FILE__, __LINE__)

#define RGBD_CHECK_THROW(statement, exception_type)                   \
  do {                                                                \
    bool threw = false;                                               \
    try {                                                             \
      statement;                                                      \
    } catch (const exception_type&) {                                 \
      threw = true;                                                   \
    } catch (...) {                                                   \
    }                                                                 \
    ::rgbd::test::Check(threw, #statement " throws " #exception_type, \
                        __FILE__, __LINE__);                          \
  } while (false)

#endif
