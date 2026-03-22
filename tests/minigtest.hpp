#pragma once

#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace minigtest {

struct TestCase {
    std::string name;
    std::function<void()> run;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

inline void registerTest(std::string name, std::function<void()> run) {
    registry().push_back(TestCase{std::move(name), std::move(run)});
}

inline void fail(const char* file, int line, const std::string& message) {
    std::ostringstream out;
    out << file << ':' << line << ": " << message;
    throw std::runtime_error(out.str());
}

inline int runAllTests() {
    int failures = 0;
    for (const TestCase& test : registry()) {
        try {
            test.run();
            std::cout << "[  PASSED  ] " << test.name << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "[  FAILED  ] " << test.name << " - " << error.what() << '\n';
        }
    }
    std::cout << registry().size() - static_cast<std::size_t>(failures)
              << " tests passed, " << failures << " failed\n";
    return failures == 0 ? 0 : 1;
}

}  // namespace minigtest

#define TEST(SUITE, NAME)                                                              \
    void SUITE##_##NAME##_impl();                                                      \
    namespace {                                                                        \
    const bool SUITE##_##NAME##_registered = [] {                                      \
        minigtest::registerTest(#SUITE "." #NAME, SUITE##_##NAME##_impl);             \
        return true;                                                                   \
    }();                                                                               \
    }                                                                                  \
    void SUITE##_##NAME##_impl()

#define EXPECT_TRUE(EXPR)                                                              \
    do {                                                                               \
        if (!(EXPR)) {                                                                 \
            minigtest::fail(__FILE__, __LINE__, "EXPECT_TRUE failed: " #EXPR);        \
        }                                                                              \
    } while (false)

#define EXPECT_FALSE(EXPR) EXPECT_TRUE(!(EXPR))

#define EXPECT_EQ(A, B)                                                                \
    do {                                                                               \
        const auto a_value = (A);                                                      \
        const auto b_value = (B);                                                      \
        if (!(a_value == b_value)) {                                                   \
            std::ostringstream msg;                                                    \
            msg << "EXPECT_EQ failed: " #A " == " #B << " (" << a_value              \
                << " vs " << b_value << ")";                                         \
            minigtest::fail(__FILE__, __LINE__, msg.str());                           \
        }                                                                              \
    } while (false)

#define EXPECT_NEAR(A, B, EPS)                                                         \
    do {                                                                               \
        const double a_value = static_cast<double>(A);                                 \
        const double b_value = static_cast<double>(B);                                 \
        const double eps_value = static_cast<double>(EPS);                             \
        if (std::fabs(a_value - b_value) > eps_value) {                                \
            std::ostringstream msg;                                                    \
            msg << "EXPECT_NEAR failed: " #A " ~= " #B << " (" << a_value            \
                << " vs " << b_value << ", eps " << eps_value << ")";               \
            minigtest::fail(__FILE__, __LINE__, msg.str());                           \
        }                                                                              \
    } while (false)

#define EXPECT_LT(A, B)                                                                \
    do {                                                                               \
        if (!((A) < (B))) {                                                            \
            minigtest::fail(__FILE__, __LINE__, "EXPECT_LT failed: " #A " < " #B);   \
        }                                                                              \
    } while (false)

#define ASSERT_EQ EXPECT_EQ
