#ifndef SENT_TEST_H
#define SENT_TEST_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

extern int test_total_;
extern int test_failed_;
extern int test_assert_failed_;

#define TEST_RUN(fn)                                                       \
    do {                                                                   \
        test_total_++;                                                     \
        test_assert_failed_ = 0;                                           \
        printf("[RUN ] %s\n", #fn);                                        \
        fn();                                                              \
        if (test_assert_failed_) {                                         \
            test_failed_++;                                                \
            printf("[FAIL] %s\n", #fn);                                    \
        } else {                                                           \
            printf("[ OK ] %s\n", #fn);                                    \
        }                                                                  \
    } while (0)

#define TEST_FAIL_(msg)                                                    \
    do {                                                                   \
        printf("       %s:%d: %s\n", __FILE__, __LINE__, msg);             \
        test_assert_failed_ = 1;                                           \
        return;                                                            \
    } while (0)

#define TEST_ASSERT(cond)                                                  \
    do {                                                                   \
        if (!(cond)) {                                                     \
            TEST_FAIL_("assertion failed: " #cond);                        \
        }                                                                  \
    } while (0)

#define TEST_ASSERT_EQ(a, b)                                               \
    do {                                                                   \
        long long ta_ = (long long)(a);                                    \
        long long tb_ = (long long)(b);                                    \
        if (ta_ != tb_) {                                                  \
            printf("       %s:%d: expected %s == %s (got %lld vs %lld)\n", \
                   __FILE__, __LINE__, #a, #b, ta_, tb_);                  \
            test_assert_failed_ = 1;                                       \
            return;                                                        \
        }                                                                  \
    } while (0)

#define TEST_ASSERT_TRUE(cond)  TEST_ASSERT(cond)
#define TEST_ASSERT_FALSE(cond) TEST_ASSERT(!(cond))

#define TEST_GLOBALS()                                                     \
    int test_total_ = 0;                                                   \
    int test_failed_ = 0;                                                  \
    int test_assert_failed_ = 0

#define TEST_REPORT_AND_EXIT()                                             \
    do {                                                                   \
        printf("\n=== %d/%d passed ===\n",                                 \
               test_total_ - test_failed_, test_total_);                   \
        return test_failed_ == 0 ? 0 : 1;                                  \
    } while (0)

#ifdef __cplusplus
}
#endif

#endif /* SENT_TEST_H */
