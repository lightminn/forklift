/* Minimal host test assertions: each failed check prints and counts; main returns the count. */
#ifndef FW_TEST_H
#define FW_TEST_H

#include <math.h>
#include <stdio.h>

static int fw_test_failures;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond);               \
            fw_test_failures++;                                                                    \
        }                                                                                          \
    } while (0)

#define CHECK_EQ_U(a, b)                                                                           \
    do {                                                                                           \
        unsigned long long fw_a_ = (unsigned long long)(a), fw_b_ = (unsigned long long)(b);       \
        if (fw_a_ != fw_b_) {                                                                      \
            fprintf(stderr, "%s:%d: %s == %s failed: 0x%llx != 0x%llx\n", __FILE__, __LINE__, #a,  \
                    #b, fw_a_, fw_b_);                                                             \
            fw_test_failures++;                                                                    \
        }                                                                                          \
    } while (0)

#define CHECK_EQ_I(a, b)                                                                           \
    do {                                                                                           \
        long long fw_a_ = (long long)(a), fw_b_ = (long long)(b);                                  \
        if (fw_a_ != fw_b_) {                                                                      \
            fprintf(stderr, "%s:%d: %s == %s failed: %lld != %lld\n", __FILE__, __LINE__, #a, #b,  \
                    fw_a_, fw_b_);                                                                 \
            fw_test_failures++;                                                                    \
        }                                                                                          \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                      \
    do {                                                                                           \
        double fw_a_ = (double)(a), fw_b_ = (double)(b);                                           \
        if (!(fabs(fw_a_ - fw_b_) <= (double)(tol))) {                                             \
            fprintf(stderr, "%s:%d: %s ~= %s failed: %.9g vs %.9g\n", __FILE__, __LINE__, #a, #b,  \
                    fw_a_, fw_b_);                                                                 \
            fw_test_failures++;                                                                    \
        }                                                                                          \
    } while (0)

#define FW_TEST_RESULT()                                                                           \
    (fw_test_failures == 0 ? 0 : (fprintf(stderr, "%d check(s) failed\n", fw_test_failures), 1))

#endif
