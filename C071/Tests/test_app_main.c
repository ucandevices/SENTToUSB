#include "test.h"

TEST_GLOBALS();

void run_bridge_multi_tests(void);
void run_core_tests(void);

int main(void) {
    run_bridge_multi_tests();
    run_core_tests();
    TEST_REPORT_AND_EXIT();
}
