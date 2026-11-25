#include "test.h"

TEST_GLOBALS();

void run_sent_crc_tests(void);
void run_sent_protocol_tests(void);
void run_sent_encoder_tests(void);
void run_sent_decoder_tests(void);
void run_sent_decoder_equiv_tests(void);
void run_sent_slow_channel_tests(void);
void run_mode_manager_tests(void);

int main(void) {
    run_sent_crc_tests();
    run_sent_protocol_tests();
    run_sent_encoder_tests();
    run_sent_decoder_tests();
    run_sent_decoder_equiv_tests();
    run_sent_slow_channel_tests();
    run_mode_manager_tests();
    TEST_REPORT_AND_EXIT();
}
