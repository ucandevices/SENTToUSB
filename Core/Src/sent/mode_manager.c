#include "sent/mode_manager.h"

#include "sent/sent_assert.h"
#include <string.h>

/* Initialize mode manager to STOPPED state and zero all statistics counters.
 * @param manager  pointer to mode manager instance */
void sent_mode_manager_init(sent_mode_manager_t* manager) {
    SENT_ASSERT(manager != NULL);
    manager->mode = SENT_MODE_STOPPED;
    memset(&manager->stats, 0, sizeof(manager->stats));
}

/* Transition mode manager to RX (receive) mode.
 * @param manager  pointer to mode manager instance */
void sent_mode_manager_start_rx(sent_mode_manager_t* manager) {
    SENT_ASSERT(manager != NULL);
    manager->mode = SENT_MODE_RX;
}

/* Transition mode manager to TX (transmit) mode.
 * @param manager  pointer to mode manager instance */
void sent_mode_manager_start_tx(sent_mode_manager_t* manager) {
    SENT_ASSERT(manager != NULL);
    manager->mode = SENT_MODE_TX;
}

/* Transition mode manager to STOPPED mode.
 * @param manager  pointer to mode manager instance */
void sent_mode_manager_stop(sent_mode_manager_t* manager) {
    SENT_ASSERT(manager != NULL);
    manager->mode = SENT_MODE_STOPPED;
}
