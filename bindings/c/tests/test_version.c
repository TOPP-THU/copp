/**
 * @file test_version.c
 * @brief Smoke test for the minimal COPP C ABI.
 *
 * This test verifies that a C translation unit can include the generated
 * public header, link against the native COPP library, call a basic
 * exported function, and resolve a status-code message.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "copp/copp.h"

int main(void) {
    const char *version = copp_version();
    assert(version != NULL);
    assert(strlen(version) > 0);
    printf("copp version: %s\n", version);

    const char *message = copp_status_message(COPP_STATUS_OK);
    assert(message != NULL);
    assert(strcmp(message, "ok") == 0);

    return 0;
}
