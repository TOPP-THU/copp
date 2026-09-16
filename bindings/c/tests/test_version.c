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

    assert(strcmp(COPP_VERSION_STRING, copp_version()) == 0);
    char numeric_version[64];
    snprintf(numeric_version,
             sizeof numeric_version,
             "%d.%d.%d",
             COPP_VERSION_MAJOR,
             COPP_VERSION_MINOR,
             COPP_VERSION_PATCH);
    assert(strncmp(COPP_VERSION_STRING, numeric_version, strlen(numeric_version)) == 0);

    const char *message = copp_status_message(COPP_STATUS_OK);
    assert(message != NULL);
    assert(strcmp(message, "ok") == 0);

    return 0;
}
