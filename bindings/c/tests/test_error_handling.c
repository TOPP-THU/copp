/**
 * @file test_error_handling.c
 * @brief Smoke test for detailed thread-local C ABI error messages.
 */

#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "copp/copp.h"

static int expect_ok(enum CoppStatus status, const char *call)
{
    if (status != COPP_STATUS_OK)
    {
        fprintf(stderr, "%s failed: %s\n", call, copp_status_message(status));
        return 1;
    }
    return 0;
}

int main(void)
{
    struct CoppRobot *robot = NULL;
    size_t robot_len = 0;
    size_t message_len = 0;
    char buffer[16];

    copp_clear_last_error();
    assert(copp_last_error_code() == COPP_STATUS_OK);
    assert(copp_last_error_message_len() == 0);
    assert(copp_last_error_message()[0] == '\0');

    enum CoppStatus status = copp_robot_len(NULL, &robot_len);
    assert(status == COPP_STATUS_NULL_POINTER);
    assert(copp_last_error_code() == COPP_STATUS_NULL_POINTER);
    assert(copp_last_error_message_len() > 0);
    assert(strstr(copp_last_error_message(), "null pointer") != NULL);

    status = copp_last_error_message_copy(buffer, sizeof(buffer), &message_len);
    if (expect_ok(status, "copp_last_error_message_copy"))
    {
        return 1;
    }
    assert(message_len >= strlen(buffer));
    assert(buffer[sizeof(buffer) - 1] == '\0' || strlen(buffer) < sizeof(buffer));
    assert(copp_last_error_code() == COPP_STATUS_NULL_POINTER);

    const unsigned char invalid_utf8[] = {'c', 'a', 'l', 'l', 'b', 'a', 'c', 'k', ':', ' ', 0xff};
    status = copp_set_last_error_message_n(
        COPP_STATUS_INVALID_ARGUMENT,
        (const char *)invalid_utf8,
        sizeof(invalid_utf8));
    if (expect_ok(status, "copp_set_last_error_message_n"))
    {
        return 1;
    }
    assert(copp_last_error_code() == COPP_STATUS_INVALID_ARGUMENT);
    assert(copp_last_error_message_len() > 0);

    status = copp_set_last_error_message(
        COPP_STATUS_ROBOT_DYNAMICS_ERROR,
        "callback detail from C");
    if (expect_ok(status, "copp_set_last_error_message"))
    {
        return 1;
    }
    assert(copp_last_error_code() == COPP_STATUS_ROBOT_DYNAMICS_ERROR);
    assert(strstr(copp_last_error_message(), "callback detail") != NULL);

    status = copp_robot_create(1, 2, &robot);
    if (expect_ok(status, "copp_robot_create"))
    {
        return 1;
    }
    assert(copp_last_error_code() == COPP_STATUS_OK);
    assert(copp_last_error_message_len() == 0);

    copp_robot_free(robot);
    assert(copp_last_error_code() == COPP_STATUS_OK);
    return 0;
}
