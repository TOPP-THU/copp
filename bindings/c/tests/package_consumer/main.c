#include <stdio.h>

#include "copp/copp.h"

int main(void) {
    const char *version = copp_version();
    const char *ok = copp_status_message(COPP_STATUS_OK);

    if (version == NULL || ok == NULL) {
        return 1;
    }

    printf("copp %s: %s\n", version, ok);
    return 0;
}
