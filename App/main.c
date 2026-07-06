#include "app_entry.h"

int main(void)
{
    app_init();

    while (1) {
        app_run_once();
    }
}
