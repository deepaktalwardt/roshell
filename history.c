#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "history.h"

int history(char *hist[], int current)
{
        int i = current;
        int hist_num = 1;

        do {
                if (hist[i]) {
                        printf("%4d  %s\n", hist_num, hist[i]);
                        hist_num++;
                }

                i = (i + 1) % HISTORY_COUNT;

        } while (i != current);

        return 0;
}

int clear_history(char *hist[])
{
        int i;

        for (i = 0; i < HISTORY_COUNT; i++) {
                free(hist[i]);
                hist[i] = NULL;
        }

        return 0;
}
