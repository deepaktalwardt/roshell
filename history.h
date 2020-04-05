#ifndef HISTORY_H
#define HISTORY_H

#define MAX_CMD_LEN  128
#define HISTORY_COUNT 20

int history(char *hist[], int current);
int clear_history(char *hist[]);

#endif //HISTORY_H
