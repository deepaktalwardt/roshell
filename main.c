#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <sys/wait.h>
#include "const.h"
#include "variable.h"
#include "command.h"
#include "history.h"

int main(int argc, char *argv[], char* envp[]) {
    // argc - argument count
    // argv - argument vector
    // envp - environment pointer
    // TODO: Should roshell inherit envp from parent shells?

    char hostname[_SC_HOST_NAME_MAX];
    char username[_SC_LOGIN_NAME_MAX];
    gethostname(hostname,_SC_HOST_NAME_MAX); // system call to get the hostname
    getlogin_r(username,_SC_LOGIN_NAME_MAX); // system call to get the username

    char *hist[HISTORY_COUNT];
    int i, current = 0;

    for (i = 0; i < HISTORY_COUNT; i++)
       hist[i] = NULL;

    // print out all the environment variables
    for(int i = 0; envp[i]; ++i)
    {
        printf("environment variable %d: %s \n", i, envp[i]);
        if (strchr(envp[i], '=') != NULL)
        {
            // add to variable list
            addVariable(envp[i]);
        }
    }

    while (1)
    {
        char input[MAX_COMM_SIZE + 1] = { 0x0 };
        printf("%s@%s $:", username,hostname);
        fgets(input, MAX_COMM_SIZE, stdin);

        /* NULL to `free` is a NO-OP */
        free(hist[current]);
        hist[current] = strdup(input);
        current = (current + 1) % HISTORY_COUNT;

        if (strcmp(input, "history\n") == 0)
           history(hist, current);
        else if (strcmp(input, "hc\n") == 0)
           clear_history(hist);

        executeCommand(input);

    }
}
