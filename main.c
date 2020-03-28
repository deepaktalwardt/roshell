#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/wait.h>
#include "const.h"
#include "variable.h"
#include "command.h"

int main(int argc, char *argv[], char* envp[]) {
    // argc - argument count
    // argv - argument vector
    // envp - environment pointer
    // TODO: Should roshell inherit envp from parent shells?

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

        printf("user@computer $:");
        fgets(input, MAX_COMM_SIZE, stdin);

        executeCommand(input);

    }
}
