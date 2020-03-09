#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MAX_COMM_SIZE 255
#define MAX_ARGS 8

int main(int argc, char *argv[], char* envp[]) {
    // argc - argument count
    // argv - argument vector 
    // envp - environment pointer 
    // TODO: Should roshell inherit envp from parent shells?

    // print out all the environment variables
    for(int i = 0; envp[i]; ++i)
    {
        printf("environment variable %d: %s \n", i, envp[i]);
    }

    while (1)
    {
        char input[MAX_COMM_SIZE + 1] = { 0x0 };
        char* args[MAX_ARGS + 1] = { NULL };
        char* ptr = input;

        printf("user@computer $:");
        fgets(input, MAX_COMM_SIZE, stdin);
        input[strlen(input) - 1] = '\0'; //terminate with null, rather than with \n

        // taken from: https://danrl.com/blog/2018/how-to-write-a-tiny-shell-in-c/
        // TODO: use strtok() instead
        for (int i = 0; i < sizeof(args) && *ptr; ptr++) {
            if (*ptr == ' ') continue;
            if (*ptr == '\n') break;
            for (args[i++] = ptr; *ptr && *ptr != ' ' && *ptr != '\n'; ptr++);
            *ptr = '\0';
        }

        if (fork() == 0) // if inside the child process
        {
            exit(execvp(args[0], args));
        }
        wait(NULL);
    }
}


