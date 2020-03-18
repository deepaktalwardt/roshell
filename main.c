#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <sys/wait.h>

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
        char hostname[_SC_HOST_NAME_MAX];
        char username[_SC_LOGIN_NAME_MAX];
        gethostname(hostname,_SC_HOST_NAME_MAX); // system call to get the hostname
        getlogin_r(username,_SC_LOGIN_NAME_MAX); // system call to get the username

        printf("%s@%s $:", username,hostname);
        fgets(input, MAX_COMM_SIZE, stdin);
        input[strlen(input) - 1] = '\0'; //terminate with null, rather than with \n

        char* token = strtok(input, " ");
        for(int i=0; token != NULL && i<MAX_ARGS; i++) //I'm assuming we're not religious about ANSI C compatibility
        {
            args[i] = token;
            token = strtok(NULL, " ");
        }
        if(args[0] == NULL) continue;

        if (fork() == 0) // if inside the child process
        {
            exit(execvp(args[0], args));
        }
        wait(NULL);
    }
}
