#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/wait.h>

#define MAX_COMM_SIZE 255
#define MAX_ARGS 8

//------------------------------------------------------------------------------
//  sourceCommand()
//
//  Basic Functionality is to read a text file with list of shell commands
//  Improvement is needed to support shell scripting
//
//------------------------------------------------------------------------------
int sourceCommand(char** input)
{
    FILE  *sourceFile;
    char sourceLine[MAX_COMM_SIZE+1] = {0x0};

    sourceFile = fopen(input[1], "r");
    while(fgets(sourceLine, MAX_COMM_SIZE, sourceFile) != NULL)
    {
        //printf("%s", sourceLine);
        char* eachCommand[MAX_ARGS + 1] = { NULL };
        char* ptr = sourceLine;
        for (int i = 0; i < sizeof(sourceLine) && *ptr; ptr++)
        {
            if (*ptr == ' ') continue;
            if ((*ptr == '\n') || (*ptr == '\0')) break;
            for (eachCommand[i++] = ptr; *ptr && *ptr != ' ' && *ptr != '\n'; ptr++);
            *ptr = '\0';
        }

        if (fork() == 0) // if inside the child process
        {
            exit(execvp(eachCommand[0], eachCommand));
        }
        wait(NULL);
    }

    fclose(sourceFile);

    return 0;
}

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

        printf("user@computer $:");
        fgets(input, MAX_COMM_SIZE, stdin);
        input[strlen(input) - 1] = '\0'; //terminate with null, rather than with \n

        char* token = strtok(input, " ");
        for(int i=0; token != NULL && i<MAX_ARGS; i++) //I'm assuming we're not religious about ANSI C compatibility
        {
            args[i] = token;
            token = strtok(NULL, " ");
        }
        if(args[0] == NULL) continue;

        if (strcmp(args[0], "source") == 0)
        {
            printf("source command\n");
            sourceCommand(args);
        }
        else
        {

           if (fork() == 0) // if inside the child process
           {
               exit(execvp(args[0], args));
           }
        }
        wait(NULL);
    }
}
