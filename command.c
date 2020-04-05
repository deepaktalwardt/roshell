#include "command.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/wait.h>
#include <unistd.h>
#include "const.h"
#include "source.h"
#include "variable.h"

void executeCommand(char* input) {
  char* args[MAX_ARGS + 1] = {NULL};
  input[strlen(input) - 1] = '\0';  // terminate with null, rather than with \n

  char* args[MAX_ARGS + 1] = { NULL };
  input[strlen(input) - 1] = '\0'; //terminate with null, rather than with \n
  
  char* token = strtok(input, " ");
  for(int i=0; token != NULL && i<MAX_ARGS; ++i)
  {
    args[i] = token;
    token = strtok(NULL, " ");
  }
  if(args[0] == NULL) return; //empty input

  // if command includes "=" set variable with the value after "=" as sting
  if (strchr(args[0], '=') != NULL) {
    // printf("variable set, %s\n", args[0]);
    addVariable(args[0]);
  } else if (strcmp(args[0], "exit") == 0)  // exit shell
  {
    exit(0);
  } else if (strcmp(args[0], "env") == 0)  // display all variables
  {
    displayVariable();
  } else if (strcmp(args[0], "source") == 0)  // source command
  {
    printf("source command\n");
    sourceCommand(args);
  } else {
    if (fork() == 0)  // if inside the child process
    {
      int comm_res = execvp(args[0], args);

     if (fork() == 0) // if inside the child process
     {
        int comm_res = execvp(args[0], args);

        if(comm_res == -1)  //execvp encountered error
        {
            printf("Command '%s' exited with the following error: %s \n", args[0], strerror(errno));  
            exit(-1);
        }
        else exit(0);
     }
  }
  wait(NULL);
}
