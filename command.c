#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/wait.h>
#include "command.h"
#include "const.h"
#include "source.h"
#include "variable.h"

void executeCommand(char* input)
{

  char* args[MAX_ARGS + 1] = { NULL };
  input[strlen(input) - 1] = '\0'; //terminate with null, rather than with \n

  char* token = strtok(input, " ");
  for(int i=0; token != NULL && i<MAX_ARGS; i++) //I'm assuming we're not religious about ANSI C compatibility
  {
      args[i] = token;
      token = strtok(NULL, " ");
  }
  if(args[0] == NULL) return;

  // if command includes "=" set variable with the value after "=" as sting
  if (strchr(args[0], '=') != NULL)
  {
      //printf("variable set, %s\n", args[0]);
      addVariable(args[0]);
  }
  else if (strcmp(args[0], "exit") == 0) // exit shell
  {
      exit(0);
  }
  else if (strcmp(args[0], "env") == 0) // display all variables
  {
      displayVariable();
  }
  else if (strcmp(args[0], "source") == 0) // source command
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
