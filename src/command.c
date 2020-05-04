#include "command.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>
#include "cd.h"
#include "const.h"
#include "source.h"
#include "variable.h"
#include "history.h"

void executeLine(char *input)
{

  // executes a line (which can be either a program call or a shell command)
  char *tokens[MAX_ARGS + 1] = {NULL}; // array to which we'll write tokens

  if (0 >= parseInput(input, tokens, MAX_TOK))
    return; // if empty, skip

  //add the entered command to the history list
  using_history();
  add_history(input);
  char *command = tokens[0];

  // try executing input as a shell commands
  // if command includes "=" set variable with the value after "=" as sting
  if (strchr(command, '=') != NULL)
  {
    addVariable(command, 0);
  }
  else if (strcmp(command, "export") == 0)
  {
    if (strchr(tokens[1], '=') != NULL)
    {
      addVariable(tokens[1], 1);
    }
  }
  else if (strcmp(command, "cd") == 0) // exit shell
  {
    changeDirectory(tokens[1]);
    /*  chdir(tokens[1]);
      // create copy not to corrupt input by strtok
      char* pwd = (char*)malloc(strlen(input) + 1);

      strcpy(input_copy, input);
      addVariable("PWD", getcwd(NULL, 0));*/
  }
  else if (strcmp(command, "exit") == 0) // exit shell
  {
    exit(0);
  }
  else if (strcmp(command, "env") == 0) // display all variables
  {
    displayVariable();
  }
  else if (strcmp(command, "source") == 0) // source command
  {
    printf("source command\n");
    sourceCommand(tokens);
  }
  else if (strcmp(command, "history") == 0)
  {
    printf("History List: \n");
    register HIST_ENTRY **the_list;
    register int i;
    the_list = history_list();
    if (the_list)
      for (i = 0; the_list[i]; i++)
      {
        printf("%s \n", the_list[i]->line);
      }
  }
  else if (strcmp(command, "hc") == 0)
  {
    clear_history();
    printf("History Cleared \n");
  }
  else // if the program is not a shell command, try executing as a program.
  {
    executeProgram(tokens);
  }
  wait(NULL);
}

int parseInput(char input[], char *tokens[], size_t max_tok)
{
  // original parsing from:
  // https://danrl.com/blog/2018/how-to-write-a-tiny-shell-in-c/ takes raw input
  // as a char array e.g. "ssh user@localhost -p 2222" writes the output to a
  // token array, e.g. {"ssh", "user@localhost", "-p", "2222"} return the number
  // of tokens written

  input[strlen(input) - 1] = '\0'; // terminate with null, rather than with \n

  char *token = strtok(input, " ");
  int n = 0;

  for (; token != NULL && n < max_tok; ++n)
  {
    tokens[n] = token;
    // if '$' is found, token is replaced with the value
    if (tokens[n][0] == '$')
    {
      tokens[n] = searchVariable(&tokens[n][1]);
    }
    token = strtok(NULL, " ");
  }
  if (tokens[0] == NULL)
    return -1; // empty input

  return n;
}

void executeProgram(char *tokens[])
{
  // executes program (such as /bin/ls, /usr/bin/git, etc. as a child process)
  if (fork() == 0) // if inside the child process
  {
    int comm_res = execvp(tokens[0], tokens);

    if (comm_res == -1) // execvp encountered error
    {
      printf("Command '%s' exited with the following error: %s \n", tokens[0],
             strerror(errno));
      exit(-1);
    }
    else
      exit(0);
  }
}
